# =============================================================================
# detector_node.py — RF-DETR 물체 검출 + 정렬 depth 로 거리·3D 위치 (Remote PC, GPU)
#
# [흐름]
#   /camera/color/compressed ─┐  stamp 로 짝 맞춤   ┌─ RF-DETR (GPU) → 박스·클래스·점수
#   /camera/depth/compressed ─┴─ FramePairer ──────┤
#   /camera/color/camera_info ─ K 행렬 (최신 것 보관) └─ 박스 중앙 영역 depth 중앙값 → 거리
#                                                     → K 로 deproject → 카메라 광학 좌표 3D 점
#   출력
#     /vision/detections            vision_msgs/Detection2DArray
#         bbox: 픽셀 박스, results[0].hypothesis: class_id(이름)·score,
#         results[0].pose.pose.position: (x,y,z) [m], frame_id = camera_color_optical_frame
#         (z 가 0 이면 그 박스에서 유효 depth 를 못 구한 것)
#     /vision/annotated/compressed  박스·라벨·거리를 그린 JPEG (camera_viewer 로 확인)
#
# [왜 depth 로 거리를 이렇게 구하나]
#   depth 가 color 에 정렬돼 있어 검출 박스의 픽셀을 그대로 depth 에 대면 된다. 박스 전체는
#   배경이 섞이므로 중앙 50% 영역만 쓰고, 0(측정 없음)·비현실 값을 뺀 중앙값을 쓴다
#   (평균은 배경/구멍에 끌려간다). 3D 점은 아직 카메라 좌표 — base_link 로 옮기는 TF 는
#   camera_link 실측 후 URDF 에 추가 (CLAUDE.md 5번).
#
# [모델] RF-DETR (Apache-2.0, COCO 사전학습) — rfdetr 1.9 의 Nano/Small/Medium/Large.
#   가중치는 RF_HOME(=weights_dir) 에 없으면 첫 실행 때 자동 다운로드되어 캐시된다.
#   TensorRT 변환은 다음 단계 (ONNX export 지원).
# =============================================================================
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from my_vision.camera_io import FramePairer, decode_color, decode_depth_mm, deproject, depth_in_box


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector')
        self.declare_parameter('model', 'medium')          # RF-DETR 크기: nano | small | medium | large
        self.declare_parameter('weights_dir', '/overlay_ws/models')
        self.declare_parameter('threshold', 0.5)           # 검출 점수 임계값
        self.declare_parameter('resolution', 0)            # RF-DETR 입력 해상도. 0 = 모델 기본값 (크기별로 다름)
        self.declare_parameter('depth_roi_frac', 0.5)      # 거리 계산에 쓰는 박스 중앙 비율
        self.declare_parameter('class_filter', [''])       # 비우면 전체, 예: ['person','chair']
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('log_period_sec', 5.0)

        p = self.get_parameter
        self.threshold = p('threshold').value
        self.roi_frac = p('depth_roi_frac').value
        self.class_filter = {c for c in p('class_filter').value if c}
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, p('jpeg_quality').value]
        self.log_period = p('log_period_sec').value

        self.model, self.class_names = self._load_model(
            p('model').value, Path(p('weights_dir').value), p('resolution').value)

        qos = qos_profile_sensor_data
        self.pub_det = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.pub_img = self.create_publisher(CompressedImage, '/vision/annotated/compressed', qos)
        self.pairer = FramePairer()
        self.K = None
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._on_info, qos)
        self.create_subscription(CompressedImage, '/camera/color/compressed',
                                 lambda m: self._on_msg('color', m), qos)
        self.create_subscription(CompressedImage, '/camera/depth/compressed',
                                 lambda m: self._on_msg('depth', m), qos)
        self.n = 0
        self.t_log = time.monotonic()
        self.infer_ms = []
        self.get_logger().info('구독 시작 — 카메라 프레임 대기 중')

    # ------------------------------------------------------------------ 모델
    def _load_model(self, size, weights_dir, resolution):
        weights_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault('RF_HOME', str(weights_dir))   # rfdetr 가중치 캐시 위치 (import 전에)
        import torch
        import rfdetr
        from rfdetr.assets.coco_classes import COCO_CLASSES
        cls = {'nano': rfdetr.RFDETRNano, 'small': rfdetr.RFDETRSmall,
               'medium': rfdetr.RFDETRMedium, 'large': rfdetr.RFDETRLarge}[size]
        self.get_logger().info(
            f'RF-DETR {size} 로드 (device: {"cuda" if torch.cuda.is_available() else "cpu"}, '
            f'가중치 캐시: {weights_dir} — 없으면 자동 다운로드)')
        model = cls(resolution=resolution) if resolution > 0 else cls()
        res = getattr(getattr(model, 'model', None), 'resolution', resolution or 576)
        # 워밍업: 첫 추론은 커널 초기화로 느리다 → 실제 프레임 전에 한 번 돌려둔다
        model.predict(np.zeros((res, res, 3), np.uint8), threshold=self.threshold)
        # 클래스명: 체크포인트가 알려주면 그것(리스트), 아니면 COCO 표(dict id→name)
        ckpt_names = getattr(getattr(model, 'model', None), 'class_names', None)
        names = {i: n for i, n in enumerate(ckpt_names)} if ckpt_names else dict(COCO_CLASSES)
        self.get_logger().info(f'모델 준비 완료 (입력 {res}px, 클래스 {len(names)}개)')
        return model, names

    # ------------------------------------------------------------------ 콜백
    def _on_info(self, msg):
        self.K = list(msg.k)

    def _on_msg(self, kind, msg):
        pair = self.pairer.add(kind, msg)
        if pair is not None:
            self._process(pair[0], pair[1])

    def _process(self, color_msg, depth_msg):
        color = decode_color(color_msg)
        depth_mm = decode_depth_mm(depth_msg)
        if color is None or depth_mm is None:
            return

        t0 = time.monotonic()
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)   # ([:, :, ::-1] 은 음수 stride 뷰라 torch 가 거부)
        dets = self.model.predict(rgb, threshold=self.threshold)
        self.infer_ms.append((time.monotonic() - t0) * 1000)

        out = Detection2DArray()
        out.header = color_msg.header
        for (x1, y1, x2, y2), score, cid in zip(dets.xyxy, dets.confidence, dets.class_id):
            name = self.class_names.get(int(cid), str(cid))
            if self.class_filter and name not in self.class_filter:
                continue
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            z_mm = depth_in_box(depth_mm, x1, y1, x2, y2, self.roi_frac)

            d = Detection2D()
            d.header = color_msg.header
            d.id = name
            d.bbox.center.position.x, d.bbox.center.position.y = float(cx), float(cy)
            d.bbox.size_x, d.bbox.size_y = float(x2 - x1), float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = float(score)
            if z_mm is not None and self.K is not None:
                X, Y, Z = deproject(cx, cy, z_mm / 1000.0, self.K)
                hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = X, Y, Z
            d.results.append(hyp)
            out.detections.append(d)

            label = f'{name} {score:.2f}' + (f' | {z_mm / 1000:.2f}m' if z_mm else ' | ?m')
            x1i, y1i, x2i, y2i = map(int, (x1, y1, x2, y2))
            cv2.rectangle(color, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
            cv2.putText(color, label, (x1i, max(y1i - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        self.pub_det.publish(out)
        ok, jpg = cv2.imencode('.jpg', color, self.jpeg_params)
        if ok:
            img = CompressedImage()
            img.header = color_msg.header
            img.format = 'jpeg'
            img.data = jpg.tobytes()
            self.pub_img.publish(img)

        self.n += 1
        now = time.monotonic()
        if now - self.t_log >= self.log_period:
            fps = self.n / (now - self.t_log)
            ms = np.mean(self.infer_ms) if self.infer_ms else 0
            summary = ', '.join(
                f'{d.id} {d.results[0].pose.pose.position.z:.2f}m' for d in out.detections[:4]) or '없음'
            self.get_logger().info(f'{fps:.1f} fps | 추론 {ms:.0f} ms | 검출 {len(out.detections)}: {summary}')
            self.n, self.infer_ms, self.t_log = 0, [], now


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
