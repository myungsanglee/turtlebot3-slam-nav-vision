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
# [설정] config/vision_params.yaml 이 단일 소스 (launch 가 로드). 아래 declare 의 기본값은 YAML 없이
#   ros2 run 으로 띄울 때의 안전한 기본값이며, 의미 설명은 YAML 주석에 있다.
# [모델] RF-DETR (Apache-2.0) — rfdetr 1.9 의 Nano/Small/Medium/Large. weights='' 면 공식 COCO
#   사전학습(RF_HOME=weights_dir 에 자동 다운로드·캐시), 경로를 주면 그 .pth(전이학습 모델)를 로드하고
#   클래스 수는 체크포인트에서 자동 추론. 클래스명: 체크포인트 → class_names 파라미터 → COCO 표.
# [백엔드] backend:=torch(기본, rfdetr predict) | tensorrt(엔진; 없으면 이 컨테이너에서 빌드·캐시)
#   — backends.py 참고. 둘 다 같은 출력 계약.
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

from my_vision.backends import TensorRTBackend, TorchBackend
from my_vision.camera_io import FramePairer, decode_color, decode_depth_mm, deproject, depth_in_box


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector')
        self.declare_parameter('model', 'medium')          # RF-DETR 크기: nano | small | medium | large
        self.declare_parameter('weights', '')              # '' = 공식 사전학습 / 경로 = 사용자 .pth
        self.declare_parameter('class_names', [''])        # 체크포인트에 이름 없을 때 id 순 지정
        self.declare_parameter('backend', 'torch')         # torch | tensorrt
        # --- tensorrt 빌드 옵션 (엔진이 없을 때 첫 실행에서 사용; trt_build.py 참고) ---
        self.declare_parameter('trt_precision', 'fp16')    # fp32 | fp16 | int8 (int8 은 trt_calib_dir 필요)
        self.declare_parameter('trt_opt_level', 3)         # 0(빠른 빌드) ~ 5(최선 엔진)
        self.declare_parameter('trt_workspace_gib', 4.0)
        self.declare_parameter('trt_tf32', True)
        self.declare_parameter('trt_timing_cache', True)   # 재빌드 가속 캐시 파일 사용
        self.declare_parameter('trt_calib_dir', '')        # INT8 캘리브레이션 이미지 디렉터리
        self.declare_parameter('trt_verify', True)         # 빌드 후 ONNX Runtime 대비 검증 로그
        self.declare_parameter('weights_dir', '/overlay_ws/models')
        self.declare_parameter('threshold', 0.5)           # 검출 점수 임계값
        self.declare_parameter('resolution', 0)            # RF-DETR 입력 해상도. 0 = 모델 기본값 (크기별로 다름)
        self.declare_parameter('depth_roi_frac', 0.5)      # 거리 계산에 쓰는 박스 중앙 비율
        self.declare_parameter('class_filter', [''])       # 비우면 전체, 예: ['person','chair']
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('log_period_sec', 5.0)
        self.declare_parameter('color_topic', '/camera/color/compressed')
        self.declare_parameter('depth_topic', '/camera/depth/compressed')
        self.declare_parameter('info_topic', '/camera/color/camera_info')

        p = self.get_parameter
        self.threshold = p('threshold').value
        self.roi_frac = p('depth_roi_frac').value
        self.class_filter = {c for c in p('class_filter').value if c}
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, p('jpeg_quality').value]
        self.log_period = p('log_period_sec').value

        trt_opts = dict(precision=p('trt_precision').value, opt_level=p('trt_opt_level').value,
                        workspace_gib=p('trt_workspace_gib').value, tf32=p('trt_tf32').value,
                        timing_cache=p('trt_timing_cache').value, calib_dir=p('trt_calib_dir').value,
                        verify=p('trt_verify').value)
        self.backend, self.class_names = self._load_model(
            p('model').value, p('backend').value, Path(p('weights_dir').value),
            p('resolution').value, trt_opts, p('weights').value,
            [c for c in p('class_names').value if c])

        qos = qos_profile_sensor_data
        self.pub_det = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.pub_img = self.create_publisher(CompressedImage, '/vision/annotated/compressed', qos)
        self.pairer = FramePairer()
        self.K = None
        self.create_subscription(CameraInfo, p('info_topic').value, self._on_info, qos)
        self.create_subscription(CompressedImage, p('color_topic').value,
                                 lambda m: self._on_msg('color', m), qos)
        self.create_subscription(CompressedImage, p('depth_topic').value,
                                 lambda m: self._on_msg('depth', m), qos)
        self.n = 0
        self.t_log = time.monotonic()
        self.infer_ms = []
        self.get_logger().info('구독 시작 — 카메라 프레임 대기 중')

    # ------------------------------------------------------------------ 모델
    def _load_model(self, size, backend, weights_dir, resolution, trt_opts, weights, class_names):
        weights_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault('RF_HOME', str(weights_dir))   # rfdetr 가중치 캐시 위치 (import 전에)
        import torch
        import rfdetr
        from rfdetr.assets.coco_classes import COCO_CLASSES
        cls = {'nano': rfdetr.RFDETRNano, 'small': rfdetr.RFDETRSmall,
               'medium': rfdetr.RFDETRMedium, 'large': rfdetr.RFDETRLarge}[size]
        kwargs = {}
        if resolution > 0:
            kwargs['resolution'] = resolution
        if weights:
            if not Path(weights).is_file():
                raise FileNotFoundError(f'weights 파일 없음: {weights}')
            kwargs['pretrain_weights'] = weights          # 사용자 체크포인트 (클래스 수는 체크포인트에서 추론)
        self.get_logger().info(
            f'RF-DETR {size} 로드 (device: {"cuda" if torch.cuda.is_available() else "cpu"}, '
            f'가중치: {weights or "공식 COCO 사전학습 (캐시 " + str(weights_dir) + ")"})')
        model = cls(**kwargs)
        res = getattr(getattr(model, 'model', None), 'resolution', resolution or 576)
        # 워밍업: 첫 추론은 커널 초기화로 느리다 → 실제 프레임 전에 한 번 돌려둔다
        model.predict(np.zeros((res, res, 3), np.uint8), threshold=self.threshold)
        # 클래스명 우선순위: 체크포인트 저장값 → class_names 파라미터 → COCO 표(dict id→name)
        ckpt_names = getattr(getattr(model, 'model', None), 'class_names', None)
        if ckpt_names:
            names, src = {i: n for i, n in enumerate(ckpt_names)}, '체크포인트'
        elif class_names:
            names, src = {i: n for i, n in enumerate(class_names)}, 'class_names 파라미터'
        else:
            names, src = dict(COCO_CLASSES), 'COCO 표'
        self.get_logger().info(f'모델 준비 완료 (입력 {res}px, 클래스 {len(names)}개 — {src})')
        if backend == 'tensorrt':
            # 엔진 캐시 키에 체크포인트 정체성을 넣는다 (파인튜닝 모델과 COCO 모델의 엔진이 섞이지 않게)
            ckpt_tag = 'coco' if not weights else f'{Path(weights).stem}-{_file_tag(weights)}'
            be = TensorRTBackend(model, size, res, len(names), weights_dir, self.threshold, trt_opts,
                                 ckpt_tag=ckpt_tag, log=self.get_logger().info)
            del model                       # 엔진이 있으니 torch 모델은 GPU 에서 내린다
            torch.cuda.empty_cache()
        else:
            be = TorchBackend(model, self.threshold)
        self.get_logger().info(f'백엔드: {be.name}')
        return be, names

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
        xyxy, confs, cids = self.backend.infer(rgb)
        self.infer_ms.append((time.monotonic() - t0) * 1000)

        out = Detection2DArray()
        out.header = color_msg.header
        for (x1, y1, x2, y2), score, cid in zip(xyxy, confs, cids):
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


def _file_tag(path, n_bytes=4 << 20):
    """파일 크기 + 앞 4MB 의 md5 앞 8자리 — 같은 이름의 다른 체크포인트를 구분하는 짧은 태그."""
    import hashlib
    h = hashlib.md5(str(os.path.getsize(path)).encode())
    with open(path, 'rb') as f:
        h.update(f.read(n_bytes))
    return h.hexdigest()[:8]


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
