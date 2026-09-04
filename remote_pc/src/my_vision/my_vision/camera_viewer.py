# =============================================================================
# camera_viewer.py — 카메라 토픽을 눈으로 확인하는 뷰어 (Remote PC 컨테이너)
#
# [ color | 정렬 depth(컬러맵) | 오버레이 ] 를 한 창에 보여준다. 오버레이로 depth 가 color 에
# 제대로 정렬됐는지(윤곽이 겹치는지) 검증하고, 중앙 십자선 위치의 거리(m)를 표시한다.
# --color-topic 을 /vision/annotated/compressed 로 주면 검출 결과가 그려진 영상을 본다.
#
# [실행] 컨테이너에서 (창은 서버 물리 세션에 뜨고 VNC 로 본다)
#   export DISPLAY=:0
#   ros2 run my_vision camera_viewer                                   # q 로 종료
#   ros2 run my_vision camera_viewer --color-topic /vision/annotated/compressed
#   ros2 run my_vision camera_viewer --snapshot /tmp/cam.jpg           # 창 없이 한 장 저장
# =============================================================================
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from my_vision.camera_io import FramePairer, colorize_depth, decode_color, decode_depth_mm


class CameraViewer(Node):

    def __init__(self, color_topic, snapshot=None):
        super().__init__('camera_viewer')
        self.snapshot = snapshot
        self.pairer = FramePairer()
        self.n = 0
        self.recent = []             # 최근 프레임 도착 시각 (슬라이딩 창 fps)
        self.last_frame = None
        self.dirty = False           # 새 프레임이 있을 때만 다시 그림 (imshow 는 비싸다)
        qos = qos_profile_sensor_data   # 퍼블리셔와 동일 (best effort) — 아니면 매칭 안 됨
        self.create_subscription(CompressedImage, color_topic, lambda m: self._on_msg('color', m), qos)
        self.create_subscription(CompressedImage, '/camera/depth/compressed',
                                 lambda m: self._on_msg('depth', m), qos)
        if snapshot is None:
            self.create_timer(1 / 30, self._gui_tick)
        self.get_logger().info(f'구독: {color_topic} + /camera/depth/compressed (best effort)')

    def _on_msg(self, kind, msg):
        pair = self.pairer.add(kind, msg)
        if pair is not None:
            self._render(decode_color(pair[0]), decode_depth_mm(pair[1]))

    def _render(self, color, depth_mm):
        h, w = depth_mm.shape
        cx, cy = w // 2, h // 2
        depth_vis = colorize_depth(depth_mm)
        overlay = cv2.addWeighted(color, 0.55, depth_vis, 0.45, 0)

        patch = depth_mm[cy - 1:cy + 2, cx - 1:cx + 2]          # 중앙 3x3 중앙값
        valid = patch[patch > 0]
        dist_txt = f'{np.median(valid) / 1000:.2f} m' if valid.size else 'N/A'
        for img in (color, depth_vis, overlay):
            cv2.drawMarker(img, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.putText(overlay, f'center: {dist_txt}', (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        self.n += 1
        now = time.monotonic()
        self.recent = [t for t in self.recent if now - t <= 2.0] + [now]
        fps = len(self.recent) / 2.0
        for img, name in ((color, 'color'), (depth_vis, 'aligned depth'), (overlay, 'overlay')):
            cv2.putText(img, name, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(color, f'{fps:.1f} fps', (w - 110, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        self.last_frame = np.hstack([color, depth_vis, overlay])
        self.dirty = True
        if self.n % 30 == 1:
            valid_pct = 100 * np.count_nonzero(depth_mm) / depth_mm.size
            self.get_logger().info(f'{fps:.1f} fps | depth 유효 {valid_pct:.0f}% | 중앙 {dist_txt}')
        if self.snapshot and self.n >= 5:      # 몇 장 흘려보낸 뒤 저장 (노출 안정)
            cv2.imwrite(self.snapshot, self.last_frame)
            self.get_logger().info(f'스냅샷 저장: {self.snapshot}')
            raise SystemExit(0)

    def _gui_tick(self):
        if self.dirty and self.last_frame is not None:
            cv2.imshow('rs_camera viewer  [q: quit]', self.last_frame)
            self.dirty = False
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--color-topic', default='/camera/color/compressed',
                    help='예: /vision/annotated/compressed (검출 결과 영상)')
    ap.add_argument('--snapshot', help='한 장을 이 경로에 저장하고 종료 (창 없이)')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = CameraViewer(args.color_topic, args.snapshot)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
