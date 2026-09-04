#!/usr/bin/env python3
# =============================================================================
# camera_viewer.py — 로봇 카메라 토픽을 눈으로 확인하는 뷰어 (Remote PC 컨테이너)
#
# realsense_bringup 의 자체 노드가 보내는 세 토픽을 구독해 디코드하고 한 창에 보여준다:
#   [ color | 정렬 depth(컬러맵) | 오버레이 ]
# 오버레이는 depth 가 color 에 제대로 정렬됐는지(물체 윤곽이 겹치는지) 눈으로 검증하는 용도.
# 중앙 십자선 위치의 거리(m)를 표시해 depth 값이 실제 거리인지도 바로 확인할 수 있다.
#
# 이 파일의 디코드 부분이 곧 Vision 노드의 입력 처리 코드다 (docs/realsense_bringup.md 3장 계약):
#   color : JPEG            → cv2.imdecode(..., IMREAD_COLOR)      → HxWx3 BGR
#   depth : PNG 16bit (mm)  → cv2.imdecode(..., IMREAD_UNCHANGED)  → HxW uint16, 0 = 측정 없음
#   두 이미지는 header.stamp 가 같으므로 stamp 로 짝을 맞춘다.
#
# [실행] 컨테이너에서 (DISPLAY 는 서버 물리 세션, VNC 로 본다)
#   python3 /overlay_ws/src/my_vision/tools/camera_viewer.py                 # 라이브 창, q 로 종료
#   python3 /overlay_ws/src/my_vision/tools/camera_viewer.py --snapshot /tmp/cam.jpg   # 한 장 저장 후 종료
# =============================================================================
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

DEPTH_MIN_M, DEPTH_MAX_M = 0.3, 4.0     # 컬러맵 범위 (D435i 실용 범위)


def decode_color(msg):
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


def decode_depth_mm(msg):
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)   # uint16, mm


def colorize_depth(depth_mm):
    """mm depth → 보기 좋은 컬러맵 (가까움=빨강, 멂=파랑, 측정 없음=검정)."""
    d = depth_mm.astype(np.float32) / 1000.0
    norm = np.clip((d - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M), 0, 1)
    img = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[depth_mm == 0] = 0
    return img


class CameraViewer(Node):

    def __init__(self, snapshot=None):
        super().__init__('camera_viewer')
        self.snapshot = snapshot
        self.pending = {}            # stamp → {'color': img, 'depth': img}
        self.n, self.t0 = 0, time.monotonic()
        self.last_frame = None
        qos = qos_profile_sensor_data   # 퍼블리셔와 동일 (best effort) — 아니면 매칭 안 됨
        self.create_subscription(CompressedImage, '/camera/color/compressed',
                                 lambda m: self._on_msg('color', m), qos)
        self.create_subscription(CompressedImage, '/camera/depth/compressed',
                                 lambda m: self._on_msg('depth', m), qos)
        if snapshot is None:
            self.create_timer(1 / 30, self._gui_tick)
        self.get_logger().info('구독 시작 — 프레임 대기 중 (퍼블리셔 QoS: best effort)')

    def _on_msg(self, kind, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        slot = self.pending.setdefault(key, {})
        slot[kind] = decode_color(msg) if kind == 'color' else decode_depth_mm(msg)
        if 'color' in slot and 'depth' in slot:
            self._render(slot['color'], slot['depth'])
            # 이 stamp 이전 것들은 짝이 안 맞은 것 → 버림 (메모리 누수 방지)
            for k in [k for k in self.pending if k <= key]:
                del self.pending[k]

    def _render(self, color, depth_mm):
        h, w = depth_mm.shape
        cx, cy = w // 2, h // 2
        depth_vis = colorize_depth(depth_mm)
        overlay = cv2.addWeighted(color, 0.55, depth_vis, 0.45, 0)

        # 중앙 십자선 + 그 지점 거리 (3x3 중앙값으로 0 노이즈 완화)
        patch = depth_mm[cy - 1:cy + 2, cx - 1:cx + 2]
        valid = patch[patch > 0]
        dist_txt = f'{np.median(valid) / 1000:.2f} m' if valid.size else 'N/A'
        for img in (color, depth_vis, overlay):
            cv2.drawMarker(img, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.putText(overlay, f'center: {dist_txt}', (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        self.n += 1
        fps = self.n / max(time.monotonic() - self.t0, 1e-6)
        for img, name in ((color, 'color'), (depth_vis, 'aligned depth'), (overlay, 'overlay')):
            cv2.putText(img, name, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(color, f'{fps:.1f} fps', (w - 110, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        self.last_frame = np.hstack([color, depth_vis, overlay])
        if self.n % 30 == 1:
            valid_pct = 100 * np.count_nonzero(depth_mm) / depth_mm.size
            self.get_logger().info(f'{fps:.1f} fps | depth 유효 {valid_pct:.0f}% | 중앙 {dist_txt}')
        if self.snapshot and self.n >= 5:      # 몇 장 흘려보낸 뒤 저장 (노출 안정)
            cv2.imwrite(self.snapshot, self.last_frame)
            self.get_logger().info(f'스냅샷 저장: {self.snapshot}')
            raise SystemExit(0)

    def _gui_tick(self):
        if self.last_frame is not None:
            cv2.imshow('rs_camera viewer  [q: quit]', self.last_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--snapshot', help='한 장을 이 경로에 저장하고 종료 (창 없이)')
    args = ap.parse_args()
    rclpy.init()
    node = CameraViewer(args.snapshot)
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
