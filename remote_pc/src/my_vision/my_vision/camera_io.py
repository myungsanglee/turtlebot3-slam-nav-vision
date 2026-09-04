# =============================================================================
# camera_io.py — 로봇 카메라 토픽을 소비하는 공용 유틸 (뷰어·검출 노드가 공유)
#
# 입력 계약 (docs/realsense_bringup.md 3장):
#   /camera/color/compressed   JPEG            → decode_color()    : HxWx3 BGR uint8
#   /camera/depth/compressed   PNG 16bit (mm)  → decode_depth_mm() : HxW uint16, 0 = 측정 없음
#   /camera/color/camera_info  color 인트린식  → K 행렬로 픽셀+거리 → 3D 점 (deproject)
#   depth 는 color 에 정렬돼 있고 세 토픽의 stamp 가 같다 → FramePairer 로 짝을 맞춘다.
# =============================================================================
import cv2
import numpy as np


def decode_color(msg):
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


def decode_depth_mm(msg):
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)


def stamp_key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


class FramePairer:
    """같은 stamp 의 color/depth 메시지를 짝지어 돌려준다 (best effort 라 한쪽이 유실될 수 있음)."""

    def __init__(self, max_pending=30):
        self._pending = {}
        self._max = max_pending

    def add(self, kind, msg):
        key = stamp_key(msg)
        slot = self._pending.setdefault(key, {})
        slot[kind] = msg
        if 'color' in slot and 'depth' in slot:
            for k in [k for k in self._pending if k <= key]:   # 이 stamp 이전 미완 항목은 버림
                del self._pending[k]
            return slot['color'], slot['depth']
        if len(self._pending) > self._max:                     # 메모리 보호
            for k in sorted(self._pending)[:-10]:
                del self._pending[k]
        return None


def colorize_depth(depth_mm, min_m=0.3, max_m=4.0):
    """mm depth → 컬러맵 (가까움=빨강, 멂=파랑, 측정 없음=검정). 사람이 보기 위한 용도."""
    d = depth_mm.astype(np.float32) / 1000.0
    norm = np.clip((d - min_m) / (max_m - min_m), 0, 1)
    img = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[depth_mm == 0] = 0
    return img


def depth_in_box(depth_mm, x1, y1, x2, y2, frac=0.5, min_mm=200, max_mm=8000):
    """박스 중앙 frac 영역의 유효 depth 중앙값(mm). 없으면 None.
    가장자리는 배경이 섞이기 쉬워 중앙만 쓰고, 0(측정 없음)과 비현실적 값은 제외한다."""
    h, w = depth_mm.shape
    bw, bh = x2 - x1, y2 - y1
    mx, my = bw * (1 - frac) / 2, bh * (1 - frac) / 2
    xa, xb = int(np.clip(x1 + mx, 0, w - 1)), int(np.clip(x2 - mx, 1, w))
    ya, yb = int(np.clip(y1 + my, 0, h - 1)), int(np.clip(y2 - my, 1, h))
    roi = depth_mm[ya:yb, xa:xb]
    valid = roi[(roi >= min_mm) & (roi <= max_mm)]
    return float(np.median(valid)) if valid.size else None


def deproject(u, v, z_m, K):
    """color 픽셀 (u,v) + 거리 z(m) → 카메라 광학 좌표계 3D 점 (m).
    K = [fx 0 ppx; 0 fy ppy; 0 0 1] (CameraInfo.k). 광학 좌표계: z 앞, x 오른쪽, y 아래."""
    fx, fy, ppx, ppy = K[0], K[4], K[2], K[5]
    return ((u - ppx) / fx * z_m, (v - ppy) / fy * z_m, z_m)
