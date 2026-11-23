"""geometry.py · map_codec.py 단위 테스트 (ROS 불필요)."""
import math

import cv2
import numpy as np

from my_web_api.geometry import (compose, downsample, quaternion_from_yaw, scan_to_points,
                                 transform_points, yaw_from_quaternion)
from my_web_api.map_codec import MAP_FREE, MAP_OCCUPIED, MAP_UNKNOWN, costmap_to_rgba, encode_png, map_to_rgba


def test_yaw_roundtrip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.0):
        assert abs(yaw_from_quaternion(*quaternion_from_yaw(yaw)) - yaw) < 1e-9


def test_transform_rotate_90():
    # 자식 좌표계가 부모 기준 (1, 2) 에 있고 90° 돌아 있으면, 자식의 +x 1m 는 부모의 (1, 3)
    assert transform_points((1.0, 2.0, math.pi / 2), [(1.0, 0.0)]) == [[1.0, 3.0]]


def test_compose():
    x, y, th = compose((1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert abs(x - 1.0) < 1e-9 and abs(y - 1.0) < 1e-9 and abs(th - math.pi / 2) < 1e-9


def test_downsample_keeps_last():
    seq = list(range(1000))
    out = downsample(seq, 100)
    assert len(out) == 100 and out[0] == 0 and out[-1] == 999
    assert downsample([1, 2, 3], 10) == [1, 2, 3]


def test_scan_to_points_filters_invalid():
    ranges = [1.0, float('inf'), 0.05, float('nan'), 2.0]
    pts = scan_to_points(ranges, 0.0, math.pi / 2, 0.12, 3.5, max_points=10)
    assert len(pts) == 2
    assert abs(pts[0][0] - 1.0) < 1e-9 and abs(pts[1][0] - 2.0) < 1e-9   # 0° 와 360°(index 4) 모두 +x 방향

def test_map_rgba_flip_and_colors():
    data = np.array([[0, 100], [-1, 0]], dtype=np.int8)   # 0번 행 = 지도 아래쪽
    img = map_to_rgba(data)
    assert (img[1, 0] == MAP_FREE).all() and (img[1, 1] == MAP_OCCUPIED).all()   # 뒤집혀서 이미지 아래 행
    assert (img[0, 0] == MAP_UNKNOWN).all()


def test_costmap_rgba_transparent_free():
    data = np.array([[0, -1, 50, 99, 100]], dtype=np.int8)
    img = costmap_to_rgba(data)
    assert img[0, 0, 3] == 0 and img[0, 1, 3] == 0 and img[0, 2, 3] > 0
    assert tuple(img[0, 3, :3]) == (0, 229, 255) and tuple(img[0, 4, :3]) == (255, 0, 212)


def test_png_roundtrip():
    img = map_to_rgba(np.array([[0, 100, -1]], dtype=np.int8))
    back = cv2.imdecode(np.frombuffer(encode_png(img), np.uint8), cv2.IMREAD_UNCHANGED)
    assert back.shape == (1, 3, 4)
    assert tuple(back[0, 1, :3][::-1]) == tuple(MAP_OCCUPIED[:3])


def test_align_to_axes_keeps_world_position():
    import math
    import numpy as np
    from my_web_api.map_codec import align_to_axes
    r = 0.05
    g = np.zeros((60, 60), np.int8)
    g[10, 40] = 100                                   # 행 10, 열 40 → 로컬 (40.5r, 10.5r)
    origin = (1.0, -0.5, math.radians(13))
    out, (x0, y0, yaw) = align_to_axes(g, r, origin, snap=(0.0, 0.0))
    assert yaw == 0.0
    assert abs(round(x0 / r) * r - x0) < 1e-9 and abs(round(y0 / r) * r - y0) < 1e-9   # 지도 칸 경계에 맞춤
    lx, ly = 40.5 * r, 10.5 * r
    c, s = math.cos(origin[2]), math.sin(origin[2])
    wx, wy = origin[0] + c * lx - s * ly, origin[1] + s * lx + c * ly
    jj, ii = np.nonzero(out == 100)
    cx, cy = x0 + (ii + 0.5) * r, y0 + (jj + 0.5) * r
    assert len(ii) >= 1 and np.min(np.hypot(cx - wx, cy - wy)) < r      # 같은 자리 (한 칸 안)
    assert out[0, 0] == -1                                               # 원래 격자 바깥 = 미지
    same, o2 = align_to_axes(g, r, (1.0, -0.5, 0.0))
    assert same is g and o2 == (1.0, -0.5, 0.0)                          # 회전 없으면 그대로
