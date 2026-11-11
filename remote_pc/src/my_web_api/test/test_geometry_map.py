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
