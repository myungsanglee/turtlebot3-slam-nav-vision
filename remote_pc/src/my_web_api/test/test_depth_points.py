import math

import numpy as np

from my_web_api.depth_points import PixelRays, depth_to_display, quat_matrix, voxel_thin

# 카메라 광학 좌표계(z 앞, x 오른쪽, y 아래) → 로봇(x 앞, y 왼쪽, z 위): TF 실측값의 회전 (RPY -90°, 0, -90°)
Q_OPT = (-0.5, 0.5, -0.5, 0.5)
K = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]


def test_quat_identity_and_optical_axes():
    np.testing.assert_allclose(quat_matrix(0, 0, 0, 1), np.eye(3), atol=1e-6)
    R = quat_matrix(*Q_OPT)
    np.testing.assert_allclose(R @ [0, 0, 1], [1, 0, 0], atol=1e-6)    # 광학 z(앞) → 로봇 x(앞)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-6)   # 광학 x(오른쪽) → 로봇 -y
    np.testing.assert_allclose(R @ [0, 1, 0], [0, 0, -1], atol=1e-6)   # 광학 y(아래) → 로봇 -z


def test_voxel_thin_keeps_one_per_cell():
    pts = np.array([[0.01, 0.01, 0.0], [0.02, 0.03, 0.01], [0.07, 0.0, 0.0]], np.float32)
    assert len(voxel_thin(pts, 0.05)) == 2


def test_depth_to_display_floor_and_obstacle_heights():
    # 카메라 높이 0.07m, 수평. 이미지 중앙 열: 정면 1m 에 물체(중앙 픽셀), 아래쪽 픽셀은 바닥
    R, t = quat_matrix(*Q_OPT), np.array([0.05, 0.0, 0.07], np.float32)
    depth = np.zeros((480, 640), np.uint16)
    depth[240, 320] = 1000                         # 정면 1m → 높이 0.07
    v = 240 + int(round(0.07 / 0.5 * 500))         # z=0.5m 거리의 바닥이 보이는 행: y = 0.07 = (v-240)/500*0.5
    depth[v, 320] = 500
    rays = PixelRays(K, 640, 480, stride=1)
    pts = depth_to_display(depth, rays, R, t, 0.2, 3.0, 0.05, 1.0)
    got = sorted(pts)
    assert got[0] == [0.55, 0.0, 0.0] and got[1] == [1.05, 0.0, 0.07]
    assert math.isclose(len(pts), 2)
