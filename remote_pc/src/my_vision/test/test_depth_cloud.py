import numpy as np

from my_vision.depth_cloud import PixelRays, depth_to_points, make_cloud

K = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]


def test_center_pixel_is_on_axis():
    depth = np.zeros((480, 640), np.uint16)
    depth[240, 320] = 1500
    rays = PixelRays(K, 640, 480, stride=1)
    pts = depth_to_points(depth, rays, 0.2, 3.0)
    assert pts.shape == (1, 3)
    np.testing.assert_allclose(pts[0], [0.0, 0.0, 1.5], atol=1e-6)


def test_pinhole_offsets_and_range_filter():
    depth = np.zeros((480, 640), np.uint16)
    depth[240, 420] = 2000          # 오른쪽 100px, 2m → x = 100/500*2 = 0.4
    depth[340, 320] = 1000          # 아래 100px, 1m → y = 0.2
    depth[0, 0] = 100               # 0.1m: min_range 밖
    depth[1, 1] = 5000              # 5m: max_range 밖
    rays = PixelRays(K, 640, 480, stride=1)
    pts = depth_to_points(depth, rays, 0.2, 3.0)
    got = {tuple(round(float(v), 4) + 0.0 for v in p) for p in pts}
    assert got == {(0.4, 0.0, 2.0), (0.0, 0.2, 1.0)}


def test_stride_samples_grid_and_cloud_layout():
    depth = np.full((480, 640), 1000, np.uint16)
    rays = PixelRays(K, 640, 480, stride=4)
    pts = depth_to_points(depth, rays, 0.2, 3.0)
    assert len(pts) == 160 * 120
    from std_msgs.msg import Header
    msg = make_cloud(pts, Header(frame_id='cam'))
    assert msg.width == len(pts) and msg.point_step == 12 and len(msg.data) == 12 * len(pts)
