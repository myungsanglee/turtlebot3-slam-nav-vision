"""robot_model.py 단위 테스트 (ROS 불필요): URDF 해석, STL 읽기, 배치, 위에서 본 이미지."""
import math
import struct

import numpy as np

from my_web_api.robot_model import base_link_visual, joint_offset, place, read_stl, render_top_view, rpy_matrix

URDF = '''<robot name="t">
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0.0 0.0 0.010" rpy="0 0 0"/></joint>
  <link name="base_link"><visual><origin xyz="0.1 0 0" rpy="0 0 1.5707963"/>
    <geometry><mesh filename="package://p/m.stl" scale="0.001 0.001 0.001"/></geometry></visual></link>
</robot>'''


def test_urdf_parse():
    v = base_link_visual(URDF)
    assert v['filename'] == 'package://p/m.stl' and v['scale'] == [0.001] * 3 and abs(v['rpy'][2] - math.pi / 2) < 1e-6
    assert joint_offset(URDF) == (0.0, 0.0, 0.01)


def test_rpy_yaw_90():
    r = rpy_matrix(0, 0, math.pi / 2)
    assert np.allclose(r @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-6)


def test_place_scale_rotate_offset():
    tri = np.array([[[1000, 0, 0], [0, 0, 0], [0, 0, 0]]], dtype=np.float32)   # mm 단위 1m 점
    out = place(tri, base_link_visual(URDF), joint_offset(URDF))
    assert np.allclose(out[0, 0], [0.1, 1.0, 0.01], atol=1e-5)    # 90° 회전 → +y, origin x +0.1, base z +0.01


def test_read_binary_stl(tmp_path):
    p = tmp_path / 'a.stl'
    with open(p, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', 1) + struct.pack('<12fH', 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0))
    t = read_stl(str(p))
    assert t.shape == (1, 3, 3) and np.allclose(t[0, 1], [1, 0, 0])


def test_top_view_orientation():
    # +x 쪽(로봇 앞)에 높은 삼각형, -x 쪽에 낮은 삼각형 → 이미지 오른쪽이 더 밝아야 한다
    lo = [[-0.1, -0.05, 0.0], [-0.02, -0.05, 0.0], [-0.06, 0.05, 0.0]]
    hi = [[0.02, -0.05, 0.1], [0.1, -0.05, 0.1], [0.06, 0.05, 0.1]]
    img, meta = render_top_view(np.array([lo, hi], dtype=np.float32), resolution=0.002)
    assert img.shape[1] == meta['width'] and img.shape[0] == meta['height']
    row = img[img.shape[0] // 2]
    left, right = row[: meta['width'] // 2], row[meta['width'] // 2:]
    assert right[right[:, 3] > 0, 0].mean() > left[left[:, 3] > 0, 0].mean()
    # 원점: 왼쪽 열 = x 최소, 아래 모서리 + 높이 = y 최대
    assert abs(meta['origin']['x'] - (-0.1)) < 1e-6
    assert abs(meta['origin']['y'] + meta['height'] * 0.002 - 0.05) < 1e-6
