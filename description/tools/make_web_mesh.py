#!/usr/bin/env python3
"""URDF 의 base_link 메시(…_lowpoly.stl, 10만 삼각형)에서 웹 콘솔용 경량 메시(…_lowpoly_web.stl)를 만든다.

웹 콘솔(my_web_api)은 URDF 의 메시 경로 옆에 `<이름>_web.stl` 이 있으면 그걸 쓴다 — 브라우저 3D 화면과
위에서 본 이미지(서버 렌더)에 10만 삼각형은 과하고, 폰에서도 가볍게 열리게 3만 삼각형으로 줄인다.
CAD 를 다시 export 해 lowpoly 를 바꿨다면 이 스크립트도 다시 돌린다 (docs/description.md 3.4).

  pip install open3d          # 호스트에서 (컨테이너엔 없음)
  python3 description/tools/make_web_mesh.py [입력.stl] [삼각형 수]
"""
import os
import sys

import numpy as np
import open3d as o3d

here = os.path.dirname(os.path.abspath(__file__))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '..', 'meshes', 'my_turtlebot3_burger_lowpoly.stl')
target = int(sys.argv[2]) if len(sys.argv) > 2 else 30000
dst = os.path.splitext(src)[0] + '_web.stl'

m = o3d.io.read_triangle_mesh(src)
m.remove_duplicated_vertices()
m.remove_degenerate_triangles()
d = m.simplify_quadric_decimation(target_number_of_triangles=target)   # 형상 오차가 작은 순서로 꼭짓점 병합
d.remove_unreferenced_vertices()
d.compute_triangle_normals()
o3d.io.write_triangle_mesh(dst, d, write_ascii=False)

b0 = np.asarray(m.get_axis_aligned_bounding_box().get_extent()) * 1000
b1 = np.asarray(d.get_axis_aligned_bounding_box().get_extent()) * 1000
print(f'{len(m.triangles)} → {len(d.triangles)} 삼각형, {os.path.getsize(dst) / 1e6:.2f} MB  →  {dst}')
print(f'외곽(mm) {b0.round(1)} → {b1.round(1)}')
