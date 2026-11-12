"""로봇 모양(CAD 메시)을 웹에 내보낸다 — URDF 가 단일 소스.

1) Pi 의 robot_state_publisher 가 보내는 /robot_description(URDF 문자열)에서 base_link 의 visual 메시
   (package://… 경로, scale, origin)를 읽는다 → 메시 파일을 바꾸면 웹도 자동으로 따라온다.
2) 위에서 내려다본 모습은 서버가 PNG 로 한 번 그려 둔다 (2D 지도용).
3) 브라우저 3D 화면엔 같은 STL 을 gzip 으로 압축해 보낸다 (10만 삼각형 5MB → 1.9MB). 3만 삼각형으로 더 줄인 경량본도
   시험했지만 구멍 뚫린 플레이트가 구겨져 보여 원본을 쓴다. URDF 가 바뀔 때만 버전이 바뀌어 브라우저가 다시 받는다.

좌표는 전부 base_footprint 기준(웹이 받는 로봇 위치가 base_footprint 이므로). base_link 는 base_footprint 위 10mm.
"""
from __future__ import annotations

import math
import os
import re
import threading
import xml.etree.ElementTree as ET

import gzip
import hashlib

import cv2
import numpy as np

from .map_codec import encode_png


# ---------------- URDF ----------------
def base_link_visual(urdf: str, link: str = 'base_link') -> dict | None:
    """URDF 에서 link 의 첫 번째 visual 메시 정보. 메시가 아니면 None."""
    root = ET.fromstring(urdf)
    for lk in root.iter('link'):
        if lk.get('name') != link:
            continue
        vis = lk.find('visual')
        mesh = vis.find('geometry/mesh') if vis is not None else None
        if mesh is None:
            return None
        origin = vis.find('origin')
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        scale = [float(v) for v in mesh.get('scale', '1 1 1').split()]
        return {'filename': mesh.get('filename'), 'xyz': xyz, 'rpy': rpy, 'scale': scale}
    return None


def joint_offset(urdf: str, parent: str = 'base_footprint', child: str = 'base_link') -> tuple[float, float, float]:
    """parent→child 고정 조인트의 xyz (base_link 가 base_footprint 위 몇 mm 인지). 없으면 0."""
    root = ET.fromstring(urdf)
    for j in root.iter('joint'):
        p, c = j.find('parent'), j.find('child')
        if p is not None and c is not None and p.get('link') == parent and c.get('link') == child:
            o = j.find('origin')
            if o is not None:
                x, y, z = (float(v) for v in o.get('xyz', '0 0 0').split())
                return x, y, z
    return 0.0, 0.0, 0.0


def resolve_package_uri(uri: str) -> str:
    """package://pkg/path → 설치된 share 경로. file:// 와 일반 경로도 받는다."""
    m = re.match(r'package://([^/]+)/(.+)', uri)
    if m:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(m.group(1)), m.group(2))
    return uri[len('file://'):] if uri.startswith('file://') else uri


# ---------------- STL ----------------
def read_stl(path: str) -> np.ndarray:
    """STL → (삼각형 수, 3 꼭짓점, 3 좌표) float32. 바이너리·ASCII 모두."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        n = int.from_bytes(data[80:84], 'little')
        if 84 + n * 50 == len(data):     # 바이너리: 헤더 80 + 개수 4 + 삼각형마다 50바이트(법선 12, 꼭짓점 36, 속성 2)
            rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]),
                                count=n, offset=84)
            return rec['v'].reshape(n, 3, 3).copy()
    vals = re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data)
    return np.array(vals, dtype=np.float32).reshape(-1, 3, 3)


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF rpy (고정축 roll→pitch→yaw) 회전 행렬 = Rz(y)·Ry(p)·Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]], dtype=np.float32)


def place(tris: np.ndarray, visual: dict, base_offset=(0.0, 0.0, 0.0)) -> np.ndarray:
    """메시 → base_footprint 좌표: scale → visual origin(rpy, xyz) → base_link 의 base_footprint 위 오프셋."""
    v = tris * np.asarray(visual['scale'], dtype=np.float32)
    v = v @ rpy_matrix(*visual['rpy']).T + np.asarray(visual['xyz'], dtype=np.float32)
    return v + np.asarray(base_offset, dtype=np.float32)


# ---------------- 위에서 본 모습 ----------------
def render_top_view(tris: np.ndarray, resolution: float = 0.0005) -> tuple[np.ndarray, dict]:
    """위에서 내려다본 모습(RGBA)과 메타(해상도·원점). 화가 알고리즘: 낮은 삼각형부터 칠하고 높은 것이 덮는다.

    색 = 높이(바닥 어둡게 → 위 밝게) × 면 방향 음영(위를 향한 면이 밝고, 비스듬한 면은 어둡게).
    이미지 위쪽 = +y(로봇 왼쪽), 오른쪽 = +x(로봇 앞) — 지도 이미지와 같은 규칙이라 웹이 같은 방식으로 깐다.
    """
    xy_min = tris[..., :2].reshape(-1, 2).min(0)
    xy_max = tris[..., :2].reshape(-1, 2).max(0)
    z = tris[..., 2]
    z_min, z_max = float(z.min()), float(z.max())
    w = int(math.ceil((xy_max[0] - xy_min[0]) / resolution)) + 2
    h = int(math.ceil((xy_max[1] - xy_min[1]) / resolution)) + 2
    img = np.zeros((h, w, 4), dtype=np.uint8)

    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nz = np.abs(n[:, 2]) / (np.linalg.norm(n, axis=1) + 1e-12)
    height = (z.mean(1) - z_min) / max(1e-6, z_max - z_min)
    dark, light = np.array([52, 60, 72], np.float32), np.array([205, 214, 226], np.float32)
    color = (dark + (light - dark) * height[:, None]) * (0.8 + 0.2 * nz[:, None])   # 높이가 주, 면 방향은 약하게

    shift = 3                                  # 1/8 픽셀 정밀도로 칠하기 (가장자리 계단 줄임)
    px = np.empty(tris.shape[:2] + (2,), dtype=np.int32)
    px[..., 0] = np.round((tris[..., 0] - xy_min[0]) / resolution * (1 << shift))
    px[..., 1] = np.round((xy_max[1] - tris[..., 1]) / resolution * (1 << shift))
    for i in np.argsort(z.max(1)):
        c = color[i]
        cv2.fillConvexPoly(img, px[i], (int(c[0]), int(c[1]), int(c[2]), 255), lineType=cv2.LINE_AA, shift=shift)

    # origin 은 지도 격자와 같은 의미: 이미지 맨 왼쪽 열·맨 아래 행의 모서리 (base_footprint 좌표).
    # 맨 위 행 = xy_max.y 이므로 맨 아래 모서리 = xy_max.y − 높이
    meta = {'width': w, 'height': h, 'resolution': resolution,
            'origin': {'x': float(xy_min[0]), 'y': float(xy_max[1]) - h * resolution, 'yaw': 0.0},
            'z_min': round(z_min, 4), 'z_max': round(z_max, 4)}
    return img, meta


class RobotModel:
    """URDF 를 받으면 메시를 읽어 위에서 본 이미지를 만들어 둔다. 실패하면 이유를 남긴다 (웹은 화살표로 대체)."""

    def __init__(self, logger):
        self._log = logger
        self.info: dict = {'available': False, 'reason': '/robot_description 대기 중'}
        self.top_png: bytes | None = None
        self.mesh_gz: bytes | None = None      # 3D 화면용 STL (gzip)
        self._urdf_hash = None

    def update(self, urdf: str):
        """URDF 가 바뀌었을 때만 다시 만든다. 렌더링(수백 ms)은 ROS 콜백을 막지 않게 별도 스레드에서."""
        h = hash(urdf)
        if h == self._urdf_hash:
            return
        self._urdf_hash = h
        threading.Thread(target=self._build, args=(urdf,), daemon=True).start()

    def _build(self, urdf: str):
        try:
            base_offset = joint_offset(urdf)              # base_footprint → base_link (이 로봇은 z +10mm)
            vis = base_link_visual(urdf)
            if vis is None:
                raise ValueError('URDF 의 base_link 에 메시 visual 이 없음')
            path = resolve_package_uri(vis['filename'])
            raw = open(path, 'rb').read()
            tris = read_stl(path)
            img, top = render_top_view(place(tris, vis, base_offset))
            self.top_png = encode_png(img)
            self.mesh_gz = gzip.compress(raw, compresslevel=6)
            self.info = {
                'available': True, 'source': vis['filename'], 'file': os.path.basename(path),
                'triangles': int(len(tris)), 'mesh_bytes': len(raw), 'mesh_gzip_bytes': len(self.mesh_gz),
                # URDF·메시가 같으면 같은 버전 → 브라우저가 캐시를 쓴다
                'version': hashlib.sha1(urdf.encode() + raw[:4096] + str(len(raw)).encode()).hexdigest()[:12],
                'top_view': top,
                # 3D 화면은 원본 STL 좌표(메시 파일 그대로)에 이 변환을 적용해 base_footprint 에 놓는다
                'transform': {'scale': vis['scale'], 'xyz': [vis['xyz'][i] + base_offset[i] for i in range(3)],
                              'rpy': vis['rpy']},
            }
            self._log.info(f'로봇 모델: {os.path.basename(path)} {len(tris)} 삼각형, 위에서 본 이미지 '
                           f'{top["width"]}x{top["height"]}px, 3D 전송 {len(raw) / 1e6:.1f}MB → gzip {len(self.mesh_gz) / 1e6:.1f}MB')
        except Exception as e:  # noqa: BLE001 — 모델이 없어도 콘솔은 동작해야 한다
            self.info = {'available': False, 'reason': f'{type(e).__name__}: {e}'}
            self._log.warn(f'로봇 모델 로드 실패 → 화살표로 표시: {e}')
