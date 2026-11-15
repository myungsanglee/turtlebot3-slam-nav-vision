"""저장 지도 관리 — 목록·미리보기·저장·삭제 (지도 파일 = nav2 map_saver 형식 yaml + pgm).

기본 지도: 지도 폴더의 `.default_map` (이름 한 줄) — 대기 중에도 그 지도를 화면에 보여 준다 (ros_node).

저장은 nav2_map_server 의 `map_saver_cli` 를 그대로 쓴다 (README·my_slam.md 에 적힌 수동 절차와 같은 도구 —
웹이 하든 사람이 하든 같은 결과 파일). 지도 폴더는 /overlay_ws/maps (호스트 remote_pc/maps, git 제외).
"""
from __future__ import annotations

import os
import re
import subprocess
import time

import cv2
import numpy as np

from . import zones as zone_lib
from .map_codec import MAP_FREE, MAP_OCCUPIED, MAP_UNKNOWN, encode_png

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')   # 파일 이름으로 안전한 문자만 (경로 조작 방지)


def valid_name(name: str) -> bool:
    return bool(NAME_RE.match(name))


def read_yaml_simple(path: str) -> dict:
    """map_saver 가 쓰는 평평한 yaml (image, resolution, origin …) 만 읽는다 — PyYAML 없이도."""
    out = {}
    for line in open(path, encoding='utf-8'):
        if ':' not in line or line.lstrip().startswith('#'):
            continue
        k, v = line.split(':', 1)
        out[k.strip()] = v.strip()
    return out


class MapStore:
    def __init__(self, directory: str):
        self.dir = directory
        os.makedirs(directory, exist_ok=True)

    def path(self, name: str, ext: str) -> str:
        if not valid_name(name):
            raise ValueError('지도 이름은 영문·숫자·_·- 1~40자')
        return os.path.join(self.dir, f'{name}.{ext}')

    def list(self) -> list[dict]:
        default = self.get_default()
        maps = []
        for f in sorted(os.listdir(self.dir)):
            if not f.endswith('.yaml'):
                continue
            name = f[:-5]
            if not valid_name(name):
                continue
            yaml_path = os.path.join(self.dir, f)
            try:
                y = read_yaml_simple(yaml_path)
                img = cv2.imread(os.path.join(self.dir, y.get('image', f'{name}.pgm')), cv2.IMREAD_UNCHANGED)
                h, w = img.shape[:2]
                res = float(y.get('resolution', 0.05))
                try:
                    n_zones = len(zone_lib.load(self.path(name, 'zones.json')))
                except (ValueError, OSError):
                    n_zones = 0
                maps.append({'name': name, 'default': name == default, 'width': w, 'height': h, 'resolution': res,
                             'size_m': [round(w * res, 2), round(h * res, 2)],
                             'saved': os.path.getmtime(yaml_path), 'yaml': yaml_path, 'zones': n_zones})
            except Exception:  # noqa: BLE001 — 깨진 파일은 목록에서 빼되 다른 지도는 보여준다
                continue
        return sorted(maps, key=lambda m: -m['saved'])

    def thumbnail(self, name: str) -> bytes:
        """pgm(254 빈 공간·0 장애물·205 미지) → 웹 지도와 같은 색의 PNG. pgm 은 이미 위쪽 = 지도 +y."""
        y = read_yaml_simple(self.path(name, 'yaml'))
        img = cv2.imread(os.path.join(self.dir, y.get('image', f'{name}.pgm')), cv2.IMREAD_GRAYSCALE)
        out = np.empty(img.shape + (4,), dtype=np.uint8)
        out[:] = MAP_UNKNOWN
        out[img >= 250] = MAP_FREE
        out[img <= 50] = MAP_OCCUPIED
        return encode_png(out)

    def save(self, name: str, timeout_sec: float = 15.0) -> dict:
        """지금 /map 을 파일로 (map_saver_cli). 같은 이름이 있으면 덮어쓴다."""
        base = self.path(name, 'yaml')[:-5]
        # --free 0.196: nav2 기본값(0.25)으로 저장하면 미탐사(회색 205) 픽셀이 다시 불러올 때 (255-205)/255=0.196 < 0.25
        #   → '빈 칸' 으로 읽힌다 (벽 바깥까지 빈 곳이 됨 — troubleshooting 2026-11-14). ROS 1 map_saver 의 값.
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', base, '--free', '0.196',
               '--ros-args', '-p', f'save_map_timeout:={timeout_sec}']
        t0 = time.time()
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_sec + 10)
        ok = r.returncode == 0 and os.path.isfile(base + '.yaml') and os.path.getmtime(base + '.yaml') >= t0 - 1
        tail = (r.stdout + r.stderr).strip().splitlines()[-3:]
        msg = '저장됨' if ok else '저장 실패: ' + ' / '.join(tail)
        # 같은 이름으로 덮어쓴 새 지도는 좌표 원점이 다를 수 있다 (SLAM 을 새로 켜면 원점이 바뀜)
        #   → 옛 금지 구역·마지막 위치(자동 위치 찾기)는 버린다
        zpath = self.path(name, 'zones.json')
        if ok and os.path.isfile(zpath):
            os.remove(zpath)
            msg += ' (새 지도라 이전 금지 구역은 지움)'
        ppath = self.path(name, 'pose.json')
        if ok and os.path.isfile(ppath):
            os.remove(ppath)
        return {'ok': ok, 'name': name, 'message': msg}

    # ---------- 기본 지도 ----------
    def _default_path(self) -> str:
        return os.path.join(self.dir, '.default_map')

    def get_default(self) -> str | None:
        """기본 지도 이름. 파일이 없거나 지도가 지워졌으면 None."""
        try:
            name = open(self._default_path(), encoding='utf-8').read().strip()
        except OSError:
            return None
        return name if valid_name(name) and os.path.isfile(self.path(name, 'yaml')) else None

    def set_default(self, name: str | None):
        if name is None:
            if os.path.isfile(self._default_path()):
                os.remove(self._default_path())
            return
        if not os.path.isfile(self.path(name, 'yaml')):
            raise FileNotFoundError(name)
        with open(self._default_path(), 'w', encoding='utf-8') as f:
            f.write(name + '\n')

    def load_grid(self, name: str):
        """저장 지도 → OccupancyGrid 와 같은 격자 (행 0 = 아래, 0 빈칸·100 장애물·-1 미지), 해상도, 원점 (x, y, yaw).
        map_server 와 같은 trinary 규칙: occ = (255 − 픽셀)/255 (negate 0), occ > occupied_thresh → 100, < free_thresh → 0."""
        y = read_yaml_simple(self.path(name, 'yaml'))
        img = cv2.imread(os.path.join(self.dir, y.get('image', f'{name}.pgm')), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(name)
        occ = (255.0 - img.astype(np.float32)) / 255.0
        if y.get('negate', '0').strip() in ('1', 'true', 'True'):
            occ = 1.0 - occ
        grid = np.full(img.shape, -1, np.int8)
        grid[occ > float(y.get('occupied_thresh', 0.65))] = 100
        grid[occ < float(y.get('free_thresh', 0.196))] = 0
        ox, oy, *rest = (float(v) for v in y['origin'].strip('[]').split(','))
        return grid[::-1].copy(), float(y.get('resolution', 0.05)), (ox, oy, rest[0] if rest else 0.0)

    def mtime(self, name: str) -> float:
        return os.path.getmtime(self.path(name, 'yaml'))

    # ---------- 금지 구역 (zones.py) ----------
    def geometry(self, name: str) -> dict:
        """저장 지도의 격자 크기·해상도·원점 (마스크를 같은 격자로 만들 때). yaml origin = [x, y, yaw]."""
        y = read_yaml_simple(self.path(name, 'yaml'))
        img = cv2.imread(os.path.join(self.dir, y.get('image', f'{name}.pgm')), cv2.IMREAD_UNCHANGED)
        h, w = img.shape[:2]
        ox, oy, *_ = (float(v) for v in y['origin'].strip('[]').split(','))
        return {'width': w, 'height': h, 'resolution': float(y.get('resolution', 0.05)), 'origin_x': ox, 'origin_y': oy}

    def get_zones(self, name: str) -> list[dict]:
        return zone_lib.load(self.path(name, 'zones.json'))

    def set_zones(self, name: str, zones) -> list[dict]:
        if not os.path.isfile(self.path(name, 'yaml')):
            raise FileNotFoundError(name)
        clean = zone_lib.validate(zones)
        zone_lib.save(self.path(name, 'zones.json'), clean)
        return clean

    def zones_mtime(self, name: str) -> float:
        p = self.path(name, 'zones.json')
        return os.path.getmtime(p) if os.path.isfile(p) else 0.0

    def delete(self, name: str) -> bool:
        y_path = self.path(name, 'yaml')
        if not os.path.isfile(y_path):
            return False
        img = read_yaml_simple(y_path).get('image', f'{name}.pgm')
        for p in (y_path, os.path.join(self.dir, os.path.basename(img)), self.path(name, 'zones.json'),
                  self.path(name, 'pose.json')):
            if os.path.isfile(p):
                os.remove(p)
        if self.get_default() is None and os.path.isfile(self._default_path()):
            os.remove(self._default_path())   # 기본 지도를 지웠으면 설정도 지운다
        return True
