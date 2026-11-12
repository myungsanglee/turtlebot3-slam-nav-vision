"""금지 구역·가상 벽 — 거울·유리처럼 센서가 못 보는 곳을 사람이 지도에 그려 막는다 (ROS 의존 없음 → 단위 테스트).

왜 필요한가: 라이다 빛·카메라 적외선은 거울에서 튕겨 나가거나(빈 공간으로 보임) 거울 속 상을 '거울 뒤'에 찍고,
유리는 통과한다. 센서만으로는 확실히 못 막으므로 로봇청소기 앱의 '가상 벽·금지 구역'처럼 사람이 표시한다.

Nav2 표준 Keepout Filter 를 쓴다: 지도와 같은 격자의 마스크(OccupancyGrid)에서 값 100 인 칸을 장애물(LETHAL)로 덮는다.
이 모듈은 '구역 목록(지도 좌표 다각형) → 마스크 격자' 변환과 저장 형식 검증을 맡는다.

  area : 점 3개 이상, 닫힌 다각형 안쪽 전부 금지
  wall : 점 2개 이상, 이은 선을 WALL_WIDTH 두께로 금지 (거울·유리문 앞에 선 하나 긋기)

구역은 저장 지도마다 `<지도이름>.zones.json` 으로 둔다 — 지도 좌표는 그 지도에서만 의미가 있다.
"""
from __future__ import annotations

import json
import math
import os

import cv2
import numpy as np

KINDS = ('area', 'wall')
MAX_ZONES = 50
MAX_POINTS = 100
WALL_WIDTH = 0.06      # m — 가상 벽 두께 (지도 칸 5cm 보다 조금 굵게: 대각선에서도 끊기지 않게)
MASK_KEEPOUT = 100     # OccupancyGrid 100 → Nav2 Costmap2D 변환 시 LETHAL(254)
MASK_FREE = 0


def validate(zones) -> list[dict]:
    """웹에서 받은 구역 목록을 검사해 정리된 목록으로. 잘못되면 ValueError (메시지는 화면에 그대로 보여 줄 한국어)."""
    if not isinstance(zones, list):
        raise ValueError('zones 는 목록이어야 함')
    if len(zones) > MAX_ZONES:
        raise ValueError(f'구역은 최대 {MAX_ZONES}개')
    out = []
    for i, z in enumerate(zones):
        if not isinstance(z, dict) or z.get('kind') not in KINDS:
            raise ValueError(f'{i + 1}번 구역: kind 는 area 또는 wall')
        pts = z.get('points')
        need = 3 if z['kind'] == 'area' else 2
        if not isinstance(pts, list) or not need <= len(pts) <= MAX_POINTS:
            raise ValueError(f'{i + 1}번 구역: 점 {need}~{MAX_POINTS}개 필요')
        clean = []
        for p in pts:
            if not (isinstance(p, (list, tuple)) and len(p) == 2
                    and all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) < 1e4 for v in p)):
                raise ValueError(f'{i + 1}번 구역: 점은 [x, y] 숫자 (m)')
            clean.append([round(float(p[0]), 3), round(float(p[1]), 3)])
        name = str(z.get('name') or '')[:30]
        out.append({'kind': z['kind'], 'points': clean, 'name': name})
    return out


def load(path: str) -> list[dict]:
    if not os.path.isfile(path):
        return []
    with open(path, encoding='utf-8') as f:
        return validate(json.load(f).get('zones', []))


def save(path: str, zones: list[dict]):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump({'zones': zones}, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)   # 쓰는 도중 꺼져도 반쯤 쓴 파일이 남지 않게


def rasterize(zones: list[dict], width: int, height: int, resolution: float,
              origin_x: float, origin_y: float) -> np.ndarray:
    """구역 → 마스크 격자 (int8, height x width). OccupancyGrid 와 같은 배치: [행 0] = 원점 쪽(아래) 줄.

    지도 좌표 (x, y) → 칸 (col, row) = ((x − ox)/res, (y − oy)/res). cv2 는 (x=열, y=행) 순서로 받는다.
    1/8 칸 정밀도(shift=3)로 그려 경계가 반 칸씩 밀리지 않게 한다."""
    img = np.zeros((height, width), dtype=np.uint8)
    shift = 3
    scale = (1 << shift) / resolution
    thick = max(1, int(round(WALL_WIDTH / resolution)))
    for z in zones:
        pts = np.array([[(x - origin_x) * scale, (y - origin_y) * scale] for x, y in z['points']],
                       dtype=np.float64).round().astype(np.int32)
        if z['kind'] == 'area':
            cv2.fillPoly(img, [pts], 1, lineType=cv2.LINE_8, shift=shift)
            cv2.polylines(img, [pts], True, 1, thickness=1, lineType=cv2.LINE_8, shift=shift)  # 테두리 칸도 포함
        else:
            cv2.polylines(img, [pts], False, 1, thickness=thick, lineType=cv2.LINE_8, shift=shift)
    return np.where(img > 0, MASK_KEEPOUT, MASK_FREE).astype(np.int8)
