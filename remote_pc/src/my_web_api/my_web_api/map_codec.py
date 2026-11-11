"""OccupancyGrid → PNG (RGBA). 브라우저는 이 이미지를 지도 좌표에 맞춰 깔기만 한다.

OccupancyGrid 의 data 는 행 우선(row-major), **0번 행이 지도 원점(아래쪽)** 이다. 이미지는 0번 행이 위쪽이므로
세로로 뒤집는다 → 이미지 위쪽 = 지도의 +y 쪽.

값의 의미 (ROS 표준):
  지도(/map)          -1 미지, 0 빈 공간, 100 장애물 (slam_toolbox 는 이 세 값만 씀)
  Nav2 costmap         -1 미지, 0 자유, 1~98 장애물 주변 비용(inflation), 99 로봇이 닿는 거리(inscribed), 100 장애물(lethal)
                       (Nav2 내부 비용 0~254 를 0~100 으로 줄여 publish 한 것)
"""
from __future__ import annotations

import cv2
import numpy as np


def _rgba(hex_rgb: str, alpha: int = 255) -> np.ndarray:
    h = hex_rgb.lstrip('#')
    return np.array([int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), alpha], dtype=np.uint8)


MAP_UNKNOWN = _rgba('#2b3038')
MAP_FREE = _rgba('#d3d7de')
MAP_OCCUPIED = _rgba('#111317')


def map_to_rgba(data: np.ndarray) -> np.ndarray:
    """지도 격자(h, w, int8) → RGBA. 0~100 사이 값은 빈 공간~장애물 사이로 섞는다."""
    grid = data.astype(np.int16)
    out = np.empty(grid.shape + (4,), dtype=np.uint8)
    known = grid >= 0
    p = np.clip(grid, 0, 100).astype(np.float32)[..., None] / 100.0
    blended = (MAP_FREE * (1 - p) + MAP_OCCUPIED * p).astype(np.uint8)
    out[known] = blended[known]
    out[~known] = MAP_UNKNOWN
    return np.flipud(out)


def costmap_to_rgba(data: np.ndarray, alpha: int = 150) -> np.ndarray:
    """costmap 격자 → RGBA. 자유·미지는 투명, inflation 은 파랑→빨강, inscribed 는 하늘색, lethal 은 자홍색
    (RViz 의 costmap 색 규칙과 비슷하게 — 개발자가 RViz 와 웹을 오가도 헷갈리지 않게)."""
    grid = data.astype(np.int16)
    out = np.zeros(grid.shape + (4,), dtype=np.uint8)
    mid = (grid >= 1) & (grid <= 98)
    t = (grid.astype(np.float32) - 1.0) / 97.0
    out[..., 0] = np.where(mid, (t * 255).astype(np.uint8), 0)
    out[..., 2] = np.where(mid, ((1 - t) * 255).astype(np.uint8), 0)
    out[..., 3] = np.where(mid, int(alpha * 0.7), 0)
    out[grid == 99] = _rgba('#00e5ff', alpha)
    out[grid == 100] = _rgba('#ff00d4', min(255, alpha + 60))
    return np.flipud(out)


def encode_png(rgba: np.ndarray) -> bytes:
    """RGBA → PNG 바이트. OpenCV 는 BGRA 순서라 채널을 바꿔서 넘긴다."""
    ok, buf = cv2.imencode('.png', cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
    if not ok:
        raise RuntimeError('PNG 인코딩 실패')
    return buf.tobytes()
