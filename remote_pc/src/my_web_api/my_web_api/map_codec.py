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
    # inflation(장애물 둘레 여유) 은 옅게 — 진하면 아래 지도의 벽을 덮어 라이다·장애물이 지도와 맞는지 안 보인다 (2026-11-23)
    out[..., 3] = np.where(mid, int(alpha * 0.35), 0)
    out[grid == 99] = _rgba('#00e5ff', alpha)
    out[grid == 100] = _rgba('#ff00d4', min(255, alpha + 60))
    return np.flipud(out)


def encode_png(rgba: np.ndarray) -> bytes:
    """RGBA → PNG 바이트. OpenCV 는 BGRA 순서라 채널을 바꿔서 넘긴다."""
    ok, buf = cv2.imencode('.png', cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
    if not ok:
        raise RuntimeError('PNG 인코딩 실패')
    return buf.tobytes()


def align_to_axes(grid: np.ndarray, resolution: float, origin, snap=None):
    """회전된 격자(원점 yaw ≠ 0)를 표시 좌표계 축에 맞춘 격자로 다시 뽑는다 (가장 가까운 칸 값 — 비용 값이 섞이지 않게).

    로컬 costmap 은 odom 기준이라 map 과 각도가 다르다 (map→odom 회전). 그대로 깔면 칸 격자가 통째로 비스듬히
    보여 지도 벽과 비교하기 어렵다 → 같은 해상도의 축 정렬 격자로 바꾸고, snap(지도 원점 x, y)이 있으면
    칸 경계를 지도 칸 경계에 맞춘다. grid 는 0번 행 = 아래(OccupancyGrid 순서), 바깥은 -1(미지).
    반환: (새 격자, (x0, y0, 0.0))
    """
    ox, oy, yaw = origin
    h, w = grid.shape
    c, s = np.cos(yaw), np.sin(yaw)
    if abs(s) < 1e-6 and c > 0:
        return grid, (ox, oy, 0.0)
    r = resolution
    cx = np.array([0, w, 0, w]) * r
    cy = np.array([0, 0, h, h]) * r
    wx, wy = ox + c * cx - s * cy, oy + s * cx + c * cy
    x0, y0 = wx.min(), wy.min()
    if snap is not None:
        x0 = snap[0] + np.floor((x0 - snap[0]) / r) * r
        y0 = snap[1] + np.floor((y0 - snap[1]) / r) * r
    W = int(np.ceil((wx.max() - x0) / r))
    H = int(np.ceil((wy.max() - y0) / r))
    # 새 칸 (I, J) 중심의 월드 좌표 → 원래 격자의 열·행 (역변환)
    m = np.array([[c, s, (c * (x0 - ox) + s * (y0 - oy)) / r + 0.5 * (c + s) - 0.5],
                  [-s, c, (-s * (x0 - ox) + c * (y0 - oy)) / r + 0.5 * (c - s) - 0.5]], dtype=np.float64)
    out = cv2.warpAffine(grid.astype(np.int16), m, (W, H), flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=-1)
    return out.astype(np.int8), (float(x0), float(y0), 0.0)
