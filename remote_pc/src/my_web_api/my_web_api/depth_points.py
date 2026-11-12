"""카메라 depth → 화면 표시용 3D 점 (지도 좌표). 라이다 빨간 점처럼 '카메라가 지금 보는 것'을 보여 준다.

Nav2 의 카메라 장애물 층(my_vision depth_cloud_node → /camera/points)과 **같은 계산 함수**를 쓰되, 웹 백엔드가
depth 를 직접 받아 계산한다. 그래서 Nav2 를 켜지 않은(IDLE) 상태에서도 보이고, /camera/points 발행자를 하나 더
만들지 않는다 (서버 터미널에서 Nav2 를 띄워도 중복 없음).

라이다(2D 평면)와 달리 카메라 광학 좌표계는 기울어 있으므로(z 앞·y 아래) 3D 회전 그대로 변환한다.
화면 전송량을 줄이려고 5cm 칸마다 점 하나만 남긴다 (Nav2 장애물 지도의 칸 크기와 같음).
"""
from __future__ import annotations

import numpy as np

from my_vision.depth_cloud import PixelRays, depth_to_points  # noqa: F401 (PixelRays 는 ros_node 가 씀)


def quat_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """쿼터니언 → 3x3 회전 행렬."""
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=np.float32)


def voxel_thin(pts: np.ndarray, size: float) -> np.ndarray:
    """size 격자 칸마다 첫 점 하나만 남긴다."""
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / size).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def depth_to_display(depth_mm: np.ndarray, rays: PixelRays, R: np.ndarray, t: np.ndarray,
                     min_range: float, max_range: float, voxel: float, z_max: float) -> list[list[float]]:
    """depth → 표시 좌표계 점 [[x, y, z], ...] (cm 단위로 반올림, z 는 바닥 기준 높이).
    z_max 보다 높은 점(벽 위쪽·천장)은 화면을 어지럽히기만 해서 뺀다."""
    pts = depth_to_points(depth_mm, rays, min_range, max_range)
    pts = pts @ R.T + t
    pts = pts[pts[:, 2] <= z_max]
    pts = voxel_thin(pts, voxel)
    return np.round(pts.astype(np.float64), 2).tolist()   # float32 그대로면 JSON 에 0.550000011920929 처럼 길게 나간다
