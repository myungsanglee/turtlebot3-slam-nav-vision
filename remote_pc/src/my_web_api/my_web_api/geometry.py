"""2D 좌표 변환 (ROS 메시지 의존 없음 → 단위 테스트 가능).

웹 화면은 '지도(map) 좌표, 미터' 하나로만 그린다. 라이다·경로·장애물 지도는 각자 다른 좌표계
(base_scan, odom ...)로 오므로, 서버가 TF 로 구한 변환(x, y, yaw)을 적용해 map 좌표로 옮겨서 보낸다.

변환 T = (tx, ty, yaw) 는 "자식 좌표계의 점 p 를 부모 좌표계로" 옮기는 것:
    p_parent = R(yaw) · p_child + t      (R 은 2D 회전 행렬)
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 → z 축 회전각(yaw, rad). 지면 로봇이라 roll·pitch 는 버린다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def compose(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    """a ∘ b — 'b 좌표계 → a 의 자식 좌표계 → a 의 부모 좌표계'. 둘 다 (x, y, yaw)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, ath + bth


def transform_points(t: Sequence[float], pts: Iterable[Sequence[float]], ndigits: int = 3) -> list[list[float]]:
    """점 목록을 변환 t 로 옮긴다. 웹 전송량을 줄이려고 mm 단위로 반올림."""
    tx, ty, th = t
    c, s = math.cos(th), math.sin(th)
    return [[round(tx + c * x - s * y, ndigits), round(ty + s * x + c * y, ndigits)] for x, y in pts]


def downsample(seq: list, max_count: int) -> list:
    """긴 목록(경로 수백~수천 점)을 균일 간격으로 솎아 max_count 이하로. 마지막 점(목표)은 항상 포함."""
    if len(seq) <= max_count or max_count < 2:
        return list(seq)
    step = (len(seq) - 1) / (max_count - 1)
    return [seq[round(i * step)] for i in range(max_count)]


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, max_points: int = 180) -> list[tuple[float, float]]:
    """LaserScan(극좌표: 각도·거리) → 라이다 좌표계(base_scan)의 (x, y) 점. 무효값(inf·nan·범위 밖)은 버린다.
    360개 빔을 다 보내지 않고 max_points 개 정도로 각도 간격을 솎는다."""
    n = len(ranges)
    step = max(1, math.ceil(n / max_points))
    out = []
    for i in range(0, n, step):
        r = ranges[i]
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_increment
        out.append((r * math.cos(a), r * math.sin(a)))
    return out
