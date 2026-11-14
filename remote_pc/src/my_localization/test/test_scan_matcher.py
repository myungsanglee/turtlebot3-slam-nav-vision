"""scan_matcher 시험 — 합성 지도에서 가상 라이다 스캔을 만들어 위치를 찾게 한다 (ROS 불필요)."""
import math

import numpy as np
import pytest

from my_localization.scan_matcher import (MapModel, Scan, angle_diff, decide, global_search, local_search,
                                          rank, raycast)

RES = 0.05
ANGLES = np.radians(np.arange(360.0))
SENSOR = (-0.10, 0.0, 0.0)   # 이 로봇의 LDS 는 base_link 뒤 10cm


def room(w_m, h_m, furniture=True):
    """벽으로 둘러싼 방 (+ 비대칭 가구). OccupancyGrid 값: 0 빈칸, 100 벽, -1 바깥."""
    w, h = int(w_m / RES), int(h_m / RES)
    g = np.full((h + 4, w + 4), -1, np.int8)
    g[1:-1, 1:-1] = 100
    g[2:-2, 2:-2] = 0
    if furniture:
        g[10:18, 30:34] = 100      # 기둥
        g[40:44, 8:30] = 100       # 긴 가구
        g[25:27, 50:70] = 100      # 선반
    return g


def scan_at(m, grid, x, y, yaw, rng, noise=0.015, dropout=0.1, clutter=0.1):
    sx, sy = x + math.cos(yaw) * SENSOR[0], y + math.sin(yaw) * SENSOR[0]
    r = raycast(m, grid, sx, sy, yaw, ANGLES) + rng.normal(0, noise, len(ANGLES))
    r = np.where(rng.random(len(r)) < dropout, np.inf, r)
    cl = rng.random(len(r)) < clutter
    r = np.where(cl, r * rng.uniform(0.3, 0.9, len(r)), r)
    return Scan(r, ANGLES, SENSOR)


def sample_pose(m, rng):
    free = m.cell_centers(m.free & (m.dist >= 0.25))
    x, y = free[rng.integers(len(free))]
    return float(x), float(y), float(rng.uniform(-math.pi, math.pi))


def test_global_search_finds_pose_in_asymmetric_room():
    grid = room(4.0, 3.0)
    m = MapModel(grid, RES, (-1.0, -0.5))
    rng = np.random.default_rng(1)
    for _ in range(15):
        x, y, yaw = sample_pose(m, rng)
        scan = scan_at(m, grid, x, y, yaw, rng)
        cands = rank(m, grid, scan, global_search(m, scan.points()))
        d = decide(cands, 0.4, 0.1)
        assert d.state == 'localized', d.message
        assert math.hypot(d.best.x - x, d.best.y - y) < 0.05
        assert abs(angle_diff(d.best.yaw, yaw)) < math.radians(3)


def test_symmetric_empty_room_is_ambiguous():
    """가구 없는 직사각형 방은 180° 돌려도 똑같다 → 억지로 고르지 않고 '애매함' 이어야 한다."""
    grid = room(4.0, 3.0, furniture=False)
    m = MapModel(grid, RES, (0.0, 0.0))
    rng = np.random.default_rng(2)
    x, y, yaw = 1.2, 1.0, 0.3
    scan = scan_at(m, grid, x, y, yaw, rng, clutter=0.0, dropout=0.0)
    cands = rank(m, grid, scan, global_search(m, scan.points()))
    d = decide(cands, 0.4, 0.1)
    assert d.state == 'ambiguous'
    # 정답과 180° 대칭 자리가 둘 다 후보에 있다
    cx, cy = 2.1, 1.6   # 방 중심 (안쪽 칸 기준)
    mirror = (2 * cx - x, 2 * cy - y)
    near = lambda c, px, py: math.hypot(c.x - px, c.y - py) < 0.1
    assert any(near(c, x, y) for c in cands[:2]) and any(near(c, *mirror) for c in cands[:2])


def test_local_search_recovers_offset_pose():
    grid = room(4.0, 3.0)
    m = MapModel(grid, RES, (0.0, 0.0))
    rng = np.random.default_rng(3)
    x, y, yaw = 1.4, 1.1, 1.0
    scan = scan_at(m, grid, x, y, yaw, rng)
    c = local_search(m, scan.points(), x + 0.2, y - 0.15, yaw + math.radians(15), radius=0.3)
    assert math.hypot(c.x - x, c.y - y) < 0.05 and abs(angle_diff(c.yaw, yaw)) < math.radians(3)


def test_wrong_place_scores_low():
    """다른 방의 스캔(이 지도에 없는 모양) → '찾지 못함'."""
    grid = room(4.0, 3.0)
    m = MapModel(grid, RES, (0.0, 0.0))
    rng = np.random.default_rng(4)
    r = rng.uniform(0.3, 3.0, 360)   # 아무 모양도 아닌 거리들
    scan = Scan(r, ANGLES, SENSOR)
    d = decide(rank(m, grid, scan, global_search(m, scan.points())), 0.4, 0.1)
    assert d.state == 'failed'


def test_scan_points_apply_sensor_offset():
    s = Scan(np.array([1.0, np.inf, 0.01]), np.array([0.0, 1.0, 2.0]), (-0.1, 0.0, math.pi / 2))
    p = s.points()
    assert p.shape == (1, 2)
    assert p[0] == pytest.approx([-0.1, 1.0])
