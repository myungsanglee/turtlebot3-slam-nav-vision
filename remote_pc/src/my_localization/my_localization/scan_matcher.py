"""스캔-지도 매칭으로 '지도 위 어디에 있는지' 찾기 (전역 위치 추정) — ROS 의존 없는 순수 numpy/OpenCV.

원리 (docs/my_localization.md 2장):
  1. 우도장(likelihood field): 지도의 각 칸에 "가장 가까운 벽까지 거리 d" 를 미리 계산해 두고
     exp(-d²/2σ²) 로 바꾼다. 벽 위 = 1, 벽에서 멀수록 0.
  2. 후보 자세 (x, y, θ) 에 라이다 점들을 올려 놓고, 점이 떨어진 칸의 우도장 값을 평균 → 점수(0~1).
     점들이 지도의 벽과 잘 겹칠수록 1 에 가깝다.
  3. 지도 전체의 빈 칸 × 모든 방향을 거칠게(10cm·3°, 흐린 우도장) 훑어 점수 높은 후보를 몇 개 고르고,
     각 후보 주변을 점점 촘촘하게(→ 약 6mm·0.4°, 선명한 우도장) 다시 찾아 정밀 위치를 얻는다.
     (Cartographer 의 correlative scan matching 과 같은 계열 — 흐린 지도로 넓게, 선명한 지도로 정밀하게)
  4. 1등과 2등(다른 곳) 점수 차가 작으면 "어디인지 확신 못 함" — 대칭인 방 등. 억지로 고르지 않는다.

좌표 약속: 지도는 nav_msgs/OccupancyGrid 와 같다 — data[row, col], row 가 y, col 이 x,
칸 (col,row) 의 왼쪽 아래 모서리 = origin + (col, row)·resolution. 지도 원점 회전은 0 이라고 가정
(slam_toolbox·map_server 지도는 0). 라이다 점은 이미 로봇 기준(base_footprint) 2D 좌표로 바꿔서 넘긴다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Candidate:
    x: float
    y: float
    yaw: float
    score: float

    def as_dict(self) -> dict:
        return {'x': round(self.x, 3), 'y': round(self.y, 3), 'yaw': round(self.yaw, 4), 'score': round(self.score, 3)}


def angle_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class MapModel:
    """지도 한 장에 대한 우도장·후보 위치를 미리 계산해 둔다 (지도가 바뀔 때만 다시 만든다)."""

    def __init__(self, grid: np.ndarray, resolution: float, origin_xy: tuple[float, float],
                 occupied_thresh: int = 65, free_thresh: int = 25, clearance: float = 0.08):
        grid = np.asarray(grid)
        self.h, self.w = grid.shape
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        occ = grid >= occupied_thresh
        # 가장 가까운 벽(점유 칸)까지 거리 [m]. distanceTransform 은 0 인 픽셀까지의 거리를 준다.
        src = np.where(occ, 0, 255).astype(np.uint8)
        self.dist = cv2.distanceTransform(src, cv2.DIST_L2, cv2.DIST_MASK_PRECISE).astype(np.float32) * self.res
        if not occ.any():
            self.dist[:] = 1e3
        # 로봇이 있을 수 있는 칸: 확실히 빈 칸이고, 벽에서 clearance 이상 떨어진 곳 (벽 속·벽에 붙은 자세 제외)
        self.free = (grid >= 0) & (grid <= free_thresh) & (self.dist >= clearance)
        self._fields: dict = {}

    # ---------- 우도장 ----------
    def field(self, sigma: float, spread: int = 0) -> np.ndarray:
        """exp(-d²/2σ²). spread>0 이면 그 칸 수만큼 최댓값으로 번지게 한다(거친 검색에서 격자 사이 놓침 방지)."""
        key = (round(sigma, 4), spread)
        if key not in self._fields:
            f = np.exp(-(self.dist ** 2) / (2 * sigma * sigma)).astype(np.float32)
            if spread > 0:
                k = 2 * spread + 1
                f = cv2.dilate(f, np.ones((k, k), np.uint8))
            self._fields[key] = f
        return self._fields[key]

    # ---------- 점수 ----------
    def score(self, field: np.ndarray, poses: np.ndarray, pts: np.ndarray) -> np.ndarray:
        """poses (P,3)=[x,y,yaw] 각각에 점들 pts (N,2) 를 올렸을 때 평균 우도. 지도 밖 점은 0."""
        poses = np.atleast_2d(np.asarray(poses, np.float64))
        c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        wx = poses[:, 0:1] + c * pts[None, :, 0] - s * pts[None, :, 1]
        wy = poses[:, 1:2] + s * pts[None, :, 0] + c * pts[None, :, 1]
        return self._lookup(field, wx, wy).mean(axis=1)

    def _lookup(self, field, wx, wy):
        col = np.floor((wx - self.ox) / self.res).astype(np.int64)
        row = np.floor((wy - self.oy) / self.res).astype(np.int64)
        ok = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        out = np.zeros(col.shape, np.float32)
        out[ok] = field[row[ok], col[ok]]
        return out

    def cell_centers(self, mask: np.ndarray) -> np.ndarray:
        rows, cols = np.nonzero(mask)
        return np.stack([self.ox + (cols + 0.5) * self.res, self.oy + (rows + 0.5) * self.res], axis=1)

    def is_free(self, x: float, y: float) -> bool:
        col, row = int((x - self.ox) // self.res), int((y - self.oy) // self.res)
        return 0 <= col < self.w and 0 <= row < self.h and bool(self.free[row, col])


@dataclass
class SearchParams:
    coarse_step: float = 0.10          # 거친 검색 위치 간격 [m] (후보가 너무 많으면 자동으로 늘림)
    coarse_angle_deg: float = 3.0      # 거친 검색 방향 간격
    coarse_sigma: float = 0.15         # 거친 검색 우도장 σ [m] (흐리게 → 격자 사이 정답도 점수가 남음)
    fine_sigma: float = 0.05           # 정밀화 우도장 σ [m]
    max_coarse_positions: int = 20000  # 거친 검색 위치 수 상한 (속도)
    coarse_points: int = 120           # 거친 검색에 쓰는 라이다 점 수 (고르게 솎음)
    peaks: int = 12                    # 정밀화할 후보 수
    peak_sep_m: float = 0.35           # 서로 다른 후보로 볼 최소 거리
    peak_sep_deg: float = 25.0         # 또는 최소 방향 차


def subsample(pts: np.ndarray, n: int) -> np.ndarray:
    if len(pts) <= n:
        return pts
    idx = np.linspace(0, len(pts) - 1, n).round().astype(int)
    return pts[idx]


def _distinct(c: Candidate, kept: list[Candidate], sep_m: float, sep_rad: float) -> bool:
    return all(math.hypot(c.x - k.x, c.y - k.y) > sep_m or abs(angle_diff(c.yaw, k.yaw)) > sep_rad for k in kept)


def refine(m: MapModel, pts: np.ndarray, x: float, y: float, yaw: float, p: SearchParams,
           step: float = 0.05, astep: float = math.radians(3.0), min_step: float = 0.006) -> Candidate:
    """언덕 오르기: 주변 3×3×3 자세 중 가장 좋은 쪽으로 옮기고, 더 좋아지지 않으면 간격을 반으로."""
    f = m.field(p.fine_sigma)
    d = np.array([-1.0, 0.0, 1.0])
    grid = np.stack(np.meshgrid(d, d, d, indexing='ij'), -1).reshape(-1, 3)
    cur = np.array([x, y, yaw])
    best = float(m.score(f, cur[None], pts)[0])
    for _ in range(200):   # 안전 상한
        poses = cur + grid * np.array([step, step, astep])
        sc = m.score(f, poses, pts)
        i = int(np.argmax(sc))
        if sc[i] > best + 1e-6:
            cur, best = poses[i], float(sc[i])
            continue
        if step <= min_step:
            break
        step, astep = step / 2, astep / 2
    return Candidate(float(cur[0]), float(cur[1]), float(angle_diff(cur[2], 0.0)), best)


def global_search(m: MapModel, pts: np.ndarray, p: SearchParams | None = None) -> list[Candidate]:
    """지도 전체에서 pts 가 가장 잘 맞는 자세 후보들 (점수 높은 순, 서로 다른 곳만)."""
    p = p or SearchParams()
    if len(pts) < 10 or not m.free.any():
        return []
    # 거친 위치 격자: 빈 칸 중 step 간격으로 (너무 많으면 간격을 늘린다)
    step = max(p.coarse_step, m.res)
    while True:
        k = max(1, int(round(step / m.res)))
        mask = np.zeros_like(m.free)
        mask[k // 2::k, k // 2::k] = True
        pos = m.cell_centers(mask & m.free)
        if len(pos) <= p.max_coarse_positions or k > 64:
            break
        step *= 1.5
    if len(pos) == 0:
        pos = m.cell_centers(m.free)
    cf = m.field(max(p.coarse_sigma, step), spread=max(0, k // 2))
    cpts = subsample(pts, p.coarse_points)
    angles = np.arange(0.0, 2 * math.pi, math.radians(p.coarse_angle_deg))
    scores = np.empty((len(angles), len(pos)), np.float32)
    for i, a in enumerate(angles):   # 방향마다: 회전한 점들 + 모든 위치
        c, s = math.cos(a), math.sin(a)
        rp = cpts @ np.array([[c, s], [-s, c]])       # (N,2) 회전
        scores[i] = m._lookup(cf, pos[:, 0:1] + rp[None, :, 0], pos[:, 1:2] + rp[None, :, 1]).mean(axis=1)

    # 점수 높은 순으로 서로 다른 후보 고르기
    flat = scores.ravel()
    top = np.argsort(flat)[::-1][: max(2000, p.peaks * 50)]
    sep_rad = math.radians(p.peak_sep_deg)
    seeds: list[Candidate] = []
    for j in top:
        ai, pi = divmod(int(j), len(pos))
        c = Candidate(float(pos[pi, 0]), float(pos[pi, 1]), float(angles[ai]), float(flat[j]))
        if _distinct(c, seeds, p.peak_sep_m, sep_rad):
            seeds.append(c)
            if len(seeds) >= p.peaks:
                break

    # 정밀화 후 같은 곳으로 모인 후보는 합친다
    out: list[Candidate] = []
    for c in sorted((refine(m, pts, s.x, s.y, s.yaw, p) for s in seeds), key=lambda c: -c.score):
        if _distinct(c, out, p.peak_sep_m, sep_rad):
            out.append(c)
    return out


def local_search(m: MapModel, pts: np.ndarray, x: float, y: float, yaw: float,
                 radius: float = 0.3, angle_deg: float = 25.0, p: SearchParams | None = None) -> Candidate:
    """알고 있는 자세 근처(±radius, ±angle)만 찾아 정밀화 — 마지막 위치 확인용."""
    p = p or SearchParams()
    f = m.field(p.coarse_sigma)
    cpts = subsample(pts, p.coarse_points)
    r = np.arange(-radius, radius + 1e-9, 0.05)
    a = np.radians(np.arange(-angle_deg, angle_deg + 1e-9, 2.5))
    g = np.stack(np.meshgrid(r, r, a, indexing='ij'), -1).reshape(-1, 3) + np.array([x, y, yaw])
    i = int(np.argmax(m.score(f, g, cpts)))
    return refine(m, pts, g[i, 0], g[i, 1], g[i, 2], p)


@dataclass
class Decision:
    state: str            # 'localized' | 'ambiguous' | 'failed'
    best: Candidate | None
    message: str


def decide(cands: list[Candidate], accept_score: float, margin: float) -> Decision:
    """1등 점수가 충분하고, 다른 곳(2등)보다 margin 이상 높아야 '확신'."""
    if not cands or cands[0].score < accept_score:
        s = f'{cands[0].score:.2f}' if cands else '-'
        return Decision('failed', cands[0] if cands else None,
                        f'지도와 맞는 곳을 찾지 못함 (최고 점수 {s} < {accept_score:.2f})')
    if len(cands) > 1 and cands[0].score - cands[1].score < margin:
        return Decision('ambiguous', cands[0],
                        f'비슷하게 맞는 곳이 여러 군데 ({cands[0].score:.2f} vs {cands[1].score:.2f})')
    return Decision('localized', cands[0], f'위치 찾음 (점수 {cands[0].score:.2f})')


# ---------- 빔 일관성: "벽을 뚫고 본" 후보 걸러내기 ----------
@dataclass
class Scan:
    """라이다 한 바퀴. angles 는 센서 기준, sensor=(x,y,yaw) 는 로봇(base_footprint) 기준 센서 장착 자세."""
    ranges: np.ndarray
    angles: np.ndarray
    sensor: tuple[float, float, float] = (0.0, 0.0, 0.0)
    range_min: float = 0.12
    range_max: float = 3.5

    def valid(self) -> np.ndarray:
        r = np.asarray(self.ranges, np.float64)
        return np.isfinite(r) & (r >= self.range_min) & (r <= self.range_max)

    def points(self) -> np.ndarray:
        """유효한 빔 끝점의 로봇 기준 2D 좌표 (N,2)."""
        v = self.valid()
        r, a = np.asarray(self.ranges, np.float64)[v], np.asarray(self.angles, np.float64)[v] + self.sensor[2]
        return np.stack([self.sensor[0] + r * np.cos(a), self.sensor[1] + r * np.sin(a)], axis=1)


def consistency(m: MapModel, grid: np.ndarray, scan: Scan, c: Candidate, tol: float = 0.2,
                beams: int = 180) -> float:
    """후보 자세에서 지도로 '보여야 할' 거리와 실제 거리를 빔마다 비교 (0~1, 높을수록 그 자리가 맞다).

    우도장 점수는 '점이 벽 근처에 떨어졌나'만 본다 → 벽 몇 개만 보이는 구석은 다른 구석과도 잘 맞는다.
    여기서는 반대 증거를 본다: 실제 거리가 지도상 벽보다 tol 이상 멀다 = 그 자리라면 레이저가 벽을 뚫은 것
    → 그 후보는 틀렸다는 강한 증거. 지도보다 가까운 건 지도에 없는 물체(사람·가구)일 수 있어 벌점 절반.
    """
    n = len(scan.ranges)
    idx = np.linspace(0, n - 1, min(beams, n)).round().astype(int)
    r = np.asarray(scan.ranges, np.float64)[idx]
    a = np.asarray(scan.angles, np.float64)[idx]
    good = np.isfinite(r) & (r >= scan.range_min)
    if good.sum() < 10:
        return 0.0
    r, a = r[good], a[good]
    cy, sy = math.cos(c.yaw), math.sin(c.yaw)
    ox = c.x + cy * scan.sensor[0] - sy * scan.sensor[1]
    oy = c.y + sy * scan.sensor[0] + cy * scan.sensor[1]
    exp = raycast(m, grid, ox, oy, c.yaw + scan.sensor[2], a, scan.range_max)
    r = np.minimum(r, scan.range_max)
    through = r > exp + tol          # 지도라면 벽에 막혀야 할 빔이 더 멀리 감
    short = r < exp - tol            # 지도에 없는 물체 (또는 틀린 자리)
    bad = through.astype(float) + 0.5 * short
    return float(1.0 - bad.mean())


def rank(m: MapModel, grid: np.ndarray, scan: Scan, cands: list[Candidate]) -> list[Candidate]:
    """최종 점수 = 우도장 점수 × 빔 일관성. 다시 정렬."""
    out = [Candidate(c.x, c.y, c.yaw, c.score * consistency(m, grid, scan, c)) for c in cands]
    return sorted(out, key=lambda c: -c.score)


# ---------- 시험용: 지도에서 가상 라이다 스캔 만들기 ----------
def raycast(m: MapModel, grid: np.ndarray, x: float, y: float, yaw: float, angles: np.ndarray,
            max_range: float = 3.5, occupied_thresh: int = 65) -> np.ndarray:
    """(x,y,yaw) 에서 각 방향으로 벽(점유 칸)에 닿는 거리. 못 닿으면 inf. (빔 일관성 검사에도 사용)"""
    occ = np.asarray(grid) >= occupied_thresh
    t = np.arange(0.0, max_range, m.res / 2)
    a = yaw + np.asarray(angles, np.float64)
    wx = x + np.cos(a)[:, None] * t[None]
    wy = y + np.sin(a)[:, None] * t[None]
    col = np.floor((wx - m.ox) / m.res).astype(int)
    row = np.floor((wy - m.oy) / m.res).astype(int)
    ok = (col >= 0) & (col < m.w) & (row >= 0) & (row < m.h)
    hit = np.zeros(col.shape, bool)
    hit[ok] = occ[row[ok], col[ok]]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    return np.where(first >= 0, t[np.maximum(first, 0)], np.inf)
