"""대기(IDLE) 중 기본 지도 위 로봇 위치 추정 — 화면 표시 전용 (ROS 없는 순수 로직).

왜: 대기 상태에선 AMCL·SLAM 이 꺼져 있어 map→odom 변환이 없다. 그래도 웹이 기본 지도를 보여 줄 때
로봇·라이다를 그 지도 위에 올리고 싶다 (로봇청소기 앱처럼). 저장 지도 주행의 자동 위치 찾기와 같은
스캔-지도 매칭(my_localization.scan_matcher)을 web-api 안에서 가볍게 돌려 map←odom 을 추정한다.

  처음·놓쳤을 때  지도 전체 검색 → 확신할 때만 채택 (decide: 점수·1등과 2등 차)
  추정이 있으면    odom 으로 예측한 자리 주변만 확인 (±25cm·±15°) → 맞으면 갱신, 2번 연속 안 맞으면 버리고 전체 검색
Nav2·로봇 제어에는 쓰지 않는다 (주행은 「이 지도로 주행」 → AMCL + relocalizer).
"""
from __future__ import annotations

import math
import threading

import numpy as np
from my_localization.scan_matcher import MapModel, Scan, decide, global_search, local_search, rank

from .geometry import compose


def inverse(p):
    x, y, th = p
    c, s = math.cos(th), math.sin(th)
    return -c * x - s * y, s * x - c * y, -th


class IdleLocator:
    def __init__(self, accept_score: float = 0.4, margin: float = 0.1, track_min: float = 0.3):
        self.accept, self.margin, self.track_min = accept_score, margin, track_min
        self._lock = threading.Lock()
        self._key = None
        self._model: MapModel | None = None
        self._grid = None
        self._est = None          # map ← odom (x, y, yaw)
        self._fails = 0
        self.state = 'none'       # none | searching | localized | ambiguous | failed
        self.score: float | None = None

    def set_map(self, key, grid: np.ndarray, resolution: float, origin_xy):
        """기본 지도가 바뀔 때만 우도장을 새로 만든다."""
        if key == self._key:
            return
        model = MapModel(grid, resolution, origin_xy)
        with self._lock:
            self._key, self._model, self._grid = key, model, grid
            self._est, self._fails, self.state, self.score = None, 0, 'searching', None

    def clear(self):
        with self._lock:
            self._key = self._model = self._grid = self._est = None
            self._fails, self.state, self.score = 0, 'none', None

    def lose(self):
        """로봇 데이터가 끊겼다 — 다시 켜지면 처음부터 찾는다 (그 사이 옮겼을 수 있다)."""
        with self._lock:
            if self._model is not None:
                self._est, self._fails, self.state, self.score = None, 0, 'searching', None

    def map_from_odom(self):
        with self._lock:
            return self._est

    def snapshot(self) -> dict:
        with self._lock:
            return {'state': self.state, 'score': None if self.score is None else round(self.score, 3)}

    def update(self, scan: Scan, odom_base) -> None:
        """스캔 하나와 그때의 odom←base 로 추정 갱신 (작업 스레드에서, 0.1초 안팎)."""
        with self._lock:
            m, grid, est = self._model, self._grid, self._est
        if m is None:
            return
        pts = scan.points()
        if len(pts) < 20:
            return
        if est is not None:
            px, py, pth = compose(est, odom_base)
            c = rank(m, grid, scan, [local_search(m, pts, px, py, pth, radius=0.25, angle_deg=15.0)])[0]
            if c.score >= self.track_min:
                with self._lock:
                    if self._model is m:
                        self._est = compose((c.x, c.y, c.yaw), inverse(odom_base))
                        self._fails, self.state, self.score = 0, 'localized', c.score
                return
            with self._lock:
                self._fails += 1
                if self._fails < 2:
                    return
                self._est, self.state = None, 'searching'
        cands = rank(m, grid, scan, global_search(m, pts))
        d = decide(cands, self.accept, self.margin)
        with self._lock:
            if self._model is not m:
                return
            self.score = d.best.score if d.best else None
            if d.state == 'localized':
                b = d.best
                self._est = compose((b.x, b.y, b.yaw), inverse(odom_base))
                self._fails, self.state = 0, 'localized'
            else:
                self._est, self.state = None, d.state
