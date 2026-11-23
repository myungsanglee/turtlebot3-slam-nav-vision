"""라이다 측정값 이상 감지 — "/scan 은 5Hz 로 오는데 내용이 가짜" (ROS 없는 순수 로직).

2026-11-24: LDS 가 모든 각도에 거리 1132mm·세기 959 를 매 바퀴 똑같이 보냈다 (내부 측정 실패, 원본 패킷까지 동일).
수신 주기는 정상이라 기존 감시(수신 Hz)로는 안 보였고, 화면엔 로봇 둘레 원, 위치 찾기는 점수 0.10 으로 실패.
판정: 유효 빔이 충분한데 거리 값 종류가 2개 이하이거나(모든 방향 같은 거리), 직전 스캔과 완전히 같은 스캔이
연속 FROZEN_SCANS 번 — 실제 라이다는 잡음 때문에 같은 바퀴가 두 번 나오지 않는다.
"""
from __future__ import annotations

import threading

import numpy as np

MIN_VALID = 50        # 유효 빔이 이보다 적으면(빈 공간·가림) 판정하지 않음
FROZEN_SCANS = 5      # 5Hz 에서 1초
STALE_SEC = 3.0


class ScanHealth:
    def __init__(self):
        self._lock = threading.Lock()
        self._prev = None
        self._bad = 0
        self._at = None
        self._state = 'unknown'
        self._detail = None

    def update(self, t: float, ranges, intensities=()) -> None:
        r = np.asarray(ranges, np.float32)
        ok = np.isfinite(r) & (r > 0)
        key = (r.tobytes(), np.asarray(intensities, np.float32).tobytes())
        with self._lock:
            same_prev = self._prev is not None and key == self._prev
            self._prev, self._at = key, t
            if ok.sum() < MIN_VALID:
                self._bad = 0
                self._state, self._detail = 'ok', None
                return
            kinds = len(np.unique(np.round(r[ok], 3)))
            if kinds <= 2 or same_prev:
                self._bad += 1
            else:
                self._bad = 0
            if self._bad >= FROZEN_SCANS:
                self._state = 'frozen'
                self._detail = (f'모든 방향 거리 {float(np.median(r[ok])):.3f} m' if kinds <= 2
                                else '매 바퀴 똑같은 값')
            elif self._bad == 0:
                self._state, self._detail = 'ok', None

    def snapshot(self, now: float) -> dict:
        with self._lock:
            if self._at is None or now - self._at > STALE_SEC:
                return {'state': 'unknown', 'detail': None}
            return {'state': self._state, 'detail': self._detail}
