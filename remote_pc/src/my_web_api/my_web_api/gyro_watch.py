"""자이로 영점 드리프트 감시 — 로봇이 멈춰 있는데 방향(odom yaw)이 저절로 도는지 (ROS 없는 순수 로직).

왜: TurtleBot3 는 회전각을 IMU 자이로로 계산한다 (`use_imu: true`). 자이로 영점(바이어스)이 틀어지면 정지 중에도
odom 방향이 분당 13~15° 돈다 (troubleshooting 2026-11-12, 11-14 — 켠 뒤 데워지면 다시 틀어지는 것으로 보임).
AMCL 은 로봇이 움직여야 갱신하므로 그동안 지도 위 로봇·스캔이 함께 돌아가 보인다.

판정: 최근 window_sec 동안
  - 바퀴 각도(/joint_states) 변화 < wheel_eps rad, odom 위치 변화 < pos_eps m  → '정지'
  - 정지 구간 전체에서 yaw 변화율 = 드리프트 [°/분]   (정상 보정 직후 0.03°/분, 틀어지면 13~15°/분)
바퀴 정보가 없으면 위치 변화만으로 정지를 판단한다 (제자리에서 손으로 돌리면 구분 못 함 → 웹은 경고만).
"""
from __future__ import annotations

import math
import threading
from collections import deque


def _unwrap(prev: float, cur: float) -> float:
    d = (cur - prev + math.pi) % (2 * math.pi) - math.pi
    return prev + d


class GyroDriftWatch:
    def __init__(self, window_sec: float = 30.0, warn_deg_min: float = 2.0, pos_eps: float = 0.005,
                 wheel_eps: float = 0.01):
        self.window = window_sec
        self.warn = warn_deg_min
        self.pos_eps = pos_eps
        self.wheel_eps = wheel_eps
        self._lock = threading.Lock()
        self._odom: deque = deque()     # (t, yaw_unwrapped, x, y)
        self._wheels: deque = deque()   # (t, (pos...))
        self._still_since: float | None = None

    # ---------- 입력 (ROS 콜백) ----------
    def add_odom(self, t: float, yaw: float, x: float, y: float):
        with self._lock:
            if self._odom:
                yaw = _unwrap(self._odom[-1][1], yaw)
                if t < self._odom[-1][0] or abs(x - self._odom[-1][2]) + abs(y - self._odom[-1][3]) > 0.5:
                    self._odom.clear()   # odom 초기화(/reset)·시간 역행 → 처음부터
                    self._still_since = None
            self._odom.append((t, yaw, x, y))
            self._trim(t)
            self._update_still(t)

    def add_wheels(self, t: float, positions):
        with self._lock:
            self._wheels.append((t, tuple(float(p) for p in positions)))
            self._trim(t)

    def reset(self):
        """재보정 직후 — 옛 기록은 버린다."""
        with self._lock:
            self._odom.clear()
            self._wheels.clear()
            self._still_since = None

    # ---------- 판정 ----------
    def _trim(self, now: float):
        for q in (self._odom, self._wheels):
            while q and now - q[0][0] > self.window:
                q.popleft()

    def _moving_recent(self) -> bool:
        o = self._odom
        if len(o) < 2:
            return True
        xs, ys = [s[2] for s in o], [s[3] for s in o]
        if max(xs) - min(xs) > self.pos_eps or max(ys) - min(ys) > self.pos_eps:
            return True
        w = self._wheels
        if len(w) >= 2:
            n = min(len(s[1]) for s in w)
            for i in range(n):
                vals = [s[1][i] for s in w]
                if max(vals) - min(vals) > self.wheel_eps:
                    return True
        return False

    def _update_still(self, now: float):
        if self._moving_recent():
            self._still_since = None
        elif self._still_since is None:
            self._still_since = self._odom[0][0]

    def snapshot(self, now: float) -> dict:
        """{state: unknown|moving|settling|ok|drift, rate_deg_min, still_sec, wheels}"""
        with self._lock:
            o = list(self._odom)
            still = None if self._still_since is None else now - self._still_since
            wheels = len(self._wheels) >= 2
        if len(o) < 2 or now - o[-1][0] > 2.0:
            return {'state': 'unknown', 'rate_deg_min': None, 'still_sec': None, 'wheels': wheels}
        if still is None:
            return {'state': 'moving', 'rate_deg_min': None, 'still_sec': 0.0, 'wheels': wheels}
        span = o[-1][0] - o[0][0]
        if still < self.window * 0.9 or span < self.window * 0.8:
            # 멈춘 지 얼마 안 됨 — 창 안에 실제 회전이 섞여 있을 수 있다
            return {'state': 'settling', 'rate_deg_min': None, 'still_sec': round(still, 1), 'wheels': wheels}
        rate = math.degrees(o[-1][1] - o[0][1]) / span * 60.0
        return {'state': 'drift' if abs(rate) >= self.warn else 'ok', 'rate_deg_min': round(rate, 2),
                'still_sec': round(still, 1), 'wheels': wheels}


def auto_reset_reason(watch: dict, *, enabled: bool, min_deg_min: float, mode: str, nav_active: bool,
                      teleop_active: bool, power_state: str, busy: bool, since_last: float | None,
                      cooldown_sec: float) -> str | None:
    """자동 재보정을 해도 되면 이유 문자열, 아니면 None.
    지도 만들기(MAPPING) 중엔 하지 않는다 — /reset 이 odom 을 0 으로 튀게 해 slam_toolbox 지도가 망가질 수 있다 (경고만)."""
    if not enabled or watch.get('state') != 'drift' or watch.get('rate_deg_min') is None:
        return None
    if abs(watch['rate_deg_min']) < min_deg_min or not watch.get('wheels'):
        return None   # 바퀴 정보 없이 '정지' 를 확신할 수 없으면 자동으로 하지 않음
    if mode == 'MAPPING' or nav_active or teleop_active or busy or power_state != 'running':
        return None
    if since_last is not None and since_last < cooldown_sec:
        return None
    return f"정지 중 방향이 {watch['rate_deg_min']:+.1f}°/분 돎 (바퀴 정지 {watch['still_sec']:.0f}초)"
