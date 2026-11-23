"""모터 상태 감시 — "로봇은 켜져 있는데 조이스틱·주행 명령에 안 움직인다" 를 웹에 알린다 (ROS 없는 순수 로직).

두 가지를 본다 (docs/my_web_api.md 4.26):
  1) 토크: OpenCR 의 /sensor_state.torque (turtlebot3_node 가 OpenCR 제어표의 motor_torque_enable 을 그대로 실음).
     false 면 속도 명령이 와도 바퀴가 힘을 내지 않는다 (2026-11-24 실제로 꺼져 있었음). /motor_power 로 다시 켠다.
  2) 무반응: /cmd_vel 로 움직이라는 명령이 1.5초 넘게 이어지는데 바퀴(/joint_states 속도)가 전혀 안 돈다.
     토크 정보가 없어도(브리지 설정 전·OpenCR 통신 끊김 등) 원인과 상관없이 잡힌다.
     바퀴가 헛도는 끼임은 여기 해당하지 않는다 (바퀴는 돈다).
"""
from __future__ import annotations

import threading

TORQUE_STALE_SEC = 3.0     # 이 시간 동안 /sensor_state 가 없으면 토크 '모름'
CMD_MIN_V = 0.02           # m/s   — 이보다 작은 명령은 '움직이라는 명령' 으로 보지 않음 (Nav2 미세 보정)
CMD_MIN_W = 0.15           # rad/s
CMD_GAP_SEC = 0.5          # 명령이 이만큼 끊기면 연속이 아님
WHEEL_MIN = 0.1            # rad/s — 바퀴가 이보다 빠르면 '돈다' (최저 속도 20% 도 0.5 rad/s 이상)
NO_RESPONSE_SEC = 1.5      # 명령이 이만큼 이어졌는데 바퀴가 안 돌면 무반응
HOLD_SEC = 5.0             # 무반응 표시를 명령이 끝난 뒤에도 잠깐 유지 (사용자가 볼 수 있게)


class MotorWatch:
    def __init__(self):
        self._lock = threading.Lock()
        self._torque = None
        self._torque_at = None
        self._cmd_since = None     # 움직이라는 명령이 이어지기 시작한 시각
        self._cmd_last = None      # 마지막으로 움직이라는 명령을 받은 시각
        self._wheel_at = None      # 마지막으로 바퀴가 돈 시각
        self._wheel_pos = None
        self._no_resp_at = None    # 마지막으로 무반응을 판정한 시각

    def set_torque(self, t: float, torque: bool):
        with self._lock:
            self._torque, self._torque_at = bool(torque), t

    def add_cmd(self, t: float, v: float, w: float):
        with self._lock:
            if abs(v) >= CMD_MIN_V or abs(w) >= CMD_MIN_W:
                if self._cmd_since is None or self._cmd_last is None or t - self._cmd_last > CMD_GAP_SEC:
                    self._cmd_since = t
                self._cmd_last = t
            else:
                self._cmd_since = None
            self._check(t)

    def add_wheels(self, t: float, velocity, position=None):
        """/joint_states 의 바퀴 속도(rad/s). 속도가 비어 있으면 위치 변화로 계산."""
        with self._lock:
            speed = None
            if velocity:
                speed = max(abs(x) for x in velocity)
            elif position:
                if self._wheel_pos is not None:
                    pt, pp = self._wheel_pos
                    if t > pt:
                        speed = max(abs(a - b) for a, b in zip(position, pp)) / (t - pt)
                self._wheel_pos = (t, list(position))
            if speed is not None and speed >= WHEEL_MIN:
                self._wheel_at = t
                self._no_resp_at = None
            self._check(t)

    def _check(self, t: float):
        if self._cmd_since is None or self._cmd_last is None or t - self._cmd_last > CMD_GAP_SEC:
            return
        if t - self._cmd_since < NO_RESPONSE_SEC:
            return
        if self._wheel_at is not None and self._wheel_at >= self._cmd_since:
            return
        self._no_resp_at = t

    def snapshot(self, now: float) -> dict:
        with self._lock:
            torque = self._torque if self._torque_at is not None and now - self._torque_at < TORQUE_STALE_SEC else None
            no_resp = self._no_resp_at is not None and now - self._no_resp_at < HOLD_SEC
        if torque is False:
            state = 'torque_off'
        elif no_resp:
            state = 'no_response'
        elif torque is None:
            state = 'unknown'
        else:
            state = 'ok'
        return {'state': state, 'torque': torque}
