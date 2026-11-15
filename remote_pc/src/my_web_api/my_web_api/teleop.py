"""웹 조이스틱 → /cmd_vel. 안전 장치가 핵심이다.

- 브라우저는 누르고 있는 동안 0.1초마다 (x, y) ∈ [-1, 1] 을 보낸다 (조이스틱 위 = 전진, 왼쪽 = 좌회전).
- 서버 ROS 타이머(10Hz)가 마지막 명령이 **0.3초 이내**일 때만 속도를 내보낸다. 끊기면(손 뗌·탭 닫힘·와이파이 끊김)
  정지 명령(0, 0)을 몇 번 보내고 조용해진다 — 계속 0 을 보내면 Nav2 주행 명령과 싸우게 되므로.
- 속도는 서버가 상한으로 자른다 (브라우저 값을 믿지 않는다).
- 조이스틱을 움직이기 시작하면 진행 중인 Nav2 목표를 취소한다 (두 명령이 섞이지 않게).
"""
from __future__ import annotations

import threading
import time

from geometry_msgs.msg import Twist

STOP_REPEAT = 3   # 끊긴 뒤 정지 명령을 몇 번 보낼지 (브리지에서 하나쯤 잃어도 멈추게)


def to_velocity(x: float, y: float, max_linear: float, max_angular: float) -> tuple[float, float]:
    """조이스틱 (x 오른쪽+, y 위+) → (전진 m/s, 회전 rad/s). 좌회전이 + 이므로 x 의 부호를 뒤집는다.
    작은 떨림은 무시(데드존 0.08), 범위 밖 값은 잘라낸다."""
    x = max(-1.0, min(1.0, float(x)))
    y = max(-1.0, min(1.0, float(y)))
    dz = lambda v: 0.0 if abs(v) < 0.08 else v   # noqa: E731
    return dz(y) * max_linear, -dz(x) * max_angular


class TeleopController:
    def __init__(self, node, topic: str, max_linear: float, max_angular: float, timeout: float = 0.3,
                 on_start=None, clock=time.monotonic):
        self._pub = node.create_publisher(Twist, topic, 10)
        self.max_linear, self.max_angular, self._timeout = max_linear, max_angular, timeout
        self._on_start = on_start
        self._clock = clock
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0)
        self._stamp = None
        self._stop_left = 0
        self._active = False
        node.create_timer(0.1, self._tick)

    def command(self, x: float, y: float, scale: float = 1.0):
        """WebSocket(asyncio 스레드)에서 호출. scale 은 화면의 속도 조절 (0.1~1)."""
        s = max(0.1, min(1.0, float(scale)))
        v, w = to_velocity(x, y, self.max_linear * s, self.max_angular * s)
        with self._lock:
            self._cmd = (v, w)
            self._stamp = self._clock()

    def release(self):
        """손 뗌·연결 끊김: 즉시 정지로."""
        with self._lock:
            self._cmd = (0.0, 0.0)
            self._stamp = None

    def state(self) -> dict:
        with self._lock:
            return {'active': self._active, 'linear': round(self._cmd[0], 3), 'angular': round(self._cmd[1], 3),
                    'max_linear': self.max_linear, 'max_angular': self.max_angular}

    def _tick(self):   # ROS 스레드
        with self._lock:
            fresh = self._stamp is not None and self._clock() - self._stamp < self._timeout
            v, w = self._cmd if fresh else (0.0, 0.0)
        moving = fresh and (v != 0.0 or w != 0.0)
        if moving:
            if not self._active and self._on_start:
                self._on_start()            # 조이스틱 시작 → Nav2 목표 취소
            self._active = True
            self._stop_left = STOP_REPEAT
            self._publish(v, w)
        elif self._active or self._stop_left > 0:
            self._active = False
            self._publish(0.0, 0.0)
            self._stop_left -= 1

    def _publish(self, v: float, w: float):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self._pub.publish(msg)


class SpeedSetting:
    """화면의 속도 조절 하나로 조이스틱과 자동 주행(Nav2) 속도를 함께 정한다.

    - scale(0.2~1) × teleop 최대 속도 = 조이스틱 속도. 서버가 값을 들고 있어 여러 브라우저가 같은 값을 본다.
    - Nav2 에는 nav2_msgs/SpeedLimit(절대값, m/s)를 /speed_limit 으로 보낸다 → controller_server 가 DWB 의
      최대 속도를 낮춘다. DWB 는 직진 한도의 비율(limit / max_speed_xy)만큼 회전 한도도 같이 줄이므로,
      nav2_params 의 FollowPath max_vel_x·max_speed_xy·max_vel_theta 를 teleop 최대(0.15 m/s·1.0 rad/s)와 같게 두면
      직진·회전 모두 조이스틱과 같아진다.
    - controller_server 구독은 volatile 이라 Nav2 가 나중에 켜져도 받도록 1초마다 다시 보낸다.
    - 마지막 값은 파일에 남겨 web-api 재시작 후에도 유지한다.
    """
    MIN, MAX = 0.2, 1.0

    def __init__(self, node, topic: str, max_linear: float, max_angular: float, state_file: str | None = None,
                 default: float = 0.6):
        from nav2_msgs.msg import SpeedLimit
        self._msg_type = SpeedLimit
        self._pub = node.create_publisher(SpeedLimit, topic, 10)
        self.max_linear, self.max_angular = max_linear, max_angular
        self._file = state_file
        self._lock = threading.Lock()
        self._scale = self._load(default)
        node.create_timer(1.0, self.publish)

    def _clamp(self, s: float) -> float:
        return round(max(self.MIN, min(self.MAX, float(s))), 2)

    def _load(self, default: float) -> float:
        try:
            with open(self._file) as f:
                return self._clamp(float(f.read().strip()))
        except (OSError, TypeError, ValueError):
            return self._clamp(default)

    @property
    def scale(self) -> float:
        with self._lock:
            return self._scale

    def set(self, scale: float) -> float:
        s = self._clamp(scale)
        with self._lock:
            self._scale = s
        if self._file:
            try:
                with open(self._file, 'w') as f:
                    f.write(f'{s}\n')
            except OSError:
                pass
        self.publish()
        return s

    def snapshot(self) -> dict:
        s = self.scale
        return {'scale': s, 'linear': round(self.max_linear * s, 3), 'angular': round(self.max_angular * s, 3)}

    def publish(self):
        msg = self._msg_type()
        msg.percentage = False
        msg.speed_limit = float(self.max_linear * self.scale)
        self._pub.publish(msg)
