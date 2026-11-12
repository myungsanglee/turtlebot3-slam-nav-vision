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
