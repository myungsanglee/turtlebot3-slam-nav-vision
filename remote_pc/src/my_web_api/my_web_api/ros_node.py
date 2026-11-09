"""rclpy 쪽: 로봇 토픽을 구독해 상태(주기·배터리·속도)와 최신 카메라 프레임을 모은다.

이 노드는 별도 스레드의 executor 에서 돈다 (server.py). 콜백은 짧게 — 기록만 하고 계산은 snapshot() 때 한다.
카메라는 JPEG 바이트를 디코딩하지 않고 그대로 보관한다 → MJPEG 로 재인코딩 없이 브라우저에 흘려보낸다.
"""
from __future__ import annotations

import threading
import time

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan

from .monitor import FrameStore, TopicMonitor, TopicSpec, battery_level

# key: (파라미터 이름, 메시지 타입, 한글 이름, 기대 Hz, optional)
#   optional=True: 안 들어와도 오류가 아님 (Vision 은 SLAM/주행 모드에서만 켜짐)
TOPICS = {
    'odom':    ('topic_odom', Odometry, '오도메트리', 20.0, False),
    'scan':    ('topic_scan', LaserScan, '라이다', 5.0, False),
    'battery': ('topic_battery', BatteryState, '배터리', 20.0, False),
    'color':   ('topic_color', CompressedImage, '카메라 컬러', 6.0, False),
    'depth':   ('topic_depth', CompressedImage, '카메라 depth', 6.0, False),
    'vision':  ('topic_vision', CompressedImage, 'Vision 검출 영상', 6.0, True),
}


class WebApiNode(Node):
    def __init__(self):
        super().__init__('web_api')
        p = self.declare_parameter
        self.host = p('host', '0.0.0.0').value
        self.port = int(p('port', 8080).value)
        self.static_dir = p('static_dir', '').value
        self.status_period = float(p('status_period_sec', 0.5).value)
        self._cells = int(p('battery_cells', 3).value)
        self._warn_v = float(p('battery_warn_cell_v', 3.5).value)
        self._crit_v = float(p('battery_critical_cell_v', 3.3).value)

        self.monitors: dict[str, TopicMonitor] = {}
        self.frames = {'color': FrameStore(), 'vision': FrameStore()}
        self._lock = threading.Lock()
        self._battery = None     # (voltage, percentage, 수신 시각)
        self._velocity = None    # (linear.x, angular.z)

        for key, (param, msg_type, label, hz, optional) in TOPICS.items():
            topic = p(param, '').value
            self.monitors[key] = TopicMonitor(TopicSpec(key, topic, label, hz, optional))
            # 센서 데이터 QoS(best effort): /scan·카메라는 publisher 가 best effort 라 reliable 로 구독하면 못 받는다.
            # reliable publisher(/odom 등)도 best effort 구독은 받을 수 있으므로 전부 이 QoS 로 통일.
            self.create_subscription(msg_type, topic, self._make_cb(key), qos_profile_sensor_data)
        self.get_logger().info(f'web_api: http://{self.host}:{self.port}  static_dir={self.static_dir or "(없음 — API 만)"}')

    def _make_cb(self, key):
        monitor = self.monitors[key]

        def cb(msg):
            monitor.record()
            if key == 'battery':
                with self._lock:
                    self._battery = (float(msg.voltage), float(msg.percentage), time.monotonic())
            elif key == 'odom':
                with self._lock:
                    self._velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
            elif key in self.frames:
                self.frames[key].put(bytes(msg.data))
        return cb

    def snapshot(self) -> dict:
        """GET /api/status 와 WebSocket 'status' 메시지의 내용."""
        topics = [m.snapshot() for m in self.monitors.values()]
        by_key = {t['key']: t for t in topics}
        with self._lock:
            battery, velocity = self._battery, self._velocity

        battery_out = None
        if battery is not None and by_key['battery']['state'] == 'ok':
            voltage, percentage, _ = battery
            battery_out = {
                'voltage': round(voltage, 2),
                # OpenCR 펌웨어 계산값. 실측 3점이 10.5V=0%·12.3V=100% 직선과 일치 → 충전 중엔 100% 를 넘는다.
                # LiPo 실제 잔량과는 비선형이라 참고용으로만 0~100 으로 잘라 준다 (판정은 level = 셀 전압).
                'percentage': round(min(100.0, max(0.0, percentage)), 1),
                'percentage_raw': round(percentage, 1),
                'cells': self._cells,
                'cell_voltage': round(voltage / self._cells, 2),
                'level': battery_level(voltage, self._cells, self._warn_v, self._crit_v),
            }
        velocity_out = None
        if velocity is not None and by_key['odom']['state'] == 'ok':
            velocity_out = {'linear': round(velocity[0], 3), 'angular': round(velocity[1], 3)}

        return {
            'time': time.time(),
            'mode': 'IDLE',                                   # P3 모드 관리자가 채운다
            'robot_connected': by_key['odom']['state'] == 'ok',
            'battery': battery_out,
            'velocity': velocity_out,
            'topics': topics,
        }
