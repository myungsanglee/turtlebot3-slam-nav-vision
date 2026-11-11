"""rclpy 쪽: 로봇·Nav2 토픽을 구독해 웹이 바로 그릴 수 있는 형태로 정리해 둔다.

- 상태(P1): 토픽별 수신 주기·배터리·속도
- 카메라(P1): JPEG 바이트 그대로 보관 → MJPEG
- 지도 화면(P2): 지도·로컬 장애물 지도(PNG), 라이다 점·경로·로봇 외곽(지도 좌표 점 목록), 로봇 위치(TF)

모든 좌표는 fixed_frame(기본 map) 기준 미터로 바꿔서 내보낸다. 브라우저는 좌표 변환을 하지 않는다.
이 노드는 별도 스레드의 executor 에서 돈다 (server.py). 콜백은 짧게 — 변환·솎기만 하고 PNG 인코딩은
요청이 올 때 버전별로 한 번만 한다.
"""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy.time
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import compose, downsample, scan_to_points, transform_points, yaw_from_quaternion
from .map_codec import costmap_to_rgba, encode_png, map_to_rgba
from .monitor import FrameStore, TopicMonitor, TopicSpec, battery_level
from .nav import NavController

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

# /map 과 Nav2 costmap 은 "늦게 구독해도 마지막 값을 받는" transient local 로 publish 된다 → 같은 QoS 로 구독
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Versioned:
    """값 하나와 버전 번호. WebSocket 은 버전이 바뀐 것만 보낸다."""

    def __init__(self):
        self._lock = threading.Lock()
        self.value = None
        self.version = 0

    def set(self, value):
        with self._lock:
            self.value = value
            self.version += 1

    def get(self):
        with self._lock:
            return self.version, self.value


class GridStore:
    """OccupancyGrid 하나(지도 또는 costmap)와 PNG 캐시. 메타데이터는 fixed_frame 기준 원점."""

    def __init__(self, to_rgba):
        self._to_rgba = to_rgba
        self._lock = threading.Lock()
        self._grid = None
        self._meta = None
        self._png = None
        self._png_version = -1
        self.version = 0
        self.stamp = None           # 마지막 수신 (monotonic)

    def set(self, grid: np.ndarray, meta: dict):
        with self._lock:
            self.version += 1
            self._grid, self._meta = grid, {**meta, 'version': self.version}
            self.stamp = time.monotonic()

    def meta(self):
        with self._lock:
            return self._meta

    def png(self) -> bytes | None:
        with self._lock:
            if self._grid is None:
                return None
            if self._png_version != self.version:
                self._png = encode_png(self._to_rgba(self._grid))
                self._png_version = self.version
            return self._png


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
        self.fixed_frame = p('fixed_frame', 'map').value
        self._base_frame = p('base_frame', 'base_footprint').value
        self._scan_points = int(p('scan_max_points', 180).value)
        self._path_points = int(p('path_max_points', 300).value)

        # ---------- P1: 상태·카메라 ----------
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

        # ---------- P2: 지도 화면 ----------
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)   # /tf, /tf_static 구독 (같은 executor 에서)
        self.map = GridStore(map_to_rgba)
        self.costmap = GridStore(costmap_to_rgba)
        self.scan_pts = Versioned()       # [[x, y], ...]  fixed_frame
        self.paths = {'global': Versioned(), 'local': Versioned()}
        self.footprint = Versioned()
        self.create_subscription(OccupancyGrid, p('topic_map', '/map').value, self._on_map, LATCHED_QOS)
        self.create_subscription(OccupancyGrid, p('topic_costmap', '/local_costmap/costmap').value,
                                 self._on_costmap, LATCHED_QOS)
        self.create_subscription(Path, p('topic_plan', '/plan').value,
                                 lambda m: self._on_path('global', m), 10)
        self.create_subscription(Path, p('topic_local_plan', '/local_plan').value,
                                 lambda m: self._on_path('local', m), 10)
        self.create_subscription(PolygonStamped, p('topic_footprint', '/local_costmap/published_footprint').value,
                                 self._on_footprint, 10)
        self.nav = NavController(self, p('nav_action', 'navigate_to_pose').value,
                                 p('topic_initialpose', '/initialpose').value, self.fixed_frame)
        self._nav2_ready = False
        self.create_timer(1.0, self._poll_nav2)

        self.get_logger().info(f'web_api: http://{self.host}:{self.port}  static_dir={self.static_dir or "(없음 — API 만)"}'
                               f'  fixed_frame={self.fixed_frame}')

    # ---------- 공통: TF ----------
    def lookup(self, source_frame: str):
        """fixed_frame ← source_frame 변환 (x, y, yaw). 최신 값을 쓴다 — 브리지 지연으로 메시지 시각에 정확히
        맞추면 'extrapolation' 오류가 잦고, 화면 표시용으로는 수십 ms 차이가 보이지 않는다. 없으면 None."""
        try:
            t = self._tf.lookup_transform(self.fixed_frame, source_frame, rclpy.time.Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def robot_pose(self):
        pose = self.lookup(self._base_frame)
        if pose is None:
            return None
        return {'x': round(pose[0], 3), 'y': round(pose[1], 3), 'yaw': round(pose[2], 4)}

    # ---------- 콜백 ----------
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
            elif key == 'scan':
                self._on_scan(msg)
            elif key in self.frames:
                self.frames[key].put(bytes(msg.data))
        return cb

    def _on_scan(self, msg: LaserScan):
        t = self.lookup(msg.header.frame_id)
        if t is None:
            return
        local = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max, self._scan_points)
        self.scan_pts.set(transform_points(t, local))

    @staticmethod
    def _grid(msg: OccupancyGrid) -> np.ndarray:
        return np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    @staticmethod
    def _origin(msg: OccupancyGrid):
        o = msg.info.origin
        return o.position.x, o.position.y, yaw_from_quaternion(o.orientation.x, o.orientation.y,
                                                               o.orientation.z, o.orientation.w)

    def _grid_meta(self, msg: OccupancyGrid, origin) -> dict:
        return {'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
                'origin': {'x': round(origin[0], 4), 'y': round(origin[1], 4), 'yaw': round(origin[2], 5)}}

    def _on_map(self, msg: OccupancyGrid):
        if msg.header.frame_id and msg.header.frame_id != self.fixed_frame:
            self.get_logger().warn(f'/map frame {msg.header.frame_id} != fixed_frame {self.fixed_frame}',
                                   throttle_duration_sec=30.0)
        self.map.set(self._grid(msg), self._grid_meta(msg, self._origin(msg)))

    def _on_costmap(self, msg: OccupancyGrid):
        # 로컬 costmap 은 odom 기준 → fixed_frame(map) 기준 원점으로 옮긴다 (회전 포함)
        t = self.lookup(msg.header.frame_id)
        if t is None:
            return
        origin = compose(t, self._origin(msg))
        self.costmap.set(self._grid(msg), {**self._grid_meta(msg, origin), 'frame': msg.header.frame_id})

    def _on_path(self, name: str, msg: Path):
        if not msg.poses:
            self.paths[name].set([])     # 빈 경로 = 지우기 (목표 도착·취소)
            return
        t = self.lookup(msg.header.frame_id)
        if t is None:
            return
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.paths[name].set(transform_points(t, downsample(pts, self._path_points)))

    def _on_footprint(self, msg: PolygonStamped):
        t = self.lookup(msg.header.frame_id)
        if t is None:
            return
        self.footprint.set(transform_points(t, [(pt.x, pt.y) for pt in msg.polygon.points]))

    def _poll_nav2(self):
        self._nav2_ready = self.nav.server_ready()

    # ---------- 상태 ----------
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

        map_age = None if self.map.stamp is None else round(time.monotonic() - self.map.stamp, 1)
        return {
            'time': time.time(),
            'mode': 'IDLE',                                   # P3 모드 관리자가 채운다
            'robot_connected': by_key['odom']['state'] == 'ok',
            'battery': battery_out,
            'velocity': velocity_out,
            'navigation': {
                'fixed_frame': self.fixed_frame,
                'localized': self.lookup(self._base_frame) is not None,   # map→base 가 이어져야 지도 위 위치가 있다
                'map_age': map_age,
                'nav2_ready': self._nav2_ready,
            },
            'topics': topics,
        }
