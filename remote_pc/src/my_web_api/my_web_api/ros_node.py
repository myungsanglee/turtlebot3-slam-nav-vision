"""rclpy 쪽: 로봇·Nav2 토픽을 구독해 웹이 바로 그릴 수 있는 형태로 정리해 둔다.

- 상태(P1): 토픽별 수신 주기·배터리·속도
- 카메라(P1): JPEG 바이트 그대로 보관 → MJPEG
- 지도 화면(P2): 지도·로컬 장애물 지도(PNG), 라이다 점·경로·로봇 외곽(지도 좌표 점 목록), 로봇 위치(TF)
- 카메라 depth 점: 정렬 depth → 지도 좌표 3D 점 (depth_points.py, Nav2 없이도 표시)
- 금지 구역(zones.py): 저장 지도로 주행 중이면 그 지도의 구역을 마스크로 만들어 Nav2 Keepout Filter 에 발행

모든 좌표는 '표시 좌표계' 기준 미터로 바꿔서 내보낸다. 브라우저는 좌표 변환을 하지 않는다.
표시 좌표계 = fixed_frame(map) — SLAM·AMCL 이 돌아 map→로봇 TF 가 있을 때.
              없으면 odom_frame(odom, 로봇이 켜진 위치 기준)으로 자동 전환해 라이다·로봇을 계속 실시간으로 보여준다
              (지도 없이 로봇만 켜 둔 대기 상태에서도 라이다가 멈추지 않게 — 2026-11-12 수정).
이 노드는 별도 스레드의 executor 에서 돈다 (server.py). 콜백은 짧게 — 변환·솎기만 하고 PNG 인코딩은
요청이 올 때 버전별로 한 번만 한다.
"""
from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rclpy.time
from geometry_msgs.msg import PolygonStamped
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .depth_points import PixelRays, depth_to_display, quat_matrix
from .geometry import compose, downsample, scan_to_points, transform_points, yaw_from_quaternion
from .map_codec import costmap_to_rgba, encode_png, map_to_rgba
from .monitor import FrameStore, TopicMonitor, TopicSpec, battery_level
from .maps import MapStore
from .mode_manager import ModeManager
from .nav import NavController
from .robot_model import RobotModel
from .robot_power import RobotPower
from .zones import rasterize
from .teleop import TeleopController

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
    """값 하나와 버전 번호. WebSocket 은 버전이 바뀐 것만 보낸다. stamp = 마지막 갱신 (monotonic)."""

    def __init__(self):
        self._lock = threading.Lock()
        self.value = None
        self.version = 0
        self.stamp = None

    def set(self, value):
        with self._lock:
            self.value = value
            self.version += 1
            self.stamp = time.monotonic()

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
        self.odom_frame = p('odom_frame', 'odom').value
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
        self.scan_pts = Versioned()       # {'frame': 표시 좌표계, 'points': [[x, y], ...]}
        self.paths = {'global': Versioned(), 'local': Versioned()}
        self.footprint = Versioned()
        # 카메라 depth 점 (표시용): {'frame', 'points': [[x, y, z], ...], 'band': [장애물 높이 하한, 상한]}
        self.depth_pts = Versioned()
        self._depth_period = float(p('depth_points_period_sec', 0.25).value)
        self._depth_stride = int(p('depth_points_stride', 8).value)
        self._depth_range = (float(p('depth_points_min_range', 0.2).value), float(p('depth_points_max_range', 3.0).value))
        self._depth_voxel = float(p('depth_points_voxel', 0.05).value)
        self._depth_zmax = float(p('depth_points_z_max', 1.0).value)
        self._depth_band = [float(p('depth_obstacle_min_height', 0.04).value),
                            float(p('depth_obstacle_max_height', 0.25).value)]
        self._depth_last = 0.0
        self._depth_rays = None
        self._cam_info = None
        self.create_subscription(CameraInfo, p('topic_camera_info', '/camera/color/camera_info').value,
                                 lambda m: setattr(self, '_cam_info', m), qos_profile_sensor_data)
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

        # 로봇 모양: Pi 가 보내는 URDF(transient local) → base_link 메시 → 위에서 본 이미지·3D 메시
        self.robot_model = RobotModel(self.get_logger())

        # ---------- P3: 모드 관리·지도 저장·조이스틱 ----------
        self.maps = MapStore(p('maps_dir', '/overlay_ws/maps').value)
        self._graph = set()
        self.create_timer(1.0, self._poll_graph)
        self.mode = ModeManager(self.get_logger(), self.maps, self._mode_ready, lambda: self._graph,
                                on_stop=lambda: self.nav.submit('cancel'),
                                ready_timeout=float(p('mode_ready_timeout_sec', 120.0).value))
        self.teleop = TeleopController(self, p('topic_cmd_vel', '/cmd_vel').value,
                                       float(p('teleop_max_linear', 0.15).value),
                                       float(p('teleop_max_angular', 1.0).value),
                                       on_start=lambda: self.nav.submit('cancel'))

        # ---------- 로봇 전원 (대기·켜기·재시작·재부팅·끄기) — Pi 에 전용 SSH 키로 ----------
        self.power = RobotPower(self.get_logger(), p('robot_power_host', '').value, p('robot_power_user', 'michael').value,
                                p('robot_power_key', '/secrets/robot_power_key').value,
                                p('robot_power_known_hosts', '/secrets/known_hosts').value,
                                robot_connected_fn=lambda: self.monitors['odom'].snapshot()['state'] == 'ok',
                                before_stop_fn=self._stop_for_power)

        # ---------- 금지 구역 → Nav2 Keepout Filter ----------
        # 필터 정보(어느 토픽의 마스크를 쓸지)와 마스크를 transient local 로 발행 — Nav2 가 나중에 떠도 마지막 값을 받는다.
        # 마스크가 바뀌면 Nav2 가 실행 중에도 새 마스크로 바꿔 끼운다 (KeepoutFilter::maskCallback).
        mask_topic = p('topic_keepout_mask', '/keepout_filter_mask').value
        self._keepout_mask_pub = self.create_publisher(OccupancyGrid, mask_topic, LATCHED_QOS)
        info_pub = self.create_publisher(CostmapFilterInfo, p('topic_keepout_info', '/costmap_filter_info').value,
                                         LATCHED_QOS)
        info = CostmapFilterInfo()
        info.header.frame_id = self.fixed_frame
        info.type = 0                  # KEEPOUT_FILTER
        info.filter_mask_topic = mask_topic
        info.base, info.multiplier = 0.0, 1.0   # keepout 은 기본값이어야 함
        info_pub.publish(info)
        self._keepout_key = None
        self._keepout_lock = threading.Lock()   # ROS 타이머와 웹 요청(PUT zones)이 동시에 부를 수 있다
        self.keepout = {'map': None, 'zones': 0}
        self.create_timer(1.0, self.sync_keepout)

        self.create_subscription(String, p('topic_robot_description', '/robot_description').value,
                                 lambda m: self.robot_model.update(m.data), LATCHED_QOS)

        self.get_logger().info(f'web_api: http://{self.host}:{self.port}  static_dir={self.static_dir or "(없음 — API 만)"}'
                               f'  fixed_frame={self.fixed_frame}')

    # ---------- 공통: TF ----------
    def lookup(self, target_frame: str, source_frame: str):
        """target ← source 변환 (x, y, yaw). 최신 값을 쓴다 — 브리지 지연으로 메시지 시각에 정확히 맞추면
        'extrapolation' 오류가 잦고, 화면 표시용으로는 수십 ms 차이가 보이지 않는다. 없으면 None."""
        try:
            t = self._tf.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def display_frame(self) -> str | None:
        """지금 화면을 그릴 좌표계. map→로봇 이 이어지면 map, 아니면 odom, 둘 다 없으면 None."""
        for frame in (self.fixed_frame, self.odom_frame):
            if self.lookup(frame, self._base_frame) is not None:
                return frame
        return None

    def robot_pose(self):
        frame = self.display_frame()
        pose = None if frame is None else self.lookup(frame, self._base_frame)
        if pose is None:
            return None
        return {'frame': frame, 'x': round(pose[0], 3), 'y': round(pose[1], 3), 'yaw': round(pose[2], 4)}

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
            elif key == 'depth':
                self._on_depth(msg)
            elif key in self.frames:
                self.frames[key].put(bytes(msg.data))
        return cb

    def _on_scan(self, msg: LaserScan):
        frame = self.display_frame()
        t = None if frame is None else self.lookup(frame, msg.header.frame_id)
        if t is None:
            # 좌표계가 없으면 '빈 목록'을 보내 화면이 멈춘 옛 점을 지우게 한다 (이전 값을 남겨 두지 않는다)
            self.scan_pts.set({'frame': None, 'points': []})
            return
        local = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max, self._scan_points)
        self.scan_pts.set({'frame': frame, 'points': transform_points(t, local)})

    def sync_keepout(self, force: bool = False):
        """지금 주행 중인 저장 지도의 금지 구역을 마스크로 발행. 지도·구역 파일이 바뀌었을 때만 (1초마다 확인).
        저장 지도 주행이 아니면(대기·SLAM) 빈 마스크 — SLAM 지도는 켤 때마다 원점이 달라 구역을 둘 수 없다."""
        with self._keepout_lock:
            self._sync_keepout(force)

    def _sync_keepout(self, force: bool):
        st = self.mode.snapshot()
        name = st['map'] if st['mode'] == 'NAVIGATION' and st['phase'] in ('starting', 'running') else None
        try:
            key = (name, self.maps.zones_mtime(name) if name else 0.0)
            if key == self._keepout_key and not force:
                return
            zones = self.maps.get_zones(name) if name else []
            g = self.maps.geometry(name) if name else {'width': 1, 'height': 1, 'resolution': 0.05,
                                                        'origin_x': 0.0, 'origin_y': 0.0}
            grid = rasterize(zones, g['width'], g['height'], g['resolution'], g['origin_x'], g['origin_y'])
        except Exception as e:  # noqa: BLE001 — 구역 파일이 깨져도 웹은 계속
            self.get_logger().error(f'금지 구역 마스크 생성 실패: {e}')
            return
        msg = OccupancyGrid()
        msg.header.frame_id = self.fixed_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = g['resolution']
        msg.info.width, msg.info.height = g['width'], g['height']
        msg.info.origin.position.x, msg.info.origin.position.y = g['origin_x'], g['origin_y']
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self._keepout_mask_pub.publish(msg)
        self._keepout_key = key
        self.keepout = {'map': name, 'zones': len(zones)}
        self.get_logger().info(f'금지 구역 마스크 발행: 지도={name or "(없음)"} 구역 {len(zones)}개 '
                               f'({g["width"]}x{g["height"]}, 막힌 칸 {int((grid > 0).sum())})')

    def lookup3d(self, target_frame: str, source_frame: str):
        """3D 변환 (회전 행렬, 이동). 카메라처럼 기울어진 좌표계용. 없으면 None."""
        try:
            t = self._tf.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return quat_matrix(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z], dtype=np.float32)

    def _on_depth(self, msg: CompressedImage):
        """depth 6Hz 중 depth_points_period 마다 한 장만 계산 (전송량·CPU 절약)."""
        now = time.monotonic()
        if now - self._depth_last < self._depth_period or self._cam_info is None:
            return
        self._depth_last = now
        frame = self.display_frame()
        rt = None if frame is None else self.lookup3d(frame, msg.header.frame_id)
        if rt is None:
            self.depth_pts.set({'frame': None, 'points': [], 'band': self._depth_band})
            return
        depth = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if depth is None:
            return
        h, w = depth.shape[:2]
        key = (tuple(self._cam_info.k), w, h, self._depth_stride)
        if self._depth_rays is None or self._depth_rays.key != key:
            self._depth_rays = PixelRays(list(self._cam_info.k), w, h, self._depth_stride)
        pts = depth_to_display(depth, self._depth_rays, rt[0], rt[1], *self._depth_range,
                               self._depth_voxel, self._depth_zmax)
        self.depth_pts.set({'frame': frame, 'points': pts, 'band': self._depth_band})

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
        # 로컬 costmap 은 odom 기준 → 표시 좌표계(보통 map) 기준 원점으로 옮긴다 (회전 포함)
        frame = self.display_frame()
        t = None if frame is None else self.lookup(frame, msg.header.frame_id)
        if t is None:
            return
        origin = compose(t, self._origin(msg))
        self.costmap.set(self._grid(msg), {**self._grid_meta(msg, origin), 'frame': frame})

    def _on_path(self, name: str, msg: Path):
        if not msg.poses:
            self.paths[name].set([])     # 빈 경로 = 지우기 (목표 도착·취소)
            return
        frame = self.display_frame()
        t = None if frame is None else self.lookup(frame, msg.header.frame_id)
        if t is None:
            return
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.paths[name].set(transform_points(t, downsample(pts, self._path_points)))

    def _on_footprint(self, msg: PolygonStamped):
        frame = self.display_frame()
        t = None if frame is None else self.lookup(frame, msg.header.frame_id)
        if t is None:
            return
        self.footprint.set(transform_points(t, [(pt.x, pt.y) for pt in msg.polygon.points]))

    def _stop_for_power(self):
        """로봇을 대기·재시작·재부팅·끄기 전에: 조이스틱·주행 목표를 멈추고, 웹에서 띄운 SLAM·Nav2 를 끈다
        (센서가 사라진 채 Nav2 가 돌면 오류만 쌓이고, 지도 만들기 중이면 지도가 망가진다)."""
        self.teleop.release()
        self.nav.submit('cancel')
        if self.mode.snapshot()['mode'] != 'IDLE':
            self.mode.request('IDLE')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 60.0:
            st = self.mode.snapshot()
            if st['mode'] == 'IDLE' and st['phase'] in ('idle', 'error'):
                return
            time.sleep(0.5)

    def _poll_nav2(self):
        self._nav2_ready = self.nav.server_ready()

    def _poll_graph(self):
        self._graph = {name for name, _ns in self.get_node_names_and_namespaces()}

    def _mode_ready(self, mode: str, started_wall: float) -> bool:
        """모드 시작이 끝났나: Nav2 액션 서버가 뜨고, 시작 뒤에 지도가 들어왔으면 준비 완료."""
        if not self._nav2_ready or self.map.stamp is None:
            return False
        map_wall = time.time() - (time.monotonic() - self.map.stamp)   # 지도 수신 시각(실제 시각)
        return map_wall >= started_wall

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

        now = time.monotonic()
        age = lambda stamp: None if stamp is None else round(now - stamp, 1)   # noqa: E731
        map_age = age(self.map.stamp)
        display = self.display_frame()
        return {
            'time': time.time(),
            'mode': self.mode.snapshot()['mode'],
            'power': self.power.snapshot(),        # 로봇 전원 상태 (running / standby / …) — robot_power.py
            'robot_connected': by_key['odom']['state'] == 'ok',
            'battery': battery_out,
            'velocity': velocity_out,
            'navigation': {
                'fixed_frame': self.fixed_frame,
                'display_frame': display,          # 화면 좌표계 (map / odom / None)
                'localized': display == self.fixed_frame,   # map→로봇 이 이어져야 지도 위 위치가 있다
                'map_age': map_age,
                # Nav2 가 꺼지면 costmap·footprint 는 더 오지 않는다 → 화면이 오래된 것을 숨기는 기준
                # (publish 주기: costmap 2Hz, footprint ≈5Hz. 지도는 저장 지도면 한 번만 오므로 age 로 숨기지 않는다)
                'costmap_age': age(self.costmap.stamp),
                'footprint_age': age(self.footprint.stamp),
                'nav2_ready': self._nav2_ready,
                'keepout': self.keepout,           # Nav2 에 적용 중인 금지 구역 (지도 이름·개수)
            },
            'topics': topics,
        }
