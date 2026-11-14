"""자동 위치 찾기 노드 — 저장 지도 주행에서 '지도 위 어디인지'를 스스로 찾아 AMCL 에 알려 준다.

AMCL 은 "대략 여기쯤" 이라는 초기 위치를 받아야 그 근처만 정밀하게 추적한다 (지도 전체를 뒤지지 않음).
이 노드가 그 초기 위치를 자동으로 만든다 (알고리즘: scan_matcher.py, 설명: docs/my_localization.md).

  시작할 때    마지막으로 저장한 위치(<지도>.pose.json) 근처를 스캔으로 확인 → 맞으면 그대로
               아니면(로봇을 옮겨 놓음) 지도 전체 검색
  요청할 때    ~/relocalize (std_srvs/Trigger) → 지도 전체 검색 (웹 「위치 찾기」 버튼)
  잃었을 때    1초마다 현재 위치에서 스캔이 지도와 맞는지 점수를 매기고, lost_sec 동안 계속 낮으면
               '위치 잃음' → retry_sec 마다 지도 전체 검색 (로봇을 들어 옮긴 경우 — 내려놓으면 다시 찾음)

결과는 /initialpose (AMCL 이 받는 '2D Pose Estimate' 와 같은 메시지) 로 보내고,
상태는 /relocalization/status (std_msgs/String, JSON, transient local) 로 알린다:
  {"state": waiting|searching|localized|ambiguous|failed|lost, "method": last_pose|global|manual|recovered,
   "score", "track_score", "pose", "candidates": [{x,y,yaw,score}], "message", "time"}
확신이 없으면(비슷한 곳이 여럿·점수 낮음) 위치를 보내지 않는다 — 웹이 후보를 보여 주고 사람이 고른다.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .scan_matcher import Candidate, MapModel, Scan, SearchParams, decide, global_search, local_search, rank


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_file_for(map_yaml: str) -> str:
    return os.path.splitext(map_yaml)[0] + '.pose.json' if map_yaml else ''


class Relocalizer(Node):
    def __init__(self):
        super().__init__('relocalizer')
        P = self.declare_parameter
        self.base_frame = P('base_frame', 'base_footprint').value
        self.global_frame = P('global_frame', 'map').value
        self.pose_file = pose_file_for(P('map_yaml', '').value)
        self.accept_score = P('accept_score', 0.4).value          # 최종 점수(우도×빔 일관성) 하한
        self.margin = P('ambiguity_margin', 0.1).value            # 1등이 2등(다른 곳)보다 이만큼 높아야 확신
        self.last_radius = P('last_pose_radius', 0.5).value       # 마지막 위치 확인 범위 [m]
        self.last_angle = P('last_pose_angle_deg', 30.0).value
        self.auto_start = P('auto_start', True).value
        self.lost_check = P('lost_check', True).value
        self.lost_score = P('lost_score', 0.3).value              # 추적 점수(검색과 같은 최종 점수)가 이보다 낮으면 의심
        self.lost_sec = P('lost_sec', 4.0).value                  # 이만큼 계속 낮으면 '위치 잃음'
        self.retry_sec = P('retry_sec', 3.0).value                # 잃은 동안 전체 검색 간격
        self.save_sec = P('pose_save_sec', 2.0).value
        self.max_range = P('max_range', 3.5).value
        self.params = SearchParams()

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, P('map_topic', '/map').value, self._on_map, latched)
        self.create_subscription(LaserScan, P('scan_topic', '/scan').value, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, P('amcl_pose_topic', '/amcl_pose').value,
                                 self._on_amcl_pose, 10)
        initialpose_topic = P('initialpose_topic', '/initialpose').value
        self.create_subscription(PoseWithCovarianceStamped, initialpose_topic, self._on_initialpose, 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, initialpose_topic, 10)
        self.pub_status = self.create_publisher(String, P('status_topic', '/relocalization/status').value, latched)
        self.create_service(Trigger, '~/relocalize', self._srv_relocalize)
        self.tf = Buffer()
        TransformListener(self.tf, self, spin_thread=False)

        self.lock = threading.Lock()
        self.map: MapModel | None = None
        self.grid: np.ndarray | None = None
        self.map_stamp = None
        self.scan_msg: LaserScan | None = None
        self.sensor: tuple | None = None        # base_footprint 기준 라이다 자세 (x, y, yaw)
        self.busy = False
        self.sent_at = 0.0                      # 마지막으로 /initialpose 를 보낸 시각 (자기 메시지 구분)
        self.grace_until = 0.0                  # 이 시각까지 추적 점수 검사 안 함 (AMCL 이 새 위치로 옮겨 갈 시간)
        self.low_since: float | None = None
        self.last_try = 0.0
        self.last_saved = 0.0
        self.status = {'state': 'waiting', 'method': None, 'score': None, 'track_score': None, 'pose': None,
                       'candidates': [], 'message': '지도·라이다 기다리는 중', 'time': time.time()}
        self._publish_status()
        self.started = False
        self.create_timer(1.0, self._tick)

    # ---------- 입력 ----------
    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        grid = np.asarray(msg.data, np.int8).reshape(info.height, info.width)
        m = MapModel(grid, info.resolution, (info.origin.position.x, info.origin.position.y))
        with self.lock:
            self.map, self.grid, self.map_stamp = m, grid, msg.header.stamp
        self.get_logger().info(f'지도 받음 {info.width}x{info.height} @ {info.resolution} m, 후보 칸 {int(m.free.sum())}')

    def _on_scan(self, msg: LaserScan):
        with self.lock:
            self.scan_msg = msg
        if self.sensor is None:
            try:
                t = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
                q = t.transform.rotation
                self.sensor = (t.transform.translation.x, t.transform.translation.y, yaw_of(q))
                self.get_logger().info(f'라이다 장착 자세 {msg.header.frame_id}: {self.sensor}')
            except Exception:   # noqa: BLE001 — 정적 TF 가 아직 안 들어옴
                pass

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """추적 중인 위치를 지도 옆 파일에 저장 → 다음에 켤 때 '마지막 위치' 로 먼저 확인."""
        if self.status['state'] != 'localized' or not self.pose_file or time.time() - self.last_saved < self.save_sec:
            return
        p = msg.pose.pose
        self._save_pose(p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_initialpose(self, msg: PoseWithCovarianceStamped):
        if time.time() - self.sent_at < 0.5:
            return   # 내가 보낸 것
        p = msg.pose.pose
        self.low_since = None
        self._set(state='localized', method='manual', score=None, candidates=[],
                  pose={'x': p.position.x, 'y': p.position.y, 'yaw': yaw_of(p.orientation)},
                  message='사용자가 위치 지정')
        self.grace_until = time.time() + 3.0

    def _srv_relocalize(self, req, res):
        ok, msg = self._start('global')
        res.success, res.message = ok, msg
        return res

    # ---------- 주기 ----------
    def _tick(self):
        if self.map is None or self.scan_msg is None or self.sensor is None:
            return
        if not self.started:
            self.started = True
            if self.auto_start:
                self._start('start')
            else:
                self._set(state='failed', message='자동 시작 꺼짐 — 「위치 찾기」 또는 직접 지정')
            return
        if self.busy or not self.lost_check or self.status['state'] not in ('localized', 'lost'):
            return
        if time.time() < self.grace_until:
            return   # 위치를 막 정함 — AMCL 이 따라올 시간
        track = self._track_score()
        if track is None:
            return
        self.status['track_score'] = round(track, 3)
        now = time.time()
        if track >= self.lost_score:
            self.low_since = None
            if self.status['state'] == 'lost':
                self._set(state='localized', message='위치 다시 맞음')
            else:
                self._publish_status()
            return
        self.low_since = self.low_since or now
        if now - self.low_since < self.lost_sec:
            self._publish_status()
            return
        if self.status['state'] != 'lost':
            self.get_logger().warn(f'위치 잃음: 추적 점수 {track:.2f} < {self.lost_score} ({self.lost_sec:.0f}초)')
            self._set(state='lost', message=f'스캔이 지도와 안 맞음 (점수 {track:.2f}) — 위치 다시 찾는 중')
        if now - self.last_try >= self.retry_sec:
            self._start('recover')

    def _track_score(self) -> float | None:
        """현재 추정 위치(map→base TF, 스캔 시각)에서 스캔이 지도와 얼마나 맞는지 — 검색과 같은 최종 점수
        (우도장 평균 × 빔 일관성). 우도장만 쓰면 엉뚱한 곳도 0.2~0.37 이 나와 정상(0.5~0.75)과 가깝다 (시뮬레이션)."""
        with self.lock:
            scan, m, grid = self.scan_msg, self.map, self.grid
        try:
            # 기다리지 않는다 (timeout 없음): 단일 스레드 executor 라 기다리는 동안 TF 콜백도 멈춘다
            t = self.tf.lookup_transform(self.global_frame, self.base_frame, Time.from_msg(scan.header.stamp))
        except Exception:  # noqa: BLE001 — AMCL 이 아직 map→odom 을 안 냄 / 스캔 시각 TF 가 아직 없음
            return None
        x, y = t.transform.translation.x, t.transform.translation.y
        yaw = yaw_of(t.transform.rotation)
        s = self._scan(scan)
        pts = s.points()
        if len(pts) < 20:
            return None
        lik = float(m.score(m.field(self.params.fine_sigma), np.array([[x, y, yaw]]), pts)[0])
        return rank(m, grid, s, [Candidate(x, y, yaw, lik)])[0].score

    # ---------- 검색 ----------
    def _scan(self, msg: LaserScan) -> Scan:
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        return Scan(np.asarray(msg.ranges, np.float64), angles, self.sensor,
                    max(msg.range_min, 0.05), min(msg.range_max, self.max_range))

    def _start(self, why: str) -> tuple[bool, str]:
        with self.lock:
            if self.busy:
                return False, '이미 찾는 중'
            if self.map is None or self.scan_msg is None or self.sensor is None:
                return False, '지도·라이다·TF 가 아직 없음'
            self.busy = True
        self.last_try = time.time()
        if why != 'recover':
            self._set(state='searching', message='위치 찾는 중')
        threading.Thread(target=self._search, args=(why,), daemon=True).start()
        return True, '위치 찾기 시작'

    def _search(self, why: str):
        try:
            with self.lock:
                m, grid, scan_msg = self.map, self.grid, self.scan_msg
            scan = self._scan(scan_msg)
            pts = scan.points()
            t0 = time.time()
            if why == 'start':
                last = self._load_pose()
                if last is not None:
                    c = local_search(m, pts, *last, radius=self.last_radius, angle_deg=self.last_angle,
                                     p=self.params)
                    c = rank(m, grid, scan, [c])[0]
                    if c.score >= self.accept_score:
                        self._send(c, scan_msg, 'last_pose', f'마지막 위치에서 시작 (점수 {c.score:.2f})')
                        return
                    self.get_logger().info(f'마지막 위치와 스캔이 안 맞음 (점수 {c.score:.2f}) → 지도 전체 검색')
            cands = rank(m, grid, scan, global_search(m, pts, self.params))
            dec = decide(cands, self.accept_score, self.margin)
            self.get_logger().info(f'전체 검색 {time.time() - t0:.2f}s: {dec.state} — {dec.message}, '
                                   f'후보 {[c.as_dict() for c in cands[:3]]}')
            top = [c.as_dict() for c in cands[:5]]
            if dec.state == 'localized':
                self._send(dec.best, scan_msg, 'recovered' if why == 'recover' else 'global', dec.message, top)
            elif why == 'recover':
                self.status['candidates'] = top
                self._set(state='lost', message=f'위치 잃음 — {dec.message}. 잠시 뒤 다시 시도')
            else:
                self._set(state=dec.state, method='global', score=round(dec.best.score, 3) if dec.best else None,
                          candidates=top, pose=None,
                          message=dec.message + ' — 후보를 고르거나 직접 지정하세요')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'위치 찾기 오류: {type(e).__name__}: {e}')
            self._set(state='failed', message=f'오류: {e}')
        finally:
            with self.lock:
                self.busy = False

    def _send(self, c: Candidate, scan_msg: LaserScan, method: str, message: str, cands=None):
        """AMCL 에 초기 위치 전송. stamp = 스캔 시각 → AMCL 이 그 뒤 움직인 만큼(odom)을 보정한다."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = scan_msg.header.stamp
        msg.pose.pose.position.x, msg.pose.pose.position.y = c.x, c.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(c.yaw / 2), math.cos(c.yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.1 ** 2              # x, y 표준편차 10cm
        cov[35] = math.radians(5.0) ** 2        # yaw 표준편차 5°
        msg.pose.covariance = cov
        self.sent_at = time.time()
        self.grace_until = self.sent_at + 3.0
        self.pub_init.publish(msg)
        self.low_since = None
        self._save_pose(c.x, c.y, c.yaw)
        self.get_logger().info(f'초기 위치 전송 ({method}): x={c.x:.3f} y={c.y:.3f} '
                               f'yaw={math.degrees(c.yaw):.1f}° 점수 {c.score:.2f}')
        self._set(state='localized', method=method, score=round(c.score, 3), pose=c.as_dict(),
                  candidates=cands or [], message=message)

    # ---------- 저장·상태 ----------
    def _load_pose(self):
        if not self.pose_file or not os.path.isfile(self.pose_file):
            return None
        try:
            with open(self.pose_file) as f:
                d = json.load(f)
            return float(d['x']), float(d['y']), float(d['yaw'])
        except (OSError, ValueError, KeyError) as e:
            self.get_logger().warn(f'마지막 위치 파일 읽기 실패 {self.pose_file}: {e}')
            return None

    def _save_pose(self, x, y, yaw):
        if not self.pose_file:
            return
        self.last_saved = time.time()
        try:
            tmp = self.pose_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 5), 'time': time.time()}, f)
            os.replace(tmp, self.pose_file)
        except OSError as e:
            self.get_logger().warn(f'마지막 위치 저장 실패: {e}')

    def _set(self, **kw):
        self.status.update(kw)
        self.status['time'] = time.time()
        self._publish_status()

    def _publish_status(self):
        self.pub_status.publish(String(data=json.dumps(self.status, ensure_ascii=False)))


def main():
    rclpy.init()
    node = Relocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
