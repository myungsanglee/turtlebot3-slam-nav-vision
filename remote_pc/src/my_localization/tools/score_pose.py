#!/usr/bin/env python3
"""지금 라이다 스캔이 지도 위 주어진 자세(들)에서 얼마나 맞는지 점수 (진단용 — 로봇을 움직이지 않음).

  python3 score_pose.py x y yaw_deg [x y yaw_deg ...]
  (주행 모드 실행 중: /map·/scan·TF 사용. 출력: 우도장·빔 일관성·최종 점수, 지금 map→base_footprint)
"""
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from my_localization.relocalizer_node import yaw_of
from my_localization.scan_matcher import Candidate, MapModel, Scan, consistency


def main():
    vals = list(map(float, sys.argv[1:]))
    poses = [(vals[i], vals[i + 1], math.radians(vals[i + 2])) for i in range(0, len(vals) - 2, 3)]
    rclpy.init()
    n = rclpy.create_node('score_pose')
    tf = Buffer()
    TransformListener(tf, n)
    got = {}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(OccupancyGrid, '/map', lambda m: got.setdefault('map', m), latched)
    n.create_subscription(LaserScan, '/scan', lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
    while 'map' not in got or 'scan' not in got:
        rclpy.spin_once(n, timeout_sec=0.5)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.1)
    mm, sc = got['map'], got['scan']
    grid = np.asarray(mm.data, np.int8).reshape(mm.info.height, mm.info.width)
    m = MapModel(grid, mm.info.resolution, (mm.info.origin.position.x, mm.info.origin.position.y))
    t = tf.lookup_transform('base_footprint', sc.header.frame_id, Time())
    sensor = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
    angles = sc.angle_min + np.arange(len(sc.ranges)) * sc.angle_increment
    s = Scan(np.asarray(sc.ranges), angles, sensor, max(sc.range_min, 0.05), min(sc.range_max, 3.5))
    try:
        cur = tf.lookup_transform('map', 'base_footprint', Time())
        cx, cy, cyaw = cur.transform.translation.x, cur.transform.translation.y, yaw_of(cur.transform.rotation)
        print(f'지금 map→base_footprint: ({cx:.3f}, {cy:.3f}) {math.degrees(cyaw):.1f}°')
        poses.append((cx, cy, cyaw))
    except Exception as e:  # noqa: BLE001
        print('map→base_footprint 없음:', e)
    f = m.field(0.05)
    for x, y, yaw in poses:
        lik = float(m.score(f, np.array([[x, y, yaw]]), s.points())[0])
        con = consistency(m, grid, s, Candidate(x, y, yaw, lik))
        print(f'({x:.3f}, {y:.3f}) {math.degrees(yaw):7.1f}°  우도 {lik:.2f}  일관성 {con:.2f}  최종 {lik * con:.2f}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
