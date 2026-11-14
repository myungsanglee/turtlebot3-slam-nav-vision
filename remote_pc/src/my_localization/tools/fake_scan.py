#!/usr/bin/env python3
"""로봇 없이 자동 위치 찾기 시험: 저장 지도 위 '진짜 위치'에서 본 가상 라이다 스캔을 발행한다.

  ROS_DOMAIN_ID=77 python3 fake_scan.py /overlay_ws/maps/<지도>.yaml x y yaw_deg
  (다른 도메인 ID 로 돌려 실제 로봇 토픽과 섞이지 않게 한다)

발행: /scan (5Hz, 360빔, 잡음·반사 없음 섞음), TF odom→base_footprint(제자리), base_footprint→base_scan(실측 -0.10 m).
진짜 위치는 실행 중에 바꿀 수 있다 — '로봇을 들어 옮기기' 흉내:
  echo "1.2 0.8 90" > /tmp/fake_scan_pose
"""
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from my_localization.scan_matcher import MapModel, raycast

POSE_FILE = '/tmp/fake_scan_pose'
SENSOR_X = -0.10


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    data = open(pgm, 'rb').read().split(b'\n', 3)
    w, h = map(int, data[1].split())
    img = np.frombuffer(data[3], np.uint8)[: w * h].reshape(h, w)[::-1]   # 이미지 위 = 지도 +y
    grid = np.where(img < 50, 100, np.where(img > 250, 0, -1)).astype(np.int8)
    return grid, meta['resolution'], meta['origin'][:2]


class FakeScan(Node):
    def __init__(self, yaml_path, pose):
        super().__init__('fake_scan')
        self.grid, res, origin = load_map(yaml_path)
        self.m = MapModel(self.grid, res, origin)
        self.pose = pose
        self.rng = np.random.default_rng(0)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.tfb = TransformBroadcaster(self)
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = 'base_footprint', 'base_scan'
        st.transform.translation.x = SENSOR_X
        st.transform.rotation.w = 1.0
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform(st)
        self.create_timer(0.2, self._tick)

    def _tick(self):
        if os.path.isfile(POSE_FILE):
            try:
                x, y, yd = map(float, open(POSE_FILE).read().split())
                if (x, y, math.radians(yd)) != self.pose:
                    self.pose = (x, y, math.radians(yd))
                    self.get_logger().info(f'진짜 위치 변경: {x} {y} {yd}°')
            except ValueError:
                pass
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_footprint'
        t.transform.rotation.w = 1.0
        self.tfb.sendTransform(t)

        x, y, yaw = self.pose
        angles = np.radians(np.arange(360.0))
        r = raycast(self.m, self.grid, x + math.cos(yaw) * SENSOR_X, y + math.sin(yaw) * SENSOR_X, yaw, angles)
        r = r + self.rng.normal(0, 0.015, r.shape)
        r = np.where(self.rng.random(r.shape) < 0.1, 0.0, r)        # LDS 는 반사 없음을 0 으로 보냄
        r = np.where(np.isfinite(r), r, 0.0)
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = now, 'base_scan'
        msg.angle_min, msg.angle_max, msg.angle_increment = 0.0, math.radians(359), math.radians(1)
        msg.range_min, msg.range_max = 0.12, 3.5
        msg.ranges = r.astype(np.float32).tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    x, y, yd = map(float, sys.argv[2:5])
    rclpy.spin(FakeScan(sys.argv[1], (x, y, math.radians(yd))))


if __name__ == '__main__':
    main()
