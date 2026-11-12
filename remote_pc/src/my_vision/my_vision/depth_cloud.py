# =============================================================================
# depth_cloud.py — 정렬 depth 이미지 → 3D 점 구름 (PointCloud2) : 카메라 회피 V1 "깊이 층"
#
#   /camera/depth/compressed (PNG16, mm, color 정렬) + /camera/color/camera_info
#        → /camera/points (sensor_msgs/PointCloud2, camera_color_optical_frame)
#        → Nav2 costmap 의 camera_layer 가 높이로 걸러 장애물로 찍는다 (nav2_params.yaml)
#
# 왜 필요한가 (docs/vision_avoidance_design.md 1장): 라이다는 바닥 위 13.5cm 평면 한 장만 본다.
# 그보다 낮은 것(신발·상자·전선)과 위로 튀어나온 것(책상 상판·의자 좌판)은 카메라로만 보인다.
#
# 원리 — 핀홀 카메라 역투영: 픽셀 (u,v) 의 거리가 z 이면, 그 점의 카메라 좌표는
#     x = (u − cx) · z / fx,   y = (v − cy) · z / fy,   z = z
# (광학 좌표계: z 앞, x 오른쪽, y 아래. 카메라 좌표 → 로봇·지도 좌표 변환은 costmap 이 TF 로 한다)
#
# 왜 my_vision 의 별도 노드인가:
#   - depth_image_proc 은 raw Image 를 받는다 → compressed 를 풀어 다시 넣는 노드가 하나 더 필요.
#     PNG 디코딩 + 역투영은 numpy 몇 줄이라 직접 하는 쪽이 단순 (camera_io 의 디코더 재사용).
#   - 검출 노드(GPU, vision:=false 로 끌 수 있음)에 붙이지 않는다 — 회피(안전)가 AI 켜짐 여부에 묶이면 안 된다.
# =============================================================================
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField

from .camera_io import decode_depth_mm


class PixelRays:
    """솎은 픽셀 격자의 '거리 1m 일 때 x, y' 를 미리 계산해 둔다 (매 프레임은 곱셈만)."""

    def __init__(self, K, width, height, stride):
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        self.key = (tuple(K), width, height, stride)
        us = np.arange(stride // 2, width, stride, dtype=np.float32)
        vs = np.arange(stride // 2, height, stride, dtype=np.float32)
        self.u_idx = us.astype(np.intp)
        self.v_idx = vs.astype(np.intp)
        uu, vv = np.meshgrid(us, vs)
        self.rx = (uu - cx) / fx
        self.ry = (vv - cy) / fy


def depth_to_points(depth_mm, rays, min_range, max_range):
    """솎은 depth → (N, 3) float32 점 (광학 좌표계, m). 0(측정 없음)과 범위 밖은 버린다."""
    z = depth_mm[np.ix_(rays.v_idx, rays.u_idx)].astype(np.float32) * 0.001
    ok = (z >= min_range) & (z <= max_range)
    zs = z[ok]
    return np.stack([rays.rx[ok] * zs, rays.ry[ok] * zs, zs], axis=1).astype(np.float32, copy=False)


_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]


def make_cloud(points, header):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


class DepthCloudNode(Node):
    def __init__(self):
        super().__init__('depth_cloud')
        self.stride = int(self.declare_parameter('stride', 4).value)             # 4 → 160x120 ≈ 1.9만 점
        self.min_range = float(self.declare_parameter('min_range', 0.2).value)   # D435i 최소 측정 거리 근처
        self.max_range = float(self.declare_parameter('max_range', 3.0).value)   # 멀수록 오차↑, local costmap 3m 창
        depth_topic = self.declare_parameter('depth_topic', '/camera/depth/compressed').value
        info_topic = self.declare_parameter('info_topic', '/camera/color/camera_info').value
        out_topic = self.declare_parameter('points_topic', '/camera/points').value

        self.info = None
        self.rays = None
        self.count = 0
        self.points_sum = 0
        self.pub = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_timer(30.0, self.report)
        self.get_logger().info(f'{depth_topic} → {out_topic} (stride {self.stride}, '
                               f'{self.min_range}~{self.max_range} m)')

    def on_info(self, msg):
        self.info = msg

    def on_depth(self, msg):
        if self.info is None:
            return
        depth = decode_depth_mm(msg)
        if depth is None:
            return
        h, w = depth.shape[:2]
        key = (tuple(self.info.k), w, h, self.stride)
        if self.rays is None or self.rays.key != key:
            self.rays = PixelRays(list(self.info.k), w, h, self.stride)
        pts = depth_to_points(depth, self.rays, self.min_range, self.max_range)
        self.pub.publish(make_cloud(pts, msg.header))   # stamp·frame 은 depth 그대로 (costmap 이 그 시각의 TF 로 변환)
        self.count += 1
        self.points_sum += len(pts)

    def report(self):
        if self.count:
            self.get_logger().info(f'최근 30초: {self.count / 30:.1f} Hz, 평균 {self.points_sum // self.count} 점')
        else:
            self.get_logger().warn('최근 30초 depth 수신 없음 (카메라·브리지 확인)')
        self.count = self.points_sum = 0


def main():
    rclpy.init()
    node = DepthCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
