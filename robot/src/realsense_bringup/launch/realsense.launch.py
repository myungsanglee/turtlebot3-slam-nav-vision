# =============================================================================
# realsense.launch.py — RealSense D435i 브링업 (Raspberry Pi 에서 실행)
#
# realsense2_camera 의 표준 런치(rs_launch.py)를 재사용하되, 이 로봇/환경에 맞춘
# 옵션만 덮어써서 띄운다.
#
# [Pi USB 이슈 대응] pyrealsense2 최소 스트리밍은 되는데 ROS 노드는
#   UVC 컨트롤 타임아웃(xioctl ... Connection timed out)으로 실패하는 경우가 있다.
#   원인은 ROS 노드가 최소 스트리밍보다 많은 장치 제어(XU 컨트롤)를 하기 때문:
#     - initial_reset  : 시작 시 USB 리셋 → 직후 제어 접근이 불안정할 수 있음
#     - align_depth    : depth↔color 정렬용 extrinsics 를 XU 로 읽음 (실패 지점)
#   그래서 기본값을 "가볍게"(둘 다 off) 잡아 기본 스트리밍부터 되게 하고,
#   안정화되면 인자로 하나씩 켜서 범인을 좁힌다.
#
# [해상도] Pi USB/원격 대역폭 고려해 낮게. 형식 "WIDTHxHEIGHTxFPS".
#
# [실행 예]
#   ros2 launch realsense_bringup realsense.launch.py                  # 가벼운 기본
#   ros2 launch realsense_bringup realsense.launch.py align_depth:=true
#   ros2 launch realsense_bringup realsense.launch.py initial_reset:=true
#   ros2 launch realsense_bringup realsense.launch.py color_profile:=424x240x15
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rs_dir = get_package_share_directory('realsense2_camera')
    rs_launch = os.path.join(rs_dir, 'launch', 'rs_launch.py')

    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')
    align_depth = LaunchConfiguration('align_depth')
    initial_reset = LaunchConfiguration('initial_reset')
    pointcloud = LaunchConfiguration('pointcloud')
    imu = LaunchConfiguration('imu')

    declares = [
        DeclareLaunchArgument('color_profile', default_value='640x480x15',
                              description='RGB 해상도 WxHxFPS'),
        DeclareLaunchArgument('depth_profile', default_value='640x480x15',
                              description='Depth 해상도 WxHxFPS'),
        # align_depth: 비전 파이프라인에서 depth-color 좌표 매칭에 필요 → 기본 on
        DeclareLaunchArgument('align_depth', default_value='true',
                              description='depth↔color 정렬. 비전에서 필요 → 기본 on'),
        # initial_reset: 이 로봇의 D435i 가 재부팅/전원 변동 후 자주 안 떠서
        #   시작 시 USB 리셋을 기본 on (전원 보강 후 off 재검토 — 2026-09-01 항목)
        DeclareLaunchArgument('initial_reset', default_value='true',
                              description='시작 시 USB 리셋. 카메라가 자주 안 떠서 기본 on'),
        DeclareLaunchArgument('pointcloud', default_value='false',
                              description='pointcloud publish (부담 큼)'),
        DeclareLaunchArgument('imu', default_value='false',
                              description='카메라 내장 IMU(gyro/accel) publish'),
    ]

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.color_profile': color_profile,
            'depth_module.depth_profile': depth_profile,
            'align_depth.enable': align_depth,
            'pointcloud.enable': pointcloud,
            'enable_gyro': imu,
            'enable_accel': imu,
            'initial_reset': initial_reset,
        }.items(),
    )

    return LaunchDescription([*declares, realsense])
