# =============================================================================
# realsense.launch.py — RealSense D435i 브링업 (Raspberry Pi 에서 실행)
#
# realsense2_camera 의 표준 런치(rs_launch.py)를 재사용하되, 이 로봇/환경에 맞춘
# 옵션만 덮어써서 띄운다.
#
# [해상도를 낮게 잡는 이유] 라즈베리파이의 USB 링크에서 D435i 기본 해상도
#   (RGB 1280x720x30 + depth 848x480x30)는 대역폭이 커서 UVC 컨트롤 타임아웃
#   (xioctl ... Connection timed out)으로 스트림 시작이 실패하기 쉽다. 낮은
#   해상도/fps 로 USB 부담을 줄인다. 또한 원격(Tailscale) 전송 대역폭도 절약.
#   → 안정화되면 값을 점차 올리며 한계를 찾는다.
#
# [지원 형식] 프로파일은 "WIDTHxHEIGHTxFPS" (예: 640x480x15). D435i depth 는
#   848x480 / 640x480 / 640x360 / 480x270 / 424x240 등 지원.
#
# [실행] Pi 에서 (realsense2_camera 설치 필요):
#   ros2 launch realsense_bringup realsense.launch.py
#   ros2 launch <이 파일 경로>/realsense.launch.py           # 빌드 없이 직접
#
# [옵션]
#   color_profile:=424x240x15   depth_profile:=480x270x15   (더 낮추기)
#   pointcloud:=true   imu:=true
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

    # 낮은 기본 해상도 (USB 안정성 우선). 안정화 후 상향.
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')
    pointcloud = LaunchConfiguration('pointcloud')
    imu = LaunchConfiguration('imu')

    declares = [
        DeclareLaunchArgument('color_profile', default_value='640x480x15',
                              description='RGB 해상도 WxHxFPS'),
        DeclareLaunchArgument('depth_profile', default_value='640x480x15',
                              description='Depth 해상도 WxHxFPS'),
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
            'align_depth.enable': 'true',       # depth 를 color 에 정렬
            'pointcloud.enable': pointcloud,
            'enable_gyro': imu,
            'enable_accel': imu,
            'initial_reset': 'true',            # 시작 시 카메라 리셋(스턱 예방)
        }.items(),
    )

    return LaunchDescription([*declares, realsense])
