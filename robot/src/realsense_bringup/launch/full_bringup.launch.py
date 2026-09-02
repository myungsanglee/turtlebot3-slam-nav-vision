# =============================================================================
# full_bringup.launch.py — 로봇 전체 브링업 (turtlebot3_bringup + RealSense)
#
# 두 셸에서 따로 띄우던 bringup 과 카메라를 한 번에 실행한다.
# turtlebot3_bringup 은 수정하지 않고 include 로 재사용 (프로젝트 원칙).
#
# [실행] Pi 에서:
#   ros2 launch realsense_bringup full_bringup.launch.py                # 로봇+카메라
#   ros2 launch realsense_bringup full_bringup.launch.py camera:=false # 로봇만
#   ros2 launch realsense_bringup full_bringup.launch.py initial_reset:=true
#
# [주의] 하나로 묶이면 Ctrl+C 에 둘 다 종료된다. 카메라만 재시작해야 하는 상황
#   (USB 꼬임 복구 등)에선 bringup 까지 재시작되어 오도메트리 원점이 리셋되므로,
#   카메라가 불안정한 날엔 기존처럼 개별 실행이 운영상 유리할 수 있다.
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    rs_bringup_dir = get_package_share_directory('realsense_bringup')

    camera = LaunchConfiguration('camera')
    initial_reset = LaunchConfiguration('initial_reset')

    declares = [
        DeclareLaunchArgument('camera', default_value='true',
                              description='RealSense 카메라 포함 여부'),
        DeclareLaunchArgument('initial_reset', default_value='true',
                              description='카메라 시작 시 USB 리셋 (자주 안 떠서 기본 on)'),
    ]

    # 로봇 기본 (모터·오도메트리·라이다·robot_state_publisher) — ROBOTIS 그대로
    tb3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py')),
    )

    # RealSense 카메라 (우리 런치 재사용, 옵션 전달)
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_bringup_dir, 'launch', 'realsense.launch.py')),
        launch_arguments={'initial_reset': initial_reset}.items(),
        condition=IfCondition(camera),
    )

    return LaunchDescription([*declares, tb3_bringup, realsense])
