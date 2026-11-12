# =============================================================================
# full_bringup.launch.py — 로봇 전체 브링업 (turtlebot3_bringup + RealSense)
#
# 두 셸에서 따로 띄우던 bringup 과 카메라를 한 번에 실행한다.
# turtlebot3_bringup 은 수정하지 않고 include 로 재사용 (프로젝트 원칙).
#
# [카메라 드라이버] camera_driver 인자로 선택
#   custom     (기본) 자체 pyrealsense2 노드 — rs_camera.launch.py
#                    /camera/color/compressed, /camera/depth/compressed(정렬), camera_info
#   realsense2        공식 realsense2_camera — realsense.launch.py
#                    /camera/camera/... (Pi4 에서 RGB 시작이 간헐 실패, 대안으로만 유지)
#
# [실행] Pi 에서:
#   ros2 launch realsense_bringup full_bringup.launch.py                       # 로봇+카메라(custom)
#   ros2 launch realsense_bringup full_bringup.launch.py camera:=false         # 로봇만
#   ros2 launch realsense_bringup full_bringup.launch.py camera_driver:=realsense2
#
# [주의] 하나로 묶이면 Ctrl+C 에 둘 다 종료된다. 카메라만 재시작해야 하는 상황에선
#   bringup 까지 재시작되어 오도메트리 원점이 리셋되므로 개별 실행이 유리할 수 있다.
#   (custom 드라이버는 스스로 복구하므로 이런 상황이 드물다)
#
# [시리얼 포트] OpenCR·LDS 를 /dev/serial/by-id/... (장치 고유 이름) 로 연다. USB 가 리셋돼 ttyACM1·ttyUSB1
#   로 다시 잡혀도 bringup 재시작만으로 복구된다 (troubleshooting 2026-11-12 "저전압 → USB 전체 재연결").
#   OpenCR 은 robot.launch.py 의 usb_port 인자, LDS 는 lidar_pkg_dir 인자를 launch/lds/ 로 돌려 넘긴다
#   (turtlebot3_bringup 수정 없음). by-id 가 없으면 기존 이름(/dev/ttyACM0·/dev/ttyUSB0)으로.
# =============================================================================
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def by_id(pattern, fallback):
    found = sorted(glob.glob(os.path.join('/dev/serial/by-id', pattern)))
    return found[0] if found else fallback


def generate_launch_description():
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    rs_bringup_dir = get_package_share_directory('realsense_bringup')

    camera = LaunchConfiguration('camera')
    driver = LaunchConfiguration('camera_driver')

    declares = [
        DeclareLaunchArgument('camera', default_value='true',
                              description='RealSense 카메라 포함 여부'),
        DeclareLaunchArgument('camera_driver', default_value='custom',
                              choices=['custom', 'realsense2'],
                              description='custom=자체 pyrealsense2 노드(기본) / realsense2=공식 노드'),
        DeclareLaunchArgument('opencr_port', default_value=by_id('*OpenCR*', '/dev/ttyACM0'),
                              description='OpenCR 시리얼 (기본: by-id 고정 이름)'),
        DeclareLaunchArgument('lds_port', default_value=by_id('*CP210*', '/dev/ttyUSB0'),
                              description='LDS-01 시리얼 (기본: by-id 고정 이름, LDS_MODEL=LDS-01 일 때만 적용)'),
    ]

    # LDS-01 일 때만 라이다 런치를 고정 포트 대체본(launch/lds/)으로 — 다른 모델은 ROBOTIS 기본 그대로
    tb3_args = {'usb_port': LaunchConfiguration('opencr_port')}
    if os.environ.get('LDS_MODEL', 'LDS-01') == 'LDS-01':
        tb3_args['lidar_pkg_dir'] = os.path.join(rs_bringup_dir, 'launch', 'lds')

    # 로봇 기본 (모터·오도메트리·라이다·robot_state_publisher) — ROBOTIS 그대로
    tb3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py')),
        launch_arguments=tb3_args.items(),
    )

    def use(name):
        return IfCondition(PythonExpression(["'", camera, "' == 'true' and '", driver, "' == '", name, "'"]))

    rs_custom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_bringup_dir, 'launch', 'rs_camera.launch.py')),
        condition=use('custom'),
    )
    rs_official = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_bringup_dir, 'launch', 'realsense.launch.py')),
        condition=use('realsense2'),
    )

    return LaunchDescription([*declares, tb3_bringup, rs_custom, rs_official])
