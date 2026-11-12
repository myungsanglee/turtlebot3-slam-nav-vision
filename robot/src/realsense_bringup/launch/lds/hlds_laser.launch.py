# =============================================================================
# lds/hlds_laser.launch.py — LDS-01 드라이버를 '고정 포트 이름'으로 띄우는 대체 런치
#
# 왜: turtlebot3_bringup/robot.launch.py 는 라이다 포트를 '/dev/ttyUSB0' 으로 고정해 넘긴다.
#   Pi 저전압으로 USB 가 리셋되면 장치가 ttyUSB1 로 다시 잡혀(옛 이름은 죽은 프로세스가 쥐고 있음)
#   bringup 재시작만으로는 복구되지 않는다 (troubleshooting 2026-11-12 "저전압 → USB 전체 재연결").
#   /dev/serial/by-id/... 는 장치 고유 이름이라 몇 번째로 잡히든 같은 경로를 가리킨다.
#
# 어떻게 (turtlebot3_bringup 은 수정하지 않는다 — 프로젝트 원칙):
#   robot.launch.py 가 공식 인자로 열어 둔 `lidar_pkg_dir`(라이다 런치 폴더) 를 이 폴더로 바꾸면,
#   robot.launch.py 는 [lidar_pkg_dir]/hlds_laser.launch.py — 즉 이 파일을 include 한다.
#   원본(hls_lfcd_lds_driver)과 같은 노드를 띄우되 포트만 `lds_port`(full_bringup 인자) 를 쓴다.
#   robot.launch.py 가 넘기는 `port`(/dev/ttyUSB0) 는 lds_port 가 없을 때의 기본값으로만 쓴다.
# =============================================================================
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('lds_port', default=LaunchConfiguration('port', default='/dev/ttyUSB0'))
    frame_id = LaunchConfiguration('frame_id', default='laser')
    return LaunchDescription([
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': port, 'frame_id': frame_id}],
            output='screen'),
    ])
