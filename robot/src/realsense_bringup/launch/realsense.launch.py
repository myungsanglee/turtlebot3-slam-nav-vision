# =============================================================================
# realsense.launch.py — RealSense D435i 브링업 (Raspberry Pi 에서 실행)
#
# realsense2_camera 의 표준 런치(rs_launch.py)를 그대로 재사용하되, 이 로봇에
# 필요한 옵션만 인자로 덮어써서 띄운다. (버전별 파라미터 이름 차이를 표준 런치가
# 흡수해주므로 안정적)
#
# [1차 목표] color + depth 가 /camera/* 로 안정적으로 뜨는 것 확인.
#   - 해상도/fps 프로파일은 일단 카메라 기본값 사용(파라미터 형식 리스크 회피).
#   - 대역폭(원격 전송) 최적화·해상도 다운·압축 전송은 동작 확인 후 2차로.
#
# [실행] Pi 에서 (realsense2_camera 설치돼 있어야 함):
#   ros2 launch realsense_bringup realsense.launch.py            # 빌드/소싱 시
#   ros2 launch <이 파일 경로>/realsense.launch.py                # 빌드 없이 직접
#
# [옵션 예]
#   ros2 launch realsense_bringup realsense.launch.py pointcloud:=true
#   ros2 launch realsense_bringup realsense.launch.py imu:=true
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

    pointcloud = LaunchConfiguration('pointcloud')
    imu = LaunchConfiguration('imu')

    declare_pointcloud = DeclareLaunchArgument(
        'pointcloud', default_value='false',
        description='pointcloud publish 여부 (연산·대역폭 부담 큼 → 기본 off)')
    declare_imu = DeclareLaunchArgument(
        'imu', default_value='false',
        description='카메라 내장 IMU(gyro/accel) publish 여부. 로봇 IMU 와 별개 '
                    '→ 기본 off (필요 시 켬)')

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            # 스트림
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',      # depth 를 color 에 정렬(비전용 정합 depth)
            'pointcloud.enable': pointcloud,
            'enable_gyro': imu,
            'enable_accel': imu,
            'unite_imu_method': '2',           # imu:=true 일 때 gyro/accel 을 하나로 결합
            # 안정화
            'initial_reset': 'true',           # 시작 시 카메라 리셋(프레임 안 옴 현상 예방)
        }.items(),
    )

    return LaunchDescription([
        declare_pointcloud,
        declare_imu,
        realsense,
    ])
