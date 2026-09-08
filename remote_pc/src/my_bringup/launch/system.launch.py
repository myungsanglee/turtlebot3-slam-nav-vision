# =============================================================================
# system.launch.py — Remote PC 통합 실행: SLAM(또는 저장 지도+AMCL) + Nav2 + Vision AI + RViz
#
# 각 패키지 런치를 조합만 한다 (설정은 각 패키지에 그대로):
#   my_navigation/navigation.launch.py  → (use_slam) my_slam 포함, Nav2 스택   [RViz 는 여기서 끔]
#   my_vision/vision.launch.py          → RF-DETR 검출, target_frame=map 으로 지도 좌표 마커
#   rviz2 + rviz/system.rviz            → 지도·코스트맵·경로·실측 footprint·로봇 모델·카메라 영상·검출 마커
#
# [전제] Pi 에서 `ros2 launch realsense_bringup full_bringup.launch.py` (bringup + 카메라)
# [실행] 컨테이너에서 (RViz 는 서버 물리 세션에 뜨므로 DISPLAY 필요)
#   export DISPLAY=:0
#   ros2 launch my_bringup system.launch.py                                   # SLAM 하며 주행 + Vision
#   ros2 launch my_bringup system.launch.py use_slam:=false map:=/overlay_ws/maps/my_map.yaml
#   ros2 launch my_bringup system.launch.py backend:=tensorrt threshold:=0.3   # Vision 옵션은 그대로 통과
#   ros2 launch my_bringup system.launch.py vision:=false rviz:=false          # SLAM+Nav2 만
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('my_navigation')
    vision_dir = get_package_share_directory('my_vision')
    rviz_config = os.path.join(get_package_share_directory('my_bringup'), 'rviz', 'system.rviz')

    declares = [
        DeclareLaunchArgument('use_slam', default_value='true',
                              description='true: SLAM 하며 주행 / false: 저장 지도+AMCL (map 필수)'),
        DeclareLaunchArgument('map', default_value='', description='저장 지도 yaml (use_slam:=false)'),
        DeclareLaunchArgument('vision', default_value='true', description='Vision AI 노드 포함'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행 (DISPLAY 필요)'),
        DeclareLaunchArgument('target_frame', default_value='map',
                              description='검출 물체 좌표/마커 프레임 (map: 지도 위 절대 위치)'),
        # Vision 런치로 그대로 통과 — 비우면 my_vision 의 YAML 값 사용
        DeclareLaunchArgument('backend', default_value='', description='torch | tensorrt (비우면 YAML)'),
        DeclareLaunchArgument('threshold', default_value='', description='검출 임계값 (비우면 YAML)'),
    ]

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_slam': LaunchConfiguration('use_slam'),
            'map': LaunchConfiguration('map'),
            'use_rviz': 'false',            # RViz 는 통합 설정(system.rviz) 하나만
        }.items(),
    )
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision_dir, 'launch', 'vision.launch.py')),
        launch_arguments={
            'target_frame': LaunchConfiguration('target_frame'),
            'backend': LaunchConfiguration('backend'),
            'threshold': LaunchConfiguration('threshold'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('vision')),
    )
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', rviz_config], condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([*declares, navigation, vision, rviz])
