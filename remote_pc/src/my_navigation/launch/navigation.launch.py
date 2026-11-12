# =============================================================================
# navigation.launch.py — Nav2 스택 (+ 기본값으로 SLAM 동시 실행)
#
# [모드 1] SLAM 하면서 주행 (기본, 로봇청소기 시나리오):
#   ros2 launch my_navigation navigation.launch.py
#   → my_slam(slam_toolbox) + Nav2 + RViz. 지도를 만들면서 목표점 주행.
#
# [모드 2] 저장된 지도로 주행 (지도 완성 후):
#   ros2 launch my_navigation navigation.launch.py \
#       use_slam:=false map:=/overlay_ws/maps/my_map.yaml
#   → map_server + AMCL(localization) + Nav2 + RViz.
#
# 카메라 깊이 층 (회피 V1): my_vision depth_cloud_node 가 depth → /camera/points 를 만들어 costmap 의
#   camera_layer 에 넣는다. 끄기: camera_obstacles:=false (Nav2 는 camera_layer 입력 없이도 그대로 동작)
#
# 전제: Pi 에서 turtlebot3_bringup 이 떠 있고 /scan, /odom, /tf 가 보이는 상태.
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_slam_dir = get_package_share_directory('my_slam')

    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'nav.rviz')

    params_file = LaunchConfiguration('params_file')
    use_slam = LaunchConfiguration('use_slam')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    camera_obstacles = LaunchConfiguration('camera_obstacles')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Nav2 파라미터 yaml 경로')

    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='true: slam_toolbox 동시 실행(지도 만들며 주행). '
                    'false: 저장된 지도 사용(map 인자 필수)')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='저장된 지도 yaml 경로 (use_slam:=false 일 때만 사용)')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='RViz2 실행 여부')

    declare_camera_obstacles = DeclareLaunchArgument(
        'camera_obstacles',
        default_value='true',
        description='카메라 depth 점 구름을 장애물 지도에 넣기 (회피 V1)')

    # 카메라 깊이 층 입력: 정렬 depth → /camera/points (가벼운 CPU 노드, Vision AI 와 무관하게 Nav2 와 함께 뜬다)
    depth_cloud_node = Node(
        package='my_vision',
        executable='depth_cloud_node',
        name='depth_cloud',
        output='screen',
        condition=IfCondition(camera_obstacles),
    )

    # [모드 1] SLAM — my_slam 의 slam_toolbox 가 map→odom TF 와 /map 제공
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_slam_dir, 'launch', 'slam.launch.py')),
        launch_arguments={'use_rviz': 'false'}.items(),
        condition=IfCondition(use_slam),
    )

    # [모드 2] Localization — map_server(저장 지도) + AMCL 이 같은 역할 제공
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", map_yaml, "' != ''"])),
    )

    # Nav2 본체: planner / controller / behaviors / bt_navigator /
    #            waypoint_follower / velocity_smoother + lifecycle manager
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_slam,
        declare_map,
        declare_use_rviz,
        declare_camera_obstacles,
        depth_cloud_node,
        slam_launch,
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
