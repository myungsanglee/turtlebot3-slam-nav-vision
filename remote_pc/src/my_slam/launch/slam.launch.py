# =============================================================================
# slam.launch.py — slam_toolbox(online async) + RViz2
#
# 사용법 (Remote PC 컨테이너 안):
#   ros2 launch my_slam slam.launch.py                # SLAM + RViz
#   ros2 launch my_slam slam.launch.py use_rviz:=false  # SLAM 만
#   ros2 launch my_slam slam.launch.py slam_params_file:=<다른 yaml>
#
# 전제: Pi 에서 turtlebot3_bringup 이 떠 있고 /scan, /odom, /tf 가 보이는 상태.
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_slam')

    default_params_file = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params_file,
        description='slam_toolbox 파라미터 yaml 경로')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='RViz2 실행 여부')

    # slam_toolbox: /scan + TF(odom) 를 받아 /map 과 map→odom TF 를 publish
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    # RViz2: 지도/스캔/TF 시각화 (QoS 설정은 slam.rviz 에 포함)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_slam_params_file,
        declare_use_rviz,
        slam_toolbox_node,
        rviz_node,
    ])
