# =============================================================================
# vision.launch.py — Vision AI 노드 (RF-DETR 검출 + 정렬 depth 거리) 실행
#
# [실행] 서버 컨테이너에서 (colcon build 후)
#   ros2 launch my_vision vision.launch.py
#   ros2 launch my_vision vision.launch.py threshold:=0.4 model:=large
#   결과 확인: ros2 run my_vision camera_viewer --color-topic /vision/annotated/compressed
#             ros2 topic echo /vision/detections
# =============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declares = [
        DeclareLaunchArgument('model', default_value='medium',
                              choices=['nano', 'small', 'medium', 'large'],
                              description='RF-DETR 크기 (medium ≈ 구 base)'),
        DeclareLaunchArgument('threshold', default_value='0.5', description='검출 점수 임계값'),
        DeclareLaunchArgument('weights_dir', default_value='/overlay_ws/models',
                              description='가중치 캐시 디렉터리 (호스트 remote_pc/models)'),
    ]
    detector = Node(
        package='my_vision', executable='detector_node', name='detector', output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'threshold': LaunchConfiguration('threshold'),
            'weights_dir': LaunchConfiguration('weights_dir'),
        }],
    )
    return LaunchDescription([*declares, detector])
