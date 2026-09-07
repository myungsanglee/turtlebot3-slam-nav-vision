# =============================================================================
# vision.launch.py — Vision AI 노드 (RF-DETR 검출 + 정렬 depth 거리) 실행
#
# [실행] 서버 컨테이너에서 (colcon build 후)
#   ros2 launch my_vision vision.launch.py
#   ros2 launch my_vision vision.launch.py threshold:=0.4 model:=large
#   ros2 launch my_vision vision.launch.py backend:=tensorrt          # 첫 실행 시 엔진 빌드 (fp16)
#   ros2 launch my_vision vision.launch.py backend:=tensorrt trt_precision:=int8 trt_calib_dir:=/overlay_ws/models/calib
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
        DeclareLaunchArgument('backend', default_value='torch', choices=['torch', 'tensorrt'],
                              description='tensorrt: 엔진이 없으면 첫 실행 때 이 컨테이너에서 빌드(수 분)'),
        DeclareLaunchArgument('trt_precision', default_value='fp16', choices=['fp32', 'fp16', 'int8'],
                              description='int8 은 trt_calib_dir(캘리브레이션 이미지) 필요'),
        DeclareLaunchArgument('trt_opt_level', default_value='3', description='빌더 최적화 레벨 0~5'),
        DeclareLaunchArgument('trt_calib_dir', default_value='',
                              description='INT8 캘리브레이션 이미지 디렉터리 (camera_viewer --save-dir 로 수집)'),
        DeclareLaunchArgument('weights_dir', default_value='/overlay_ws/models',
                              description='가중치 캐시 디렉터리 (호스트 remote_pc/models)'),
    ]
    detector = Node(
        package='my_vision', executable='detector_node', name='detector', output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'threshold': LaunchConfiguration('threshold'),
            'weights_dir': LaunchConfiguration('weights_dir'),
            'backend': LaunchConfiguration('backend'),
            'trt_precision': LaunchConfiguration('trt_precision'),
            'trt_opt_level': LaunchConfiguration('trt_opt_level'),
            'trt_calib_dir': LaunchConfiguration('trt_calib_dir'),
        }],
    )
    return LaunchDescription([*declares, detector])
