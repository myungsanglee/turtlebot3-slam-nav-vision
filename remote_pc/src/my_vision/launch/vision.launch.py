# =============================================================================
# vision.launch.py — Vision AI 노드 (RF-DETR 검출 + 정렬 depth 거리) 실행
#
# 설정은 config/vision_params.yaml 이 단일 소스다. 이 파일을 고쳐 실행하면 그대로 적용된다.
#   ros2 launch my_vision vision.launch.py                                  # config/vision_params.yaml
#   ros2 launch my_vision vision.launch.py params_file:=/overlay_ws/my.yaml # 다른 설정 파일
#   ros2 launch my_vision vision.launch.py threshold:=0.3 backend:=tensorrt # 준 인자만 YAML 을 덮어씀
#   결과 확인: ros2 run my_vision camera_viewer --color-topic /vision/annotated/compressed
#             ros2 topic echo /vision/detections
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 런치 인자로 덮어쓸 수 있는 파라미터와 타입. 비워 두면(기본) YAML 값을 그대로 쓴다.
OVERRIDES = {
    'model': str, 'weights': str, 'threshold': float, 'backend': str,
    'trt_precision': str, 'trt_opt_level': int, 'trt_calib_dir': str,
}


def _setup(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    overrides = {}
    for name, cast in OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = cast(value)
    parameters = [params_file] + ([overrides] if overrides else [])   # 뒤가 앞을 덮어쓴다
    return [Node(package='my_vision', executable='detector_node', name='detector',
                 output='screen', parameters=parameters)]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('my_vision'), 'config', 'vision_params.yaml')
    declares = [DeclareLaunchArgument('params_file', default_value=default_params,
                                      description='노드 파라미터 YAML')]
    declares += [DeclareLaunchArgument(name, default_value='',
                                       description=f'{name} 를 YAML 대신 이 값으로 (비우면 YAML)')
                 for name in OVERRIDES]
    return LaunchDescription([*declares, OpaqueFunction(function=_setup)])
