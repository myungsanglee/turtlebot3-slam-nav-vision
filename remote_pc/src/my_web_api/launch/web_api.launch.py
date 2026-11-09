# =============================================================================
# web_api.launch.py — 웹 UI 백엔드 실행 (compose 의 web-api 서비스가 이걸 실행한다)
#   ros2 launch my_web_api web_api.launch.py                         # API 만 (config/web_api.yaml)
#   ros2 launch my_web_api web_api.launch.py static_dir:=/web/dist   # + React 빌드 결과 제공
#   ros2 launch my_web_api web_api.launch.py port:=8081              # 포트만 바꾸기
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    params = [LaunchConfiguration('params_file').perform(context)]
    overrides = {}
    static_dir = LaunchConfiguration('static_dir').perform(context)
    port = LaunchConfiguration('port').perform(context)
    if static_dir:
        overrides['static_dir'] = static_dir
    if port:
        overrides['port'] = int(port)
    if overrides:
        params.append(overrides)              # 뒤가 앞(YAML)을 덮어쓴다
    return [Node(package='my_web_api', executable='server', name='web_api',
                 output='screen', emulate_tty=True, parameters=params)]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('my_web_api'), 'config', 'web_api.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params, description='노드 파라미터 YAML'),
        DeclareLaunchArgument('static_dir', default_value='', description='React 빌드 폴더 (비우면 YAML 값)'),
        DeclareLaunchArgument('port', default_value='', description='HTTP 포트 (비우면 YAML 값)'),
        OpaqueFunction(function=_setup),
    ])
