# =============================================================================
# rs_camera.launch.py — 자체 RealSense 노드(rs_camera_node.py) 브링업 (Pi 기본 드라이버)
#
# 공식 realsense2_camera 대신 pyrealsense2 기반 자체 노드를 띄운다. 이유와 설계는
# scripts/rs_camera_node.py 머리말과 docs/realsense_bringup.md 참고 — 요약하면
# Pi4+RSUSB 에서 공식 노드가 RGB 컨트롤 경로 불안정에 막히는 문제를, 최소 스트리밍
# + 캘리브레이션 캐시 + 프로세스 분리 감시로 해결한 것.
#
# [출력] /camera/color/compressed (jpeg), /camera/depth/compressed (16UC1 png, mm,
#        color 에 정렬), /camera/color/camera_info — 전부 sensor_data QoS(best effort)
#
# [실행 예] Pi 에서:
#   ros2 launch realsense_bringup rs_camera.launch.py
#   ros2 launch realsense_bringup rs_camera.launch.py fps:=15                  # 고속 (Pi 부하·대역폭 ↑)
#   ※ fps 는 카메라가 지원하는 값만 가능 — 640x480 에서 6/15/30/60 (5 는 없음)
#
# [respawn] 노드는 내부에서 캡처 프로세스를 감시·재시작하므로 스스로 죽는 일이 드물지만,
#   부모 프로세스 자체가 죽는 경우(예외 등)를 대비해 런치 수준 respawn 도 건다.
# =============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declares = [
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='6',
                              description='640x480 지원값 6/15/30/60. Pi 부하·대역폭 때문에 기본 6'),
        DeclareLaunchArgument('jpeg_quality', default_value='80',
                              description='color JPEG 품질 (0~100)'),
        DeclareLaunchArgument('png_compression', default_value='1',
                              description='depth PNG 압축 (0~9, 낮을수록 빠름·큼)'),
    ]

    node = Node(
        package='realsense_bringup',
        executable='rs_camera_node.py',
        name='rs_camera',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'png_compression': LaunchConfiguration('png_compression'),
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([*declares, node])
