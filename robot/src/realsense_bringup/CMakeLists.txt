cmake_minimum_required(VERSION 3.8)
project(realsense_bringup)

find_package(ament_cmake REQUIRED)

# 런치 파일 install
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 자체 카메라 노드 (Python 스크립트) — `ros2 run realsense_bringup rs_camera_node.py`
install(
  PROGRAMS scripts/rs_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
