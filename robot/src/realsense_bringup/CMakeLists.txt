cmake_minimum_required(VERSION 3.8)
project(realsense_bringup)

find_package(ament_cmake REQUIRED)

# 런치 파일만 install (Pi 에서 빌드하거나, 파일 경로로 직접 실행도 가능)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
