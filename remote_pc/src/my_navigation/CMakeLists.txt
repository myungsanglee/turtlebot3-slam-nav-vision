cmake_minimum_required(VERSION 3.8)
project(my_navigation)

find_package(ament_cmake REQUIRED)

# 소스 코드 없이 설정/런치/RViz 파일만 install 하는 패키지
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
