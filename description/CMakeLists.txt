cmake_minimum_required(VERSION 3.8)
project(my_description)
find_package(ament_cmake REQUIRED)
# urdf/ 와 meshes/ 를 share 에 설치 → RViz 가 package://my_description/meshes/*.stl 을 찾는다
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME}
        PATTERN "my_turtleBot3_burger.stl" EXCLUDE)   # CAD 원본(59MB, git 제외)은 설치하지 않음
ament_package()
