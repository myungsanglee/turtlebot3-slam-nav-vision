# =============================================================================
# TurtleBot3 Remote PC (ROS 2 Humble) - Dockerfile
#
# 베이스: OSRF 공식 ROS 2 이미지 (osrf/ros:humble-desktop)
# 그 위에 ROBOTIS 공식 humble 브랜치 소스(DynamixelSDK / turtlebot3_msgs /
# turtlebot3)를 빌드하고, SLAM(Cartographer/slam_toolbox)·Nav2를 추가.
#
# 호스트: Ubuntu 24.04 + NVIDIA RTX A6000 (드라이버 + NVIDIA Container Toolkit 설치 가정)
# 컨테이너 베이스는 22.04(Humble)이며, 호스트 OS와 무관하게 동작.
#
# [설치 방침]
#  - DynamixelSDK / turtlebot3_msgs / turtlebot3 : 전부 "소스(git clone)"로 통일.
#    -> apt(ros-humble-dynamixel-sdk 등)와 중복 설치하지 않는다(오버레이 충돌 방지).
#  - Gazebo : apt로 명시하지 않는다(실물 로봇 위주). turtlebot3 메타패키지가
#    요구하는 gazebo 관련 의존성은 rosdep install 이 자동으로 해결한다.
#
# [환경 source 방침]
#  - entrypoint.sh : `docker compose run` 으로 들어올 때 ROS 환경 source (시작 명령)
#  - /root/.bashrc : `docker compose exec ... bash` 로 들어올 때도 자동 source
#    (exec 는 entrypoint 를 거치지 않으므로, 대화형 셸마다 .bashrc 가 source 한다)
#  -> 두 방식(run/exec) 어느 쪽으로 들어와도 `ros2` 명령이 항상 동작.
#
# [빌드 컨텍스트 주의]
#  - compose의 build.context 가 tb3-remote-pc/ 이므로 COPY 경로는 그 루트 기준.
#    entrypoint.sh 는 docker/ 안에 있으므로 COPY ./docker/entrypoint.sh 로 가리킨다.
# =============================================================================

ARG ROS_DISTRO=humble

# ----------------------------------------------------------------------------
# base: 의존성 + ROBOTIS 패키지 빌드까지 끝낸 언더레이
# ----------------------------------------------------------------------------
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- NVIDIA(OpenGL) GUI 가속을 위한 환경변수 (RViz를 A6000으로 렌더) ---
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1

# --- 공통 도구 + SLAM/Nav2 (apt로 받는 게 빠른 것들만) ---
# 주의: dynamixel-sdk / turtlebot3-msgs 는 아래에서 소스로 clone하므로
#       여기 apt 목록에 넣지 않는다(중복/충돌 방지).
#       gazebo-* 도 넣지 않는다 - 필요한 의존성은 rosdep이 자동 해결.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      curl \
      wget \
      vim \
      python3-pip \
      python3-argcomplete \
      python3-colcon-common-extensions \
      python3-vcstool \
      ros-${ROS_DISTRO}-cartographer \
      ros-${ROS_DISTRO}-cartographer-ros \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# --- ROBOTIS 공식 소스(humble 브랜치)로 워크스페이스 빌드 ---
# 세 패키지 모두 소스로 통일. 커스텀(낮고 길게 개조한) 로봇의 URDF/파라미터를
# 직접 수정·오버라이드하기 위해 turtlebot3 도 소스로 둔다.
WORKDIR /turtlebot3_ws/src
RUN git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
 && git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
 && git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3.git

# rosdep 가 src 안의 패키지는 빌드 대상으로 인식하므로 apt로 다시 받지 않고,
# 부족한 외부 의존성(gazebo 관련 등)만 자동으로 채운다.
WORKDIR /turtlebot3_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && colcon build --symlink-install \
 && rm -rf /var/lib/apt/lists/*

# --- 대화형 셸(특히 docker compose exec ... bash)에서도 자동 source ---
# exec 는 entrypoint 를 거치지 않으므로 .bashrc 에 source 를 심어둔다.
# (${ROS_DISTRO} 를 빌드 시점 값으로 박기 위해 변수 확장은 이스케이프하지 않음)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "[ -f /turtlebot3_ws/install/setup.bash ] && source /turtlebot3_ws/install/setup.bash" >> /root/.bashrc \
 && echo "[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash" >> /root/.bashrc \
 && echo "export TURTLEBOT3_MODEL=\${TURTLEBOT3_MODEL:-burger}" >> /root/.bashrc \
 && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-30}" >> /root/.bashrc \
 && echo "export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" >> /root/.bashrc

# ----------------------------------------------------------------------------
# overlay: 사용자(=Michael) 본인의 SLAM/Nav 프로젝트가 들어갈 워크스페이스
# ----------------------------------------------------------------------------
FROM base AS overlay

# 본인 프로젝트 소스를 여기에 마운트(개발 중)하거나 COPY(배포 시)한다.
# 개발 단계에서는 compose의 volumes로 호스트 폴더를 마운트하는 것을 권장.
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws

# (배포용 예시) 프로젝트를 이미지에 굽고 싶을 때 아래 주석을 해제:
# COPY ./my_robot_ws/src /overlay_ws/src
# RUN source /turtlebot3_ws/install/setup.bash \
#  && rosdep install --from-paths src --ignore-src -r -y \
#  && colcon build --symlink-install

# --- 진입 시 환경 자동 source + TurtleBot3 모델/RMW/도메인 설정 ---
# context 루트(tb3-remote-pc/) 기준 경로이므로 docker/ 를 명시한다.
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 컨테이너 기본 환경값 (compose/run에서 -e로 덮어쓸 수 있음)
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=30
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
