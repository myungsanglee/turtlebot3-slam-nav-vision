#!/bin/bash
# =============================================================================
# entrypoint.sh - 컨테이너 진입 시 ROS 2 환경을 단계적으로 source
#   1) ROS 2 기본          /opt/ros/humble
#   2) TurtleBot3 언더레이  /turtlebot3_ws  (ROBOTIS 패키지)
#   3) 사용자 오버레이      /overlay_ws     (본인 프로젝트, 빌드돼 있으면)
# =============================================================================
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -f /turtlebot3_ws/install/setup.bash ]; then
  source /turtlebot3_ws/install/setup.bash
fi

if [ -f /overlay_ws/install/setup.bash ]; then
  source /overlay_ws/install/setup.bash
fi

# 로봇(파이)과 반드시 일치시켜야 하는 값들 — compose에서 주입됨
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

exec "$@"
