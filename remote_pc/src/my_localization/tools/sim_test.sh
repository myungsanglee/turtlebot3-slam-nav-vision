#!/bin/bash
# 로봇 없이 자동 위치 찾기 전체 흐름 시험 (map_server + AMCL + relocalizer + 가상 스캔). 컨테이너 안에서:
#   bash /overlay_ws/src/my_localization/tools/sim_test.sh /overlay_ws/maps/<지도>.yaml x y yaw_deg
# 실제 로봇과 섞이지 않게 별도 ROS_DOMAIN_ID 를 쓴다. 끝내려면 Ctrl+C.
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_SIM:-77}
MAP=$1; shift
DIR=$(dirname "$0")
PARAMS=$(ros2 pkg prefix my_navigation)/share/my_navigation/config/nav2_params.yaml
pids=()
# ros2 run 은 실제 노드를 자식으로 띄운다 → 자식까지 SIGINT
cleanup() { for p in "${pids[@]}"; do pkill -INT -P "$p" 2>/dev/null; kill -INT "$p" 2>/dev/null; done; wait; }
trap cleanup EXIT INT TERM
python3 "$DIR/fake_scan.py" "$MAP" "$@" & pids+=($!)
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP" & pids+=($!)
ros2 run nav2_amcl amcl --ros-args --params-file "$PARAMS" & pids+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true \
  -p node_names:="['map_server','amcl']" & pids+=($!)
ros2 run my_localization relocalizer_node --ros-args \
  --params-file "$(ros2 pkg prefix my_localization)/share/my_localization/config/relocalizer.yaml" \
  -p map_yaml:="$MAP" & pids+=($!)
wait
