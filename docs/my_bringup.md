# my_bringup — Remote PC 통합 실행 (SLAM + Nav2 + Vision + RViz 한 창)

> 각 컴포넌트의 원리는 각자의 문서(my_slam.md, my_navigation.md, my_vision.md)에 있다.
> 이 패키지는 그것들을 **한 명령으로 띄우고 한 RViz 창에서 보는 조합**만 담당한다 — 설정은 각 패키지에 그대로.

## 1. 한 줄 요약

```
Pi:  ros2 launch realsense_bringup full_bringup.launch.py        # 로봇 기본 + 카메라
서버: export DISPLAY=:0; ros2 launch my_bringup system.launch.py   # SLAM + Nav2 + Vision + RViz
```
로봇이 지도를 만들며 주행하고, 카메라가 본 물체가 **지도 위 절대 위치에 마커로 찍히는** 화면이 나온다.

## 2. 구조

```
remote_pc/src/my_bringup/
├── launch/system.launch.py   # 조합: navigation.launch(use_slam/map, RViz 끔) + vision.launch(target_frame=map) + rviz2
└── rviz/system.rviz          # nav.rviz + RobotModel(/robot_description) + VisionMarkers(/vision/markers) + VisionImage
```

| 인자 | 기본 | 의미 |
|---|---|---|
| `use_slam` / `map` | true / '' | SLAM 하며 주행 (기본) / 저장 지도 + AMCL (`use_slam:=false map:=…yaml`) |
| `vision` | true | Vision AI 노드 포함 |
| `target_frame` | map | 검출 물체 좌표·마커 프레임 — 지도 위 절대 위치. (SLAM/지도가 없으면 base_link 로 fallback) |
| `backend`, `threshold` | '' | my_vision 런치로 통과 (비우면 vision_params.yaml) |
| `rviz` | true | RViz (서버 물리 세션에 뜸 → `DISPLAY=:0`, VNC 로 봄) |

## 3. RViz 화면 구성 (system.rviz)

| 디스플레이 | 토픽 | 보이는 것 |
|---|---|---|
| Map | /map (Transient Local) | slam_toolbox 가 만드는 점유격자 |
| GlobalCostmap / LocalCostmap | /global_costmap/costmap, /local_costmap/costmap | 장애물 + inflation 안전 여유 |
| Path-Global / Path-Local | /plan, /local_plan | 계획 경로 / DWB 로컬 경로 |
| Footprint | /local_costmap/published_footprint | **실측 직사각형**(앞 0.062 / 뒤 0.220 / 좌우 ±0.090) |
| LaserScan | /scan (Best Effort) | 라이다 |
| RobotModel | /robot_description (Transient Local, 브리지 경유) | 실측 보정 URDF 형상 (카메라 박스 포함) |
| **VisionMarkers** | /vision/markers | 검출 물체 구 + "이름 점수 \| 거리" 라벨, map 좌표 |
| **VisionImage** | /vision/annotated/compressed | 검출 박스가 그려진 카메라 영상 (compressed 플러그인 필요 — 컨테이너에 포함) |

목표 지정은 툴바 **2D Goal Pose**, AMCL 모드의 초기 위치는 **2D Pose Estimate**.

## 4. 왜 이렇게 조합했나

- **RViz 는 하나만**: navigation.launch 와 slam.launch 는 각자 RViz 를 띄울 수 있어(`use_rviz`) 통합 런치에선
  전부 끄고 `system.rviz` 하나만 띄운다. 디스플레이·QoS 설정이 한 파일에 모여 관리가 쉽다.
- **Vision 은 map 프레임으로**: 검출 노드의 `target_frame` 만 `map` 으로 넘기면 SLAM 의 `map→odom` 을 통해
  물체가 지도 위 절대 위치가 된다. 로봇이 움직여도 물체 마커는 제자리에 남는다(lifetime 내).
- **설정은 각 패키지에**: 통합 런치는 인자를 통과만 시킨다. 임계값·모델·Nav2 파라미터를 바꾸려면
  각 패키지의 YAML 을 고치면 되고, 통합 런치는 손댈 필요가 없다.

## 5. 실측 (2026-09-08)

- slam_toolbox + Nav2 전 노드 lifecycle active, 검출 노드 `[map xyz m]` 로 출력(6fps, TensorRT 8ms),
  `/vision/markers` frame_id map, RViz 31fps. 캡처: 지도·코스트맵·실측 footprint·스캔 한 창.
- 발견: 컨테이너에 `image_transport` 의 compressed 플러그인이 없으면 RViz Image 가 "No Image"
  (`image_transport/compressed_sub does not exist`) → Dockerfile 에 `ros-humble-image-transport-plugins` 추가.

![통합 RViz 캡처](./images/rviz_system_demo.png)

위 캡처(2026-09-08): 왼쪽 footprint 안이 로봇, 앞 2m 책장 벽 위치에 검출된 책·키보드 마커(구 + "이름 점수 | 거리"),
왼쪽 위 패널이 검출 박스가 그려진 카메라 영상. 카메라 광학 좌표 → base_link → map 변환이 실물과 맞음.

## 6. 다음

- 실주행: 목표점 주행하며 footprint·inflation 튜닝 (my_navigation.md 8장), 검출 마커의 지도 위 정합 확인.
- 검출 물체를 Nav2 목표로 삼는 "물체 찾아가기" 데모 (Detection → goal_pose).
