# my_slam — slam_toolbox 기반 2D SLAM 패키지

> 로봇 분야를 전혀 모르는 사람도 이 문서와 코드를 같이 보면
> "무엇이 어떻게 계산되고, 그래서 코드를 왜 이렇게 작성했는지"를
> 이해할 수 있도록 쓴 문서다.

## 1. 이 패키지가 하는 일 (한 줄 요약)

**로봇(TurtleBot3)이 보내주는 라이다 데이터와 바퀴 이동량을 받아서,
"방의 지도"를 실시간으로 그리고 그 지도 안에서 로봇의 위치를 추정한다.**
이 문제를 SLAM(Simultaneous Localization and Mapping, 동시적 위치추정 및 지도작성)이라 부른다.

```
[로봇 (Raspberry Pi) — 이미 돌고 있음, 수정 안 함]     [Remote PC — 이 패키지]
  /scan      : 라이다 측정값 (초당 5회)          ──▶   slam_toolbox 노드가 구독
  /odom, /tf : 바퀴로 계산한 이동량              ──▶        │
  /tf_static : 부품들의 부착 위치 (URDF 치수)    ──▶        ▼
                                                     /map (지도) publish
                                                     map→odom TF publish
                                                          │
                                                          ▼
                                                     RViz2 로 시각화
```

## 2. 사전 지식 — 이것만 알면 나머지가 다 읽힌다

### 2.1 좌표계(frame): "위치"는 항상 "무엇 기준"인지가 필요하다

"벽이 2m 앞에 있다"는 말은 **누구의 앞**인지 정하지 않으면 의미가 없다.
로봇공학에서는 기준점마다 **좌표계(frame)** 를 정의한다.
frame 하나 = 원점 위치 + x/y/z 축 방향이 정해진 기준점이다.

이 프로젝트에 등장하는 frame 은 5개다:

| frame | 원점이 어디인가 | 성격 |
|---|---|---|
| `base_scan` | 라이다 부품의 중심 | 라이다 측정값의 기준 |
| `base_link` | 로봇 몸통의 중심 | 로봇 자신의 기준 |
| `base_footprint` | 몸통 중심을 바닥에 수직 투영한 점 | 2D 지도용 기준 (TurtleBot3 의 대표 frame) |
| `odom` | 로봇 전원을 켠 순간의 그 자리 | 오도메트리의 기준 (박제됨) |
| `map` | SLAM 이 만드는 지도의 원점 | 최종 목표 기준 |

### 2.2 `/odom` — 오도메트리: 바퀴가 굴러간 만큼 계산한 위치

바퀴 지름을 알고 있으므로, 모터의 엔코더가 "바퀴가 몇 바퀴 돌았는지"를 세면
이동 거리를 계산할 수 있다. 좌/우 바퀴 회전량의 차이로 회전각도 계산한다.
이것이 **오도메트리(odometry)** 이고, 로봇이 `/odom` 토픽으로 계속 내보낸다.

```
/odom (nav_msgs/Odometry)
├── pose : 출발점(odom frame) 기준 현재 위치/방향  예: x=1.5m, y=0.3m, 45도
└── twist: 현재 속도                              예: 전진 0.1m/s, 회전 0.2rad/s
```

**치명적 한계**: 바퀴는 미끄러진다. 오차가 스스로 복구되지 않고 **계속 누적**된다.
눈 감고 걸음 수로 위치를 추측하는 것과 같다 — 10걸음은 정확하지만 1000걸음이면 크게 틀어진다.
→ **"짧게는 부드럽고 정확, 길게는 틀어짐"**. SLAM 이 존재하는 이유가 이 한계 때문이다.

### 2.3 `/tf`, `/tf_static` — frame 간 변환을 방송하는 공용 게시판

TF(transform)는 "지금 이 순간 frame A 기준으로 frame B 가 어디에 어떤 각도로 있는지"를
시스템 전체에 방송하는 메커니즘이다. 토픽이 두 개로 나뉜다:

- **`/tf`** — 움직이는 변환. 예: 로봇 위치(odom→base_footprint)는 주행 중 계속 바뀌므로 초당 수십 번 갱신.
- **`/tf_static`** — 영원히 안 변하는 변환. 예: 라이다는 나사로 고정돼 있으므로
  몸통→라이다(base_link→base_scan) 변환은 한 번만 방송하면 된다.
  이 값들은 로봇의 **URDF 파일(부품 조립도)** 에 적힌 치수에서 나온다.

### 2.4 TF 트리 — 5개 frame 이 한 줄로 연결된다

```
map → odom → base_footprint → base_link → base_scan
```

화살표 하나 = 변환(transform) 하나. **누가 publish 하는지**가 역할 분담의 핵심이다:

| 변환 | 의미 | 누가 publish | 어디로 |
|---|---|---|---|
| `map → odom` | 오도메트리 누적 오차의 보정값 | ★ **slam_toolbox (이 패키지)** | /tf |
| `odom → base_footprint` | 출발점 기준 이동량 (오도메트리) | 로봇 bringup | /tf |
| `base_footprint → base_link` | 바닥에서 몸통 중심까지 높이 | 로봇 URDF | /tf_static |
| `base_link → base_scan` | 몸통 중심 기준 라이다 부착 위치 | 로봇 URDF | /tf_static |

**왜 map→base_footprint 를 바로 주지 않고 map→odom 을 끼워 넣는가?**
slam_toolbox 는 라이다 스캔을 지도에 맞춰보고 "오도메트리가 지금 얼마나 틀어져 있는지"
(= 누적 오차량)를 알아낸다. 그 오차량 자체를 `map→odom` 변환으로 publish 한다.

```
map → odom            : "출발점이 사실 지도 기준으로 여기다" = 오차 보정 (SLAM, 느리지만 정확)
odom → base_footprint : "출발점 기준 이만큼 이동했다"        = 오도메트리 (로봇, 빠르고 부드러움)
──────────────────────────────────────────────────────────
합성하면 = 지도 기준 로봇의 진짜 위치 (부드럽고 + 정확)
```

이 분업 덕에 SLAM 이 루프 클로저로 오차를 확 보정해도 map→odom 값만 점프하고,
odom 기준으로 도는 주행 제어는 흔들리지 않는다. 이 구조는 ROS 표준 규약
**REP-105** 이며, Nav2 등 모든 표준 도구가 이를 전제로 만들어져 있다.

### 2.5 SLAM 의 계산 원리: 스캔 매칭과 루프 클로저

slam_toolbox 내부에서 일어나는 일을 두 단어로 압축하면:

1. **스캔 매칭(scan matching)**: 오도메트리가 "대략 이만큼 움직였을 것"이라는 초기
   추정을 주면, 현재 라이다 스캔을 지금까지 그린 지도에 겹쳐보며 가장 잘 맞는 위치를
   찾아 미세 보정한다. 보정된 위치에서 스캔을 지도에 새로 그려 넣는다.
2. **루프 클로저(loop closure)**: 왔던 곳을 다시 인식하는 순간, "출발할 때의 이 벽과
   지금 보는 이 벽은 같은 벽"이라는 강력한 단서가 생긴다. 이 단서로 그동안 누적된
   오차 전체를 그래프 최적화(Ceres 솔버)로 한 번에 되감아 보정한다.

라이다 점 하나가 지도에 찍히는 전체 과정:

```
"라이다 기준 정면 2m"                          (base_scan 기준 측정값)
 → base_link→base_scan 변환:   "몸통 기준 앞 1.97m"     (/tf_static, URDF 치수)
 → odom→base_footprint 변환:   "출발점 기준 (3.2, 1.1)" (/tf, 오도메트리)
 → map→odom 변환:              "지도 기준 (3.5, 1.2)"   (/tf, SLAM 보정)
 → 지도의 해당 픽셀을 "벽"으로 칠함
```

## 3. 코드 구조 — 왜 "코드"가 거의 없는가

```
remote_pc/src/my_slam/
├── package.xml                              # 패키지 정보 + 의존성 선언
├── CMakeLists.txt                           # 빌드 규칙 (파일 install 만 함)
├── config/
│   └── mapper_params_online_async.yaml      # ★ slam_toolbox 동작을 결정하는 설정
├── launch/
│   └── slam.launch.py                       # slam_toolbox + RViz 실행 스크립트
└── rviz/
    └── slam.rviz                            # RViz 화면 구성 (QoS 설정 포함)
```

C++/Python 소스가 한 줄도 없다. 이는 ROS 2 의 정상적인 개발 방식이다:
**검증된 표준 노드(slam_toolbox)를 파라미터/런치로 조립하고, 내 로봇에만 있는 부분만
직접 코딩한다.** 현업에서도 SLAM 을 바닥부터 짜지 않고 표준 스택을 커스텀 로봇에 맞게
통합·튜닝한다. 이 프로젝트에서 직접 코딩이 들어가는 부분은 추후 Vision AI 노드다.

### 사용한 패키지/라이브러리

| 패키지 | 역할 | 왜 선택했나 |
|---|---|---|
| **slam_toolbox** | SLAM 엔진 (스캔매칭 + 그래프 최적화 + 지도 생성) | 현 ROS 2 사실상 표준. Cartographer 대비 정확도·자원효율·유지보수 우위 |
| rviz2 | 시각화 | ROS 표준 시각화 도구 |
| launch / launch_ros | 노드 실행 스크립트 프레임워크 | ROS 2 표준 |
| ament_cmake | 빌드 시스템 | 설정 파일 install 용 |
| (nav2_map_server) | 완성된 지도를 파일로 저장 | 실행 시에만 사용 (`map_saver_cli`) |

slam_toolbox 는 ROS 2 본체에 포함되지 않으며, 이 프로젝트에서는
`docker/Dockerfile` 에서 `ros-humble-slam-toolbox` 로 설치한다.
연산은 전부 Remote PC 에서 하므로 로봇(Pi)에는 설치하지 않는다.

### slam_toolbox 의 여러 모드 중 online async 를 쓰는 이유

- **online**: 저장된 데이터가 아니라 실시간으로 들어오는 데이터를 처리
- **async**: 처리가 밀리면 스캔을 버리더라도 항상 최신 스캔 기준으로 동작 (실시간성 우선)
- 대안인 sync 모드는 모든 스캔을 빠짐없이 처리(오프라인 고품질 지도용)라서
  실시간 주행 목적에는 async 가 현업 기본 선택이다.

## 4. 설정 파일 읽는 법 — `mapper_params_online_async.yaml`

slam_toolbox 기본값에서 **이 로봇에 맞게 바꾼 항목**이 핵심이다 (파일 내 ★ 주석):

| 파라미터 | 값 | 왜 이 값인가 |
|---|---|---|
| `base_frame` | `base_footprint` | TurtleBot3 의 대표 frame (base_link 아님). 로봇 쪽 TF 트리와 일치해야 함 |
| `odom_frame` / `map_frame` | `odom` / `map` | REP-105 표준 이름. 로봇 bringup 과 일치해야 함 |
| `scan_topic` | `/scan` | 로봇 라이다 토픽 |
| `max_laser_range` | `3.5` | LDS 라이다의 실제 최대 사거리 스펙. 크게 잡으면 사거리 밖 노이즈가 지도에 들어감 |
| `minimum_travel_distance` | `0.2` (기본 0.5) | 이만큼 이동할 때마다 스캔을 지도에 반영. 소형 로봇·좁은 실내라 촘촘하게 |
| `minimum_travel_heading` | `0.2` rad (약 11도) | 회전 시에도 지도 갱신 |
| `mode` | `mapping` | 지도 작성 모드. 지도 완성 후 주행 단계에서는 localization 모드로 전환 예정 |
| `transform_publish_period` | `0.02` | map→odom TF 를 50Hz 로 publish |
| `map_update_interval` | `5.0` | /map "토픽"의 갱신 주기(시각화용). 내부 지도 그래프는 스캔마다 갱신됨 |

나머지(Ceres 솔버, 상관관계 탐색, 루프 클로저 임계값들)는 검증된 기본값을 유지했다.
지도 품질 문제가 생기면 그때 근거를 가지고 조정한다.

## 5. 런치 파일 — `slam.launch.py`

"어떤 노드를 어떤 설정으로 띄울지"를 기술하는 실행 스크립트다. 하는 일:

1. `slam_toolbox` 패키지의 `async_slam_toolbox_node` 실행체를, 우리 yaml 을 파라미터로 실행
2. `use_rviz:=true`(기본)면 RViz2 를 우리 `.rviz` 설정으로 함께 실행

```bash
ros2 launch my_slam slam.launch.py                   # SLAM + RViz
ros2 launch my_slam slam.launch.py use_rviz:=false   # SLAM 만
```

## 6. RViz 설정과 QoS 함정 — `slam.rviz`

QoS(Quality of Service)는 ROS 2 토픽 통신의 "전달 보장 수준" 계약이다.
**publisher 와 subscriber 의 계약이 안 맞으면 에러 없이 조용히 데이터가 안 온다.**
이 프로젝트에서 걸리는 지점 두 곳을 `.rviz` 파일에 미리 박아 두었다:

| 토픽 | publisher 설정 | RViz 구독 설정 | 이유 |
|---|---|---|---|
| `/scan` | BEST_EFFORT | **Best Effort** | 센서 데이터는 "유실돼도 최신값 우선" 이 관례. Reliable 로 구독하면 아예 안 보임 |
| `/map` | TRANSIENT_LOCAL | **Transient Local** | "늦게 접속한 구독자에게도 마지막 지도를 준다"(latch). Volatile 로 구독하면 다음 갱신까지 지도가 안 보임 |

## 7. 실행 방법

**전제조건**: 로봇(Pi)에서 `turtlebot3_bringup` 이 떠 있고, Discovery Server 가 동작 중.

```bash
# Remote PC 에서
docker compose up -d
docker compose exec remote-pc bash

# 컨테이너 안 (RViz 를 서버 물리 세션에 띄우려면 DISPLAY 지정, 예: :1)
export DISPLAY=:1
ros2 launch my_slam slam.launch.py

# 로봇을 teleop 으로 천천히 몰면서 지도가 그려지는 것을 확인
# 지도가 완성되면 저장:
mkdir -p /overlay_ws/maps
ros2 run nav2_map_server map_saver_cli -f /overlay_ws/maps/my_map
```

저장하면 `my_map.pgm`(지도 이미지) + `my_map.yaml`(해상도/원점 메타데이터)가 생기며,
이후 Nav2 주행 단계에서 이 지도를 불러 쓴다.

## 8. 검증 기록 (2026-07-07, 실물 로봇 연결 상태에서 확인)

- `/scan` 수신율 4.98Hz (LDS 정상 주기)
- slam_toolbox 가 `map→odom` TF publish 확인 (`tf2_echo map odom`)
- `/map` 실제 생성 확인: 54×103 셀, 해상도 0.05m
- 파라미터 로드 확인: 로그에 Ceres 솔버, 스택 크기 등 yaml 값 반영됨

## 9. 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `ros2 topic list` 에 로봇 토픽이 안 보임 (통신은 정상인데) | Discovery Server 환경에서 CLI 는 자기 관심 토픽만 발견함 | `export ROS_SUPER_CLIENT=TRUE` 후 `ros2 daemon stop && ros2 daemon start` |
| RViz 에서 /scan 이 안 보임 | QoS 불일치 (Reliable 구독) | LaserScan 디스플레이의 Reliability 를 Best Effort 로 (slam.rviz 에는 반영됨) |
| RViz 에서 /map 이 한참 뒤에야 보임 | QoS Durability 불일치 | Map 디스플레이의 Durability 를 Transient Local 로 (slam.rviz 에는 반영됨) |
| 실행 로그의 "minimum laser range (0.0) exceeds (0.1)" 경고 | 라이다 최소사거리보다 작은 설정값 | 자동 클리핑되므로 무해 |
| 회전 시 벽이 이중으로 그려짐 / 지도가 밀림 | base_link→base_scan TF(라이다 부착 위치)가 실물과 다름 | URDF 실측 보정 필요 (아래 한계 참고) |

## 10. 현재 한계와 다음 단계

지금은 로봇이 보내주는 **ROBOTIS 표준 URDF 의 TF** 를 그대로 쓰고 있다.
이 로봇은 라이다(LDS) 위치/높이가 표준과 다르므로, `base_link→base_scan` 변환이
실물과 어긋난 만큼 지도 품질에 오차가 들어간다 (2.5절의 변환 과정에서 두 번째 줄이 틀어짐).

**다음 작업**: LDS 부착 위치를 실측해서 커스텀 URDF 로 보정 (`description/` 에서 작업 예정).
그 다음 RealSense camera_link TF 추가 → Nav2 → Vision AI 순서로 진행한다.
