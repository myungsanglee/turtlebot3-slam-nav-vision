# CLAUDE.md — turtlebot3-slam-nav-vision

> 이 파일은 Claude Code 가 세션 시작 시 읽는 프로젝트 컨텍스트다.
> 새 세션에서도 이 문서만으로 프로젝트 전체 상황을 파악할 수 있게 유지한다.

## 1. 프로젝트 목적

ROBOTIS TurtleBot3 실물 로봇으로 **SLAM · Navigation · Vision AI** 를 구현하는
개인 포트폴리오 프로젝트. 최종 지향점은 **로봇청소기 같은 자율주행 로봇**
(돌아다니며 지도를 만들면서 동시에 주행 + 카메라 기반 인식).

- **목표**: 커리어 역량 향상 및 국내/해외(특히 독일/DACH) 로보틱스 취업 포트폴리오.
- 따라서 코드 품질·문서화·현업 표준 스택 사용이 중요하다. 데모를 그대로 쓰는 게
  아니라 **커스텀 로봇에 맞춰 실제 문제를 해결한 과정**을 보여주는 것이 핵심 차별점.

## 2. 시스템 아키텍처 (2대 + 중앙 디스커버리)

물리적으로 떨어진 두 머신을 Tailscale(WireGuard VPN)로 연결한다.

```
[Raspberry Pi @ TurtleBot3]        [Remote PC / 회사 서버]
  - turtlebot3_bringup (고정)         - Docker: ROS 2 Humble 컨테이너
  - RealSense D435i (추가)            - SLAM / Nav2 / Vision AI (개발 대상)
  - Tailscale                        - Discovery Server (컨테이너)
        │                            - Tailscale, NVIDIA RTX A6000
        └──────── Tailscale (DDS / Discovery Server) ────────┘
```

- **역할 분리**: Pi = 센서 publish 전용(엣지), Remote PC = 무거운 연산 전부.
- **bringup 은 변경하지 않는다.** 모터 제어/오도메트리 등 로봇 기본은 ROBOTIS
  `turtlebot3_bringup` 을 그대로 사용. 로봇 쪽에 새로 추가하는 것은 **RealSense
  카메라 영상 publish 뿐**.
- Remote PC 는 그 토픽들(`/scan`, `/odom`, `/imu`, `/camera/*`)을 받아
  **나만의 SLAM/Nav/Vision** 을 돌린다.

## 3. 네트워크 / 디스커버리 (중요 — 삽질 끝에 확정된 구성)

집-회사처럼 다른 망이라 DDS 멀티캐스트가 안 된다. **Fast DDS Discovery Server**
방식으로 확정(XML/initialPeers 방식은 localhost 간섭·순서 의존 문제로 폐기).

- RMW: `rmw_fastrtps_cpp` (Humble 기본)
- **`ROS_DISCOVERY_SERVER=<서버 Tailscale IP>:11811`** 를 모든 노드가 바라봄
- Discovery Server 는 Remote PC 의 Docker 컨테이너에서 상시 실행
  (`fastdds discovery -i 0 -l <서버 IP> -p 11811`, compose 의 별도 서비스)
- `ROS_DOMAIN_ID` 는 Pi 와 Remote PC 가 **반드시 동일** (현재: 30 — 실제 값 확인 후 유지)
- 같은 와이파이(LAN) 테스트 시엔 `ROS_DISCOVERY_SERVER` 를 끄면 멀티캐스트로 동작
  (전환용 셸 함수 `ros_ds` / `ros_lan` 사용)

주소(실제 값):
- 서버 Tailscale IP: `100.95.193.1`
- 로봇(Pi) Tailscale IP: `100.71.74.81`

> QoS 주의: `/scan` 은 publisher 가 BEST_EFFORT. 구독/RViz 는 Best Effort 로 맞출 것.

## 4. 기술 스택 (확정)

| 영역 | 선택 | 비고 |
|------|------|------|
| ROS 2 | **Humble** (Ubuntu 22.04) | TurtleBot3 공식 지원, 생태계 성숙 |
| 실행 환경 | Docker (24.04 호스트 위 22.04 컨테이너) | 베이스 `osrf/ros:humble-desktop` |
| SLAM | **slam_toolbox** | 현 ROS2 표준. Cartographer 아님(유지보수/Nav2 통합/성능 우위) |
| Navigation | **Nav2** | ROS2 내비게이션 표준 |
| 센서 융합 | **robot_localization (EKF)** | LiDAR+IMU+엔코더 융합, 커스텀 로봇 odom 안정화 |
| Localization | AMCL 또는 slam_toolbox localization 모드 | 지도 완성 후 주행 단계 |
| 카메라 | **realsense2_camera** | D435i, depth/color/pointcloud |
| Vision AI | 본인 파이프라인 (RF-DETR/RTMDet + TensorRT) | ROS2 노드로 래핑, `/camera` 구독→추론→publish |
| 시각화 | RViz2 (+ 추후 Foxglove) | SLAM+Nav+영상 통합 .rviz 한 창 |

## 5. 커스텀 로봇 — 반드시 반영할 것 (표준 데모 그대로 못 씀)

이 TurtleBot3 는 ROBOTIS 표준과 형태가 다르다:

- **바퀴 폭(wheel separation): 표준과 동일** → 휠 오도메트리 파라미터는 그대로 OK.
- **LDS(라이다) 센서의 위치와 높이가 다름** → URDF/xacro 에서
  `base_link → base_scan` 의 x/y/z 오프셋을 **실측값으로 수정 필수**.
  (안 맞으면 slam_toolbox 맵이 어긋나거나 회전 시 이중으로 그려짐)
- **RealSense 추가** → `base_link → camera_link` static transform 을 실측값으로 추가.
- Nav2 풋프린트: 로봇 형태가 표준과 다르면 `robot_radius` 대신 실제 외형에 맞는
  다각형 `footprint` 고려.

> 이 "커스텀 로봇에 맞춘 TF/URDF 보정" 과정 자체가 포트폴리오의 차별점이므로
> 문서화(README, 커밋 메시지)를 잘 남긴다.

## 6. 레포 구조 (모노레포 — Pi/서버 코드 한 곳에)

이 레포 하나에 Remote PC 와 Raspberry Pi(RealSense) 코드를 모두 둔다.
Pi 배포는 git pull 로. (Claude Code 는 Remote PC 에서 이 레포 전체를 보고 작업)

```
turtlebot3-slam-nav-vision/
├── CLAUDE.md                  # 이 파일
├── README.md
├── docs/                      # 컴포넌트별 상세 문서 (개발 완료 시마다 작성 — 9번 규칙)
│   ├── my_slam.md
│   └── my_navigation.md
├── docker/                    # Remote PC 컨테이너 (ROS2 Humble)
│   ├── Dockerfile
│   └── entrypoint.sh
├── docker-compose.yml         # discovery-server + remote-pc 서비스
├── config/                    # 파라미터, .rviz, EKF 설정 등
├── remote_pc/                 # Remote PC 에서 도는 패키지들
│   └── src/
│       ├── my_slam/           # slam_toolbox 설정/런치
│       ├── my_navigation/     # Nav2 설정/런치
│       └── my_vision/         # Vision AI 노드 (TensorRT 추론)
├── robot/                     # Raspberry Pi 에서 도는 것 (RealSense 관련)
│   └── src/
│       └── realsense_bringup/ # RealSense 실행 런치 (bringup 은 건드리지 않음)
└── description/               # URDF/xacro (커스텀 로봇: LDS/카메라 TF 실측 반영)
```

> 참고: 현재 컨테이너의 colcon 워크스페이스는 `/overlay_ws` 에 마운트됨
> (compose 의 `./my_robot_ws:/overlay_ws`). 위 구조로 정리 시 마운트 경로도 함께 갱신.

## 7. 개발 워크플로

- **편집**: 맥북에서 VS Code Remote-SSH 로 Remote PC 에 접속(Tailscale). Claude Code 도 여기서.
- **빌드/실행**: Remote PC 의 Docker 컨테이너 안 (`docker compose exec remote-pc bash`)
- **RViz 확인**: 서버 물리 세션을 x11vnc 로 미러링, 맥에서 RealVNC/화면공유로 접속
  (컨테이너 RViz 는 `DISPLAY` 를 서버 물리 세션 `:0`/`:1` 에 맞춰 실행)
- **환경 자동 source**: 컨테이너는 entrypoint(run 용) + .bashrc(exec 용) 양쪽에서
  ROS 환경을 자동 source 하도록 이미 구성됨.

## 8. 현재 진행 상황 / 다음 할 일

**완료**
- Docker ROS2 Humble 컨테이너 (osrf 베이스 + ROBOTIS 소스 3종 + SLAM/Nav2)
- Discovery Server 방식으로 Pi↔서버 통신 안정화 (map 그려짐, 양방향 teleop OK)
- x11vnc 로 RViz 원격 확인 환경 구축
- 레포 구조 정리 (위 6번 구조 스캐폴딩, README / .gitignore)
- **my_slam 패키지** — slam_toolbox(online async) 설정/런치/RViz 구성.
  실물 로봇으로 엔드투엔드 검증 완료(/scan 5Hz, map→odom TF, /map 생성).
  상세는 `docs/my_slam.md`
- **my_navigation 패키지** — Nav2 설정/런치/RViz. 기본 모드는 SLAM 동시 실행
  (지도 만들며 주행), 저장 지도+AMCL 모드 지원. 전체 lifecycle 활성화·코스트맵
  publish 실기 검증 완료(실주행 테스트는 사용자 입회 하 예정). footprint 는
  표준 burger 임시값(★실측 교체 필요). 상세는 `docs/my_navigation.md`

**다음 (우선순위 순)**
1. **커스텀 로봇 URDF 보정** — LDS 위치/높이 실측 반영, RealSense camera_link TF 추가
2. **Nav2 footprint 실측 교체** — nav2_params.yaml 의 robot_radius(임시 0.105) →
   실측 다각형 footprint
3. RealSense 런치 (Pi 쪽) + bringup 과 함께 뜨는 통합 런치
4. robot_localization EKF 설정 (LiDAR+IMU+엔코더 융합)
5. SLAM + Nav2 + RealSense + RViz 통합 런치 (한 창에서 다 보기)
6. Vision AI 노드 (TensorRT 추론) 통합

## 9. 규칙 / 선호

- SLAM 은 slam_toolbox 사용 (Cartographer 로 되돌리지 말 것 — 위 4번 근거).
- 로봇(Pi)의 `turtlebot3_bringup` 은 수정하지 않는다. 추가는 RealSense 만.
- 커스텀 로봇 치수(LDS TF 등)는 임의 값이 아니라 **실측값**을 쓴다. 값이 불확실하면
  하드코딩하지 말고 사용자에게 실측을 요청할 것.
- 취업 포트폴리오이므로 코드와 문서(README, 커밋)를 깔끔하게 유지한다.
- **컴포넌트 개발이 끝나면 반드시 `docs/<패키지명>.md` 문서를 작성한다.**
  내용: 코드 구조, 사용 라이브러리와 선택 이유, 무엇을 어떻게 개발했는지,
  그리고 관련 개념(좌표계/TF/QoS 등)을 로봇 분야를 전혀 모르는 사람도
  코드와 문서만 보고 "어떤 원리로 계산되고 왜 이렇게 작성했는지" 이해할 수
  있는 수준으로 설명. 예시: `docs/my_slam.md`
- 응답/주석은 한국어 기본.
