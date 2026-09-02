# turtlebot3-slam-nav-vision

ROBOTIS **TurtleBot3 실물 로봇**으로 **SLAM · Navigation · Vision AI**를 구현하는
개인 포트폴리오 프로젝트입니다. 최종 지향점은 로봇청소기처럼 **돌아다니며 지도를
만들면서 동시에 주행하고, 카메라로 주변을 인식하는 자율주행 로봇**입니다.

표준 데모를 그대로 쓰는 게 아니라, **표준과 형태가 다른 커스텀 로봇**에 맞춰
실제 문제(센서 TF 보정, 원격 네트워크 통신 등)를 해결한 과정을 보여주는 것이
핵심입니다.

## 시스템 아키텍처

물리적으로 떨어진 두 머신을 Tailscale(VPN)로 연결하고 역할을 분리합니다.

```
[Raspberry Pi @ TurtleBot3]              [Remote PC / 서버]
  - turtlebot3_bringup (센서 publish)      - Docker (ROS 2 Humble 컨테이너)
  - RealSense D435i (compressed 전송)      - SLAM / Nav2 / Vision AI (개발 대상)
  - zenoh-bridge (systemd)                - zenoh-bridge (컨테이너)
  - Tailscale                             - Tailscale, NVIDIA GPU
        └──────── Tailscale (Zenoh Bridge, tcp/7447) ────────┘
```

- **Pi** = 센서 publish 전용 엣지 (bringup 은 수정하지 않음)
- **Remote PC** = 무거운 연산 전부 (Docker 컨테이너 안에서 개발)

자세한 아키텍처·네트워크·기술 스택은 [CLAUDE.md](./CLAUDE.md)를 참고하세요.

## 기술 스택

| 영역 | 선택 |
|------|------|
| ROS 2 | Humble (Ubuntu 22.04) |
| 실행 환경 | Docker (`osrf/ros:humble-desktop` 기반) |
| SLAM | slam_toolbox (online async) |
| Navigation | Nav2 |
| 원격 통신 | **Zenoh Bridge** (zenoh-bridge-ros2dds) — Fast DDS 의 VPN 한계 진단 후 전환 |
| 카메라 | RealSense D435i — **자체 pyrealsense2 노드** (공식 노드의 Pi4 USB 불안정을 캘리브레이션 캐시·프로세스 분리로 우회), 컬러 + 컬러 정렬 depth compressed 원격 전송 |
| Vision AI | 자체 파이프라인 (RF-DETR/RTMDet + TensorRT) — 예정 |
| 센서 융합 | robot_localization (EKF) — 예정 |
| 시각화 | RViz2 |

## 레포 구조

```
turtlebot3-slam-nav-vision/
├── docs/                      # 컴포넌트별 상세 문서 + 트러블슈팅 로그
├── docker/                    # Remote PC 컨테이너 (Dockerfile, entrypoint)
├── docker-compose.yml         # zenoh-bridge + remote-pc 서비스
├── config/                    # zenoh 브리지 설정 등 공용 설정
├── remote_pc/src/             # Remote PC 패키지 (my_slam, my_navigation, my_vision)
├── robot/src/                 # Raspberry Pi 패키지 (realsense_bringup)
└── description/               # URDF/xacro (커스텀 로봇 센서 TF 실측 보정)
```

## 개발한 내용

각 컴포넌트의 "정상 동작 원리"는 `docs/` 에 초심자도 이해할 수준으로 정리돼 있습니다.

| 컴포넌트 | 내용 | 상태 | 문서 |
|---|---|---|---|
| **my_slam** | slam_toolbox 기반 2D SLAM. 로봇 라이다(`/scan`)로 지도 작성 + 위치추정 | ✅ 실물 검증 | [docs/my_slam.md](./docs/my_slam.md) |
| **my_navigation** | Nav2 자율주행. 지도 만들며 주행(기본) / 저장 지도+AMCL 모드 | ✅ 실기 검증 (실주행 예정) | [docs/my_navigation.md](./docs/my_navigation.md) |
| **description** | 커스텀 로봇 URDF 센서 TF 실측 보정 (LDS 위치, IMU 회전) | ✅ Pi 배포·TF 검증 | [docs/description.md](./docs/description.md) |
| **realsense_bringup** | D435i 브링업 — **자체 pyrealsense2 노드**: 컬러 + 컬러에 정렬된 depth(PNG 16bit, mm) compressed publish. 공식 노드가 Pi4 에서 간헐 실패하는 문제를 캘리브레이션 캐시·프로세스 분리 감시·온화한 복구로 해결 | ✅ 15fps 원격 수신 | [docs/realsense_bringup.md](./docs/realsense_bringup.md) |
| **인프라/네트워크** | Tailscale + **Zenoh Bridge** (Fast DDS Discovery Server 의 VPN 한계를 진단 후 전환) | ✅ 검증 완료 | [docs/troubleshooting.md](./docs/troubleshooting.md) |
| my_vision | Vision AI 노드 (TensorRT 추론) | ⏳ 예정 | — |

## 시작하기

> 최초 환경 구축(새 머신)은 [docs/pi_setup.md](./docs/pi_setup.md)(로봇) ·
> [docs/server_setup.md](./docs/server_setup.md)(서버) 참고.
> 아래는 구축이 끝난 상태에서의 일상 실행 절차입니다.

### 1. 로봇(Pi) 쪽 실행

zenoh-bridge 는 systemd 로 부팅 시 자동 실행됩니다 (`systemctl status zenoh-bridge`).

```bash
# 통합 실행 — 로봇 기본(모터·오도메트리·라이다) + RealSense 카메라 한 번에
ros2 launch realsense_bringup full_bringup.launch.py
#   카메라 제외: camera:=false / 공식 드라이버로 비교: camera_driver:=realsense2
```

개별 실행:
```bash
ros2 launch turtlebot3_bringup robot.launch.py      # 셸 1 — 로봇 기본
ros2 launch realsense_bringup rs_camera.launch.py   # 셸 2 — 카메라 (자체 노드, 기본)
#   대역폭 절약: fps:=6 jpeg_quality:=70
```

> 카메라 노드는 USB 가 꼬여도 스스로 복구한다(로그의 `[자식]` 단계 메시지·10초 stats 참고).
> ⚠️ 시작 중인 카메라 프로세스를 `kill -9` 하지 말 것 — 카메라가 USB 버스에서 떨어져
> 물리 재연결이 필요해진다 ([docs/realsense_bringup.md](./docs/realsense_bringup.md) 7장).

### 2. 서버 컨테이너 실행

```bash
docker compose up -d              # zenoh-bridge + remote-pc 기동
docker compose exec remote-pc bash
ros2 topic hz /scan               # 로봇 연결 확인 (~5Hz)
```

> 드물게 "토픽은 보이는데 데이터 0"이면 브리지만 재시작하면 복구됩니다
> (`docker compose restart zenoh-bridge` / Pi: `sudo systemctl restart zenoh-bridge`).

### 3. SLAM — 지도 만들기 (서버)

```bash
export DISPLAY=:1                          # RViz 를 서버 세션에 표시
ros2 launch my_slam slam.launch.py
# 로봇을 teleop 으로 천천히 몰면 지도가 그려짐. 완성되면:
ros2 run nav2_map_server map_saver_cli -f /overlay_ws/maps/my_map
```

### 4. Navigation — 자율주행 (서버)

```bash
# 모드 1: SLAM 하며 주행 (지도 만들며 목표점 이동)
ros2 launch my_navigation navigation.launch.py

# 모드 2: 저장된 지도로 주행
ros2 launch my_navigation navigation.launch.py \
    use_slam:=false map:=/overlay_ws/maps/my_map.yaml
```
RViz 툴바의 **2D Goal Pose** 로 목표점을 찍으면 로봇이 이동합니다.

### 참고

- Pi↔서버 통신은 zenoh-bridge 가 담당하며, 브리징되는 토픽은
  `config/zenoh-bridge-server.json5` / `robot/config/zenoh-bridge-pi.json5` 의
  allow 리스트로 관리합니다 (카메라는 compressed 만 — raw 는 대역폭 초과).
- 카메라 토픽: `/camera/color/compressed`(JPEG) · `/camera/depth/compressed`(컬러 정렬
  depth, PNG 16bit mm) · `/camera/color/camera_info`, 셋 다 같은 stamp, best effort QoS.
- 상세 개발 문서: [docs/](./docs/) · 프로젝트 전체 컨텍스트: [CLAUDE.md](./CLAUDE.md)
