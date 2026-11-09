# web UI 설계 — 터미널 대신 웹 화면으로 로봇을 운용한다

> 상태: **설계 (2026-11-09)**. 구현 전 문서이며, 구현하면서 바뀐 결정은 이 문서를 고친다.
> 구현이 끝나면 패키지별 문서(`docs/my_web_api.md` 등)로 "정상 동작 방식"을 옮긴다.

## 1. 왜 만드나

지금은 서버에서 `docker compose exec` → `ros2 launch my_bringup system.launch.py` 를 직접 치고,
RViz 를 VNC 로 봐야 로봇을 쓸 수 있다. 개발자에겐 충분하지만 **제품은 이렇게 운용하지 않는다.**

실제 로봇 제품의 공통 구조는 세 층이다 (조사 근거는 9장).

| 층 | 제품에서 | 지금 이 프로젝트 |
|---|---|---|
| 실행 | 부팅하면 ROS 노드가 **서비스로 자동 시작** (Clearpath 는 `robot_upstart` 로 launch 를 systemd 서비스화) | 터미널에서 수동 launch |
| 연결 | 화면은 ROS 메시지를 직접 다루지 않고 **로봇 전용 API** 를 쓴다 (MiR 는 웹 UI 와 별도로 REST API 제공) | 없음 |
| 화면 | **브라우저**로 PC·태블릿·폰에서 접속 (MiR 대시보드: 지도 그리기, 조이스틱, 미션) | RViz (개발자 도구) |

목표는 **"전원을 켜고 브라우저를 열면 지도 만들기·주행·카메라 확인을 버튼으로 할 수 있는 상태"**.
RViz/Foxglove 는 개발자 디버깅용으로 남긴다.

## 2. 전체 구조

```
[Raspberry Pi]                         [서버 — docker compose]
 systemd: tb3-bringup.service           zenoh-bridge (기존)
  └ full_bringup.launch.py               remote-pc 이미지 기반 서비스들
    (모터·라이다·IMU·카메라)               ├ web-api   : FastAPI + rclpy (이 문서의 핵심)
 systemd: zenoh-bridge (기존)               │   ├ ROS 측: 토픽 구독/발행, 서비스, Nav2 액션
        │                                   │   ├ 모드 관리자: SLAM/주행 launch 를 켜고 끔
        └──── Tailscale (Zenoh) ────────────┤   └ HTTP/WebSocket API + 웹 정적 파일 제공
                                            └ (개발용) foxglove_bridge → Foxglove 앱
                                                         ▲
                              [브라우저] React 웹앱 ──────┘  http://<서버>:8080
```

핵심 결정 세 가지.

1. **브라우저는 ROS 를 모른다.** 웹앱은 `/api/...` 와 WebSocket 하나만 쓴다. ROS 메시지 해석·TF 계산·
   액션 처리는 전부 서버의 `web-api` 가 한다. → 프론트엔드가 ROS 버전·메시지 타입 변화에 영향받지 않고,
   나중에 Electron/모바일 앱을 붙여도 같은 API 를 쓴다.
2. **launch 를 켜고 끄는 주체가 사람이 아니라 `web-api` 의 모드 관리자다.** "지도 만들기" 버튼 =
   `system.launch.py use_slam:=true`, "주행" 버튼 = `use_slam:=false map:=<선택한 지도>`.
3. **로봇(Pi)은 항상 켜져 있는 센서 서버다.** 모드와 무관하게 bringup 은 systemd 로 상시 실행.
   모드 전환은 서버 쪽 launch 만 갈아끼운다 (기존 역할 분리 원칙 그대로).

## 3. 라이브러리·프레임워크 선택

| 영역 | 선택 | 검토한 대안 | 선택 이유 |
|---|---|---|---|
| ROS ↔ 웹 연결 방식 | **자체 백엔드 API** (FastAPI + rclpy) | foxglove_bridge / rosbridge_suite 를 브라우저가 직접 사용 | 아래 3.1 |
| 백엔드 프레임워크 | **FastAPI** (+ uvicorn) | Flask, Node.js + rclnodejs | rclpy 와 같은 Python. 비동기·WebSocket 기본 지원, Pydantic 으로 API 스키마 자동 문서화(`/docs`) |
| 백엔드 ROS 클라이언트 | **rclpy** | rclnodejs | 이미 my_vision 이 rclpy. Nav2 액션(ActionClient)·TF2(Buffer) 를 공식 API 로 바로 사용 |
| 프론트엔드 | **React + TypeScript + Vite** | Vue, Svelte | 로봇 웹 UI 레퍼런스(rmf-web, Foxglove 패널, transAct)가 React. 채용 시장에서도 가장 범용 |
| 지도 그리기 | **react-konva** (Canvas) | ros2djs / nav2djs, three.js | ros2djs 계열은 roslibjs 전제·유지보수 불분명. 지도는 백엔드가 PNG 로 주므로 2D 캔버스 라이브러리면 충분 — 레이어(지도·경로·로봇·마커), 확대/이동, 클릭 이벤트가 필요해 Konva |
| UI 컴포넌트 | **MUI** | Mantine, shadcn/ui | 기업용 대시보드에 가장 널리 쓰임, 컴포넌트가 많아 버튼·대화상자·표 구현 시간 최소 |
| 서버 상태 조회 | **TanStack Query** | 직접 fetch | REST 캐싱·재시도·주기 갱신을 선언적으로 |
| 실시간 데이터 | **브라우저 기본 WebSocket** (JSON) | Socket.IO | 의존성 없음. 메시지량이 작음(상태 5Hz·위치 10Hz) |
| 카메라 영상 | **MJPEG over HTTP** (`multipart/x-mixed-replace`) | WebRTC, WebSocket 바이너리 | 이미 JPEG 로 압축돼 오므로 재인코딩 없이 그대로 흘리면 됨. `<img src>` 한 줄로 표시. 6fps·0.2MB/s 라 WebRTC 의 이점 없음 |
| 서버 자동 실행 | **docker compose** `restart: always` | — | 기존 구성의 연장 |
| Pi 자동 실행 | **직접 쓴 systemd 유닛** | robot_upstart | 유닛 하나라 도구 의존 없이 투명하게. 종료 신호를 SIGINT 로 지정해야 함 (6장) |
| 개발자 시각화 | **foxglove_bridge + Foxglove 앱** | RViz over VNC | VNC 없이 맥에서 바로 연결. 제품 UI 와 분리된 디버깅 경로 |
| 데스크톱 앱 (선택) | 같은 웹 코드를 **Tauri** 또는 Electron 으로 포장 | — | 웹이 먼저. 설치형이 필요해질 때만 |

### 3.1 왜 브리지를 브라우저에 직접 붙이지 않나

브리지 방식은 RViz 처럼 "모든 토픽을 보는 도구"에는 좋지만 제품 UI 의 기반으로는 약점이 있다.

- **foxglove_bridge**: 바이너리 WebSocket 이라 빠르고 Foxglove 가 새 구성에 권장한다. 하지만 브라우저 쪽
  JS 클라이언트(`@foxglove/ws-protocol`)는 Foxglove 문서상 더 이상 활발히 개발되지 않고, ROS 2 메시지(CDR)를
  직접 디코딩해야 하며, Nav2 액션은 숨김 토픽 노출(`include_hidden:=true`)을 켜야 보인다.
- **rosbridge_suite + roslibjs**: 자료는 가장 많지만 JSON 이라 이미지·대용량에서 유실 보고가 있고,
  roslibjs 의 ROS 2 액션 지원은 정식 릴리스 여부가 불분명하다.
- 두 방식 모두 **TF 계산(로봇이 지도 어디에 있나)을 브라우저가 해야** 하고, 화면이 ROS 토픽 이름·타입에
  직접 묶인다.

자체 API 를 두면 위 문제가 사라지고, "로봇 기능 = API" 로 정리돼 문서화·테스트가 쉽다. 대신 API 를 직접
설계·구현하는 비용이 든다 — 포트폴리오에선 이 부분이 오히려 보여줄 거리다.

## 4. 운용 모드 (모드 관리자)

```
            ┌──────────── 지도 저장 ────────────┐
            ▼                                   │
  ┌──────┐  지도 만들기   ┌──────────┐          │
  │ IDLE │ ────────────▶ │ MAPPING  │ ─────────┘
  │      │ ◀──── 정지 ── │ SLAM+Nav2│
  │      │               └──────────┘
  │      │  주행(지도 선택) ┌────────────┐
  │      │ ─────────────▶ │ NAVIGATION │
  │      │ ◀──── 정지 ─── │ AMCL+Nav2  │
  └──────┘                └────────────┘
```

| 모드 | 실행하는 것 | 화면에서 할 수 있는 것 |
|---|---|---|
| IDLE | 없음 (web-api 만) | 로봇 상태·배터리·카메라 보기, 조이스틱 |
| MAPPING | `system.launch.py use_slam:=true rviz:=false` | 실시간 지도, 클릭 목표로 탐사, 조이스틱, **지도 저장** |
| NAVIGATION | `system.launch.py use_slam:=false map:=<yaml> rviz:=false` | 초기 위치 지정, 클릭 목표 주행, 검출 물체 보기 |

- 모드 전환 = 현재 launch 의 **프로세스 그룹 전체에 SIGINT** → 최대 15초 대기 → 남으면 SIGTERM → SIGKILL,
  그다음 새 launch. (2026-11-05 에 겪은 "백그라운드 launch 가 SIGINT 를 무시하는" 함정을 피하려고
  `start_new_session=True` 로 띄우고 그룹 단위로 신호를 보낸다 — troubleshooting 2026-11-05.)
- 전환 중엔 Nav2 목표를 먼저 취소하고, 전환이 끝날 때까지 다른 모드 요청은 거절한다.
- Vision 은 두 모드 모두에서 켠다 (`vision:=true`, target_frame=map).

## 5. API 설계 (초안)

### 5.1 REST

| 메서드·경로 | 설명 | ROS 쪽 구현 |
|---|---|---|
| `GET /api/status` | 모드, 배터리 전압, 연결 상태(토픽 수신 여부), Nav 상태, 카메라 fps | 각 토픽 마지막 수신 시각·값 |
| `POST /api/mode` `{mode, map?}` | 모드 전환 | 모드 관리자 |
| `GET /api/map` | 현재 지도 PNG + 메타(해상도, 원점, 크기) | `/map` (OccupancyGrid) → PNG 변환, 갱신 시 버전 증가 |
| `GET /api/maps` · `POST /api/maps {name}` · `DELETE /api/maps/{name}` | 저장 지도 목록·저장·삭제 | slam_toolbox 지도 저장 서비스 → `/overlay_ws/maps/<name>.yaml/.pgm` |
| `POST /api/nav/goal {x, y, yaw}` | 지도 좌표로 이동 | Nav2 `NavigateToPose` 액션 |
| `POST /api/nav/cancel` | 이동 취소 | 액션 cancel |
| `POST /api/nav/initial_pose {x, y, yaw}` | AMCL 초기 위치 | `/initialpose` 발행 |
| `GET /api/camera.mjpg` · `GET /api/vision.mjpg` | 카메라 / 검출 결과 영상 | `/camera/color/compressed`, `/vision/annotated/compressed` 의 JPEG 를 그대로 전달 |

### 5.2 WebSocket `/ws`

서버 → 브라우저 (JSON, `type` 필드로 구분)

| type | 주기 | 내용 | ROS 원천 |
|---|---|---|---|
| `status` | 2Hz | `/api/status` 와 같은 내용 | — |
| `pose` | 10Hz | 지도 좌표의 로봇 위치·방향 | TF `map → base_footprint` |
| `nav` | 이벤트+1Hz | 진행 상태, 남은 거리, 결과 | 액션 feedback/result |
| `path` | 갱신 시 | 전역 경로 점 목록 (간략화) | `/plan` |
| `objects` | 갱신 시 | 검출 물체 이름·점수·지도 좌표 | `/vision/objects` |
| `map_updated` | 갱신 시 | 지도 버전 → 브라우저가 `GET /api/map` 재요청 | `/map` |

브라우저 → 서버

| type | 내용 | 안전 장치 |
|---|---|---|
| `teleop` | `{vx, wz}` 10Hz | **0.3초 동안 안 오면 서버가 정지 명령(0,0) 발행**, WebSocket 끊기면 즉시 정지, 속도 상한 클램프 |

## 6. 자동 실행

**Pi** — `/etc/systemd/system/tb3-bringup.service` (설치 명령은 사용자가 sudo 로 — 레포에 유닛 파일만 둔다)

```ini
[Unit]
Description=TurtleBot3 bringup + RealSense
After=network-online.target zenoh-bridge.service

[Service]
User=michael
ExecStart=/bin/bash -lc 'source ~/turtlebot3_ws/install/setup.bash && source ~/realsense_ros_ws/install/setup.bash && ros2 launch realsense_bringup full_bringup.launch.py'
Environment=ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-01
KillSignal=SIGINT          # ros2 launch 의 정상 종료 경로 (기본 SIGTERM 은 노드를 고아로 남길 수 있음)
TimeoutStopSec=30          # 카메라가 시작 중이면 정상 시작을 기다린 뒤 종료되도록 넉넉히
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**서버** — compose 에 `web-api` 서비스 추가 (`remote-pc` 이미지 재사용, `restart: always`, 포트 8080).
프론트엔드는 빌드 결과(정적 파일)를 web-api 가 함께 제공 → 컨테이너 하나로 끝.

## 7. 레포 구조 (추가분)

```
remote_pc/src/my_web_api/        # ROS 2 ament_python 패키지 — FastAPI + rclpy 백엔드
├── my_web_api/
│   ├── app.py                  # FastAPI 앱, 라우트
│   ├── ros_bridge.py           # rclpy 노드 (별도 스레드 executor): 구독·발행·액션·TF
│   ├── mode_manager.py         # launch 프로세스 그룹 관리 (상태 머신)
│   ├── map_codec.py            # OccupancyGrid → PNG
│   └── teleop.py               # 워치독 포함 cmd_vel 발행
├── config/web_api.yaml         # 포트, 지도 폴더, 속도 상한, 토픽 이름
└── test/                       # mode_manager·map_codec 단위 테스트
web/                            # React 프론트엔드 (ROS 패키지 아님, npm 프로젝트)
├── src/{api,components,pages}/
└── package.json
robot/systemd/tb3-bringup.service
```

## 8. 단계별 계획

| 단계 | 내용 | 완료 기준 |
|---|---|---|
| **P0 자동 실행** | Pi systemd 유닛, compose 에 web-api 자리 | Pi 재부팅 후 서버에서 `/scan` `/odom` `/camera/*` 수신, `systemctl stop` 시 노드 전부 정상 종료 |
| **P1 상태·영상** | web-api 골격, `/api/status`, 카메라 MJPEG, 최소 화면 | 브라우저에서 배터리 전압·연결 상태·카메라 영상 확인 |
| **P2 지도·주행** | 지도 PNG, 로봇 위치, 클릭 목표·취소, 경로 표시 (모드는 수동 launch 상태에서) | 지도 클릭으로 로봇이 이동, 진행 상태 표시 |
| **P3 모드 관리** | IDLE/MAPPING/NAVIGATION 전환, 지도 저장·선택 | 터미널 없이 지도 만들기 → 저장 → 그 지도로 주행 |
| **P4 조작·인식** | 조이스틱(워치독), 검출 물체 목록·지도 표시, "물체로 이동" | 검출 목록에서 물체를 눌러 그 앞으로 이동 (기존 로드맵의 데모와 합류) |
| P5 (선택) | Tauri/Electron 포장, 간단한 인증 | 설치형 앱에서 같은 화면 |

각 단계는 실기(로봇)로 검증하고 `docs/troubleshooting.md`·패키지 문서를 같이 갱신한다.

## 9. 미결·구현 중 확인할 것

- slam_toolbox 의 지도 저장 서비스 이름·형식 (`/slam_toolbox/save_map` 또는 nav2 `map_saver`) — 실기 확인.
- rclpy executor 를 별도 스레드로 돌리고 FastAPI(asyncio)와 데이터를 주고받는 방식 — 스레드 안전 큐로 시작.
- 모드 전환 시 Nav2 lifecycle 재시작 시간 (수 초 예상) — 화면에 "전환 중" 표시.
- 보안: 현재는 Tailscale 내부 접속만 전제 (인증 없음). 외부 공개 시 인증 필요.
- 다수 로봇·관제는 범위 밖 (필요해지면 Open-RMF / rmf-web 검토).

## 10. 조사 근거

- 브리지: [Foxglove Bridge 문서](https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge) (새 구성엔 rosbridge 대신 권장),
  [ws-protocol](https://github.com/foxglove/ws-protocol) · [Foxglove 커스텀 연결 문서](https://docs.foxglove.dev/docs/getting-started/custom) (JS 클라이언트 개발 중단, CDR 은 직접 디코딩),
  [foxglove-ros-adapter README](https://cdn.jsdelivr.net/npm/foxglove-ros-adapter@0.5.1/README.md) (액션은 `include_hidden` 필요),
  [ROS Discourse — rosbridge 고속 데이터 유실](https://discourse.openrobotics.org/t/connect-reliably-to-ros-1-and-2-stacks-with-high-rate-data-and-large-messages/29700),
  [rosbridge_suite](https://index.ros.org/p/rosbridge_suite/) · [roslibjs ROS 2 액션 이슈](https://github-redirect.dependabot.com/RobotWebTools/roslibjs/issues/646)
- 제품 사례: [MiR 웹 인터페이스](https://janjericevic.github.io/MiRco_docs/MiR100/mir_web_interface), [MiR600 사용자 가이드](https://allied-automation.com/mir600-user-guide-10_en) (브라우저 접속·REST API)
- 웹 대시보드: [rmf-web](https://github.com/open-rmf/rmf-web) (React), [transAct](https://discourse.openrobotics.org/t/announcing-transact-an-open-source-robot-fleet-management-dashboard-template/41705),
  [Foxglove 커스텀 패널](https://docs.foxglove.dev/docs/visualization/extensions/guides/create-custom-panel)
- 지도 위젯: [nav2djs](https://github.com/GT-RAIL/nav2djs), [ros2djs](https://gittrend.io/repo/RobotWebTools/ros2djs) (roslibjs 전제, 유지보수 불분명)
- 자동 실행: [Clearpath ROS 2 서비스 구성](https://docs.clearpathrobotics.com/docs/ros2humble/ros/config/services), [robot_upstart](https://docs.ros.org/en/jazzy/p/robot_upstart/doc/index.html)
- 클라우드 관제(범위 밖 참고): [InOrbit](https://inorbit.ai/faq), [Transitive Robotics](https://transitiverobotics.com/caps/)
