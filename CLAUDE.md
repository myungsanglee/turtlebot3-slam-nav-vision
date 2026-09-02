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

## 2. 시스템 아키텍처 (2대 + Zenoh Bridge)

물리적으로 떨어진 두 머신을 Tailscale(WireGuard VPN)로 연결한다.

```
[Raspberry Pi @ TurtleBot3]        [Remote PC / 회사 서버]
  - turtlebot3_bringup (고정)         - Docker: ROS 2 Humble 컨테이너
  - RealSense D435i (추가)            - SLAM / Nav2 / Vision AI (개발 대상)
  - zenoh-bridge (systemd)           - zenoh-bridge (컨테이너, tcp/7447 listen)
  - Tailscale                        - Tailscale, NVIDIA RTX A6000
        └──────── Tailscale (Zenoh Bridge, tcp/7447) ────────┘
```

- **역할 분리**: Pi = 센서 publish 전용(엣지), Remote PC = 무거운 연산 전부.
- **bringup 은 변경하지 않는다.** 모터 제어/오도메트리 등 로봇 기본은 ROBOTIS
  `turtlebot3_bringup` 을 그대로 사용. 로봇 쪽에 새로 추가한 것은 **RealSense
  카메라 publish(realsense_bringup) 와 zenoh-bridge(systemd) 뿐**.
- Remote PC 는 그 토픽들(`/scan`, `/odom`, `/imu`, `/camera/*`)을 받아
  **나만의 SLAM/Nav/Vision** 을 돌린다.

## 3. 네트워크 / 통신 (중요 — 2026-09 Zenoh 전환으로 확정된 구성)

집-회사처럼 다른 망이라 DDS 멀티캐스트가 안 된다. 초기의 **Fast DDS Discovery
Server** 방식은 라이다까진 됐지만 **카메라(토픽 20+개 복잡 참가자)의 데이터
전달이 VPN+멀티홈에서 실패**해 폐기하고 **Zenoh Bridge** 로 전환했다
(진단 여정: docs/troubleshooting.md 2026-08-29, 09-01 항목).

**현재 아키텍처 (zenoh-bridge-ros2dds, 양쪽 버전 동일 필수 — 현재 1.10.0)**
- 각 호스트 내부: `ROS_LOCALHOST_ONLY=1` → DDS 는 loopback 만 (RMW 는 그대로
  `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=30` 양쪽 동일)
- 호스트 간: zenoh 브리지가 유일한 통로 — 서버가 `tcp/7447` listen
  (compose 의 `zenoh-bridge` 서비스), Pi 가 접속 (`robot/config/zenoh-bridge-pi.json5`,
  systemd 서비스 `zenoh-bridge`)
- 브리징 토픽은 양쪽 config 의 **allow 리스트**로 제한. ★ 카메라는 **compressed 만**
  브리징 (raw 640x480 ≈ 14MB/s 는 Tailscale 초과, compressed ≈ 1MB/s)
- ROS_DISCOVERY_SERVER / 인터페이스 화이트리스트 / ROS_SUPER_CLIENT 전부 불필요
  (CLI 도 그냥 동작). 같은 LAN 테스트도 동일 구성으로 동작.

**운영 주의**: 정상 상태에선 시작 순서 무관 (브리지가 먼저 떠 있어도 나중에
시작한 노드가 붙는 것 실증됨). 단, env 를 갈아엎는 등 비정상 상황 후 "토픽은
보이는데 데이터 0" 증상이 나면 **브리지만 재시작**하면 복구된다
— Pi: `sudo systemctl restart zenoh-bridge`, 서버: `docker compose restart zenoh-bridge`.

주소(실제 값):
- 서버 Tailscale IP: `100.95.193.1` (UFW 는 tailscale0 전체 허용 규칙 필요)
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
| 원격 통신 | **zenoh-bridge-ros2dds** | Fast DDS Discovery Server 의 VPN 한계로 전환 (3번) |
| 카메라 | **realsense2_camera** (RSUSB 소스 빌드) | D435i. Pi 커널 미패치로 apt 판 불가 (troubleshooting 08-27) |
| Vision AI | 본인 파이프라인 (RF-DETR/RTMDet + TensorRT) | ROS2 노드로 래핑, `/camera` 구독→추론→publish |
| 시각화 | RViz2 (+ 추후 Foxglove) | SLAM+Nav+영상 통합 .rviz 한 창 |

## 5. 커스텀 로봇 — 반드시 반영할 것 (표준 데모 그대로 못 씀)

이 TurtleBot3 는 ROBOTIS 표준과 형태가 다르다:

- **바퀴 폭(wheel separation): 표준과 동일** → 휠 오도메트리 파라미터는 그대로 OK.
- ✅ **LDS(라이다) 위치/높이 다름** → `base_link → base_scan` 실측 보정 **완료**
  (xyz=-0.100,0,0.125 — docs/description.md). IMU 회전(yaw=-1.57)도 반영,
  IMU 위치 xyz 는 미실측(EKF 전 교체).
- ⏳ **RealSense 추가** → `base_link → camera_link` static transform 을 실측값으로
  URDF 에 추가 (다음 우선 작업).
- ⏳ Nav2 풋프린트: `robot_radius`(임시 0.105) 대신 실제 외형 실측 다각형
  `footprint` 로 교체.

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
│   ├── my_navigation.md
│   ├── description.md
│   ├── troubleshooting.md     # 비자명한 문제의 진단·해결 기록 (인시던트 로그)
│   ├── pi_setup.md            # 새 Pi(로봇) 환경 재구축 가이드
│   └── server_setup.md        # 새 Remote PC(서버) 환경 재구축 가이드
├── docker/                    # Remote PC 컨테이너 (ROS2 Humble)
│   ├── Dockerfile
│   └── entrypoint.sh
├── docker-compose.yml         # zenoh-bridge + remote-pc 서비스
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

> 참고: 컨테이너의 colcon 워크스페이스는 `./remote_pc` → `/overlay_ws` 마운트.
> Pi 쪽 `realsense_bringup` 은 Pi 의 `~/realsense_ros_ws/src` 에 심볼릭 링크로
> 연결되어 빌드된다 (docs/pi_setup.md 5단계).

## 7. 개발 워크플로

- **편집**: 맥북에서 VS Code Remote-SSH 로 Remote PC 에 접속(Tailscale). Claude Code 도 여기서.
- **빌드/실행**: Remote PC 의 Docker 컨테이너 안 (`docker compose exec remote-pc bash`)
- **Pi 원격 작업**: 서버에서 `ssh michael@100.71.74.81` (서버 공개키 등록됨) —
  Claude 도 이 경로로 Pi 진단·작업 가능. ⚠️ 원격 `pkill -f` 는 자기 SSH 명령과
  패턴이 매치되지 않게 bracket 트릭 사용 (troubleshooting 2026-09-01(2))
- **RViz 확인**: 서버 물리 세션을 x11vnc 로 미러링, 맥에서 RealVNC/화면공유로 접속
  (컨테이너 RViz 는 `DISPLAY` 를 서버 물리 세션 `:0`/`:1` 에 맞춰 실행)
- **환경 자동 source**: 컨테이너는 entrypoint(run 용) + .bashrc(exec 용) 양쪽에서
  ROS 환경을 자동 source 하도록 이미 구성됨.

## 8. 현재 진행 상황 / 다음 할 일

**완료**
- Docker ROS2 Humble 컨테이너 (osrf 베이스 + ROBOTIS 소스 3종 + SLAM/Nav2)
- x11vnc 로 RViz 원격 확인 환경 구축
- 레포 구조 정리 (위 6번 구조 스캐폴딩, README / .gitignore)
- **my_slam 패키지** — slam_toolbox(online async) 설정/런치/RViz 구성.
  실물 로봇으로 엔드투엔드 검증 완료(/scan 5Hz, map→odom TF, /map 생성).
  상세는 `docs/my_slam.md`
- **my_navigation 패키지** — Nav2 설정/런치/RViz. 기본 모드는 SLAM 동시 실행
  (지도 만들며 주행), 저장 지도+AMCL 모드 지원. 전체 lifecycle 활성화·코스트맵
  publish 실기 검증 완료(실주행 테스트는 사용자 입회 하 예정). footprint 는
  표준 burger 임시값(★실측 교체 필요). 상세는 `docs/my_navigation.md`
- **커스텀 로봇 URDF 보정** — description/urdf/turtlebot3_burger.urdf 에 실측 반영:
  scan_joint(LDS) xyz=-0.100,0,0.125 / imu_joint yaw=-1.57(OpenCR 90° 회전).
  Pi 배포 + TF 실기 검증 완료. 제자리 회전 정밀 검증은 공간 확보 시 예정.
  IMU 위치 xyz 는 미실측(표준값 유지, EKF 전 교체). 상세는 `docs/description.md`
- **realsense_bringup (Pi)** — D435i 를 RSUSB 백엔드 realsense-ros(소스 빌드)로
  구동, color/depth 640x480x15 + compressed 퍼블리시 (docs/troubleshooting.md
  2026-08-27 참고). ※ docs/realsense_bringup.md 작성 예정
- **통신 아키텍처 Zenoh 전환** — Discovery Server 폐기, zenoh-bridge-ros2dds 로
  전환 (위 3번). 엔드투엔드 검증 완료: /scan 5Hz, /odom 20Hz, 카메라 compressed
  15fps(≈1MB/s), tf/camera_info 정상. Pi 브리지 systemd 서비스 등록.

**다음 (우선순위 순)**
1. **base_link→camera_link TF 추가** — 카메라 장착 위치 실측 → URDF 반영 (5번 규칙)
2. **Nav2 footprint 실측 교체** — nav2_params.yaml 의 robot_radius(임시 0.105) →
   실측 다각형 footprint
3. **Pi 전원 보강** — OpenCR 5V 출력이 Pi4+D435i 에 한계(undervoltage 재발,
   troubleshooting 2026-09-01(2)). 부품 결정됨: 배터리(T-plug)→5V/5A 컨버터
   (Pololu D24V50F5 또는 UBEC) → Pi 직결. 개발 중엔 벽 어댑터로 대체 가능
4. SLAM 실주행 정밀 검증 (제자리 회전 벽 이중선) — 공간 확보 시
5. robot_localization EKF 설정 (LiDAR+IMU+엔코더 융합) — 전에 IMU 위치 실측
6. SLAM + Nav2 + RealSense + RViz 통합 런치 (한 창에서 다 보기)
7. Vision AI 노드 (TensorRT 추론) 통합 — compressed 구독→디코드→추론
8. docs/realsense_bringup.md 작성 (9번 규칙)

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
- **비자명한 문제를 해결하면 `docs/troubleshooting.md` 에 기록한다.**
  "증상 → 진단 과정 → 근본 원인 → 해결 → 재발 방지" 형식. 특히 git 에 안 남는
  설정(방화벽·로봇 하드웨어 등)은 여기가 유일한 기록이므로 반드시 남긴다.
  (컴포넌트 "정상 동작 방식"은 docs/<패키지>.md, 문제 해결 "여정"은 여기)
- **개발·변경이 끝나면 `README.md` 를 같은 작업에서 갱신한다** — 컴포넌트 개발뿐
  아니라 아키텍처·스택·실행 절차·진행 상태가 바뀌는 모든 변경이 트리거다.
  점검할 섹션: ① 아키텍처 다이어그램 ② 기술 스택 표 ③ 레포 구조 ④ "개발한 내용"
  표(상태 포함) ⑤ 시작하기(실행 명령). (docs 는 상세, README 는 개요·실행법 요약)
- **아키텍처급 변경(통신 방식·스택 교체 등) 후에는 문서 전체 잔재 스윕을 한다.**
  폐기된 개념의 키워드로 `CLAUDE.md·README·docs/` 전체를 grep 해서 낡은 서술을
  일괄 갱신한다 (CLAUDE.md 자신도 대상). 교훈: Zenoh 전환 때 README 다이어그램·
  CLAUDE.md 2번 제목 등이 한동안 구버전(Discovery Server)으로 남아 있었다 —
  변경 순간에 스윕했으면 없었을 드리프트.
- **환경·실행 절차가 바뀌면 `docs/pi_setup.md`(로봇)·`docs/server_setup.md`(서버)를
  같은 작업에서 함께 갱신한다.** 해당하는 변경의 예: 의존성 추가/제거(apt·소스
  빌드), 환경변수(.bashrc·compose env), 버전 고정값(예: zenoh-bridge 버전),
  systemd/compose 서비스 구성, 워크스페이스 구조, 하드웨어/배선 절차.
  판단 기준: **"이 문서만 보고 새 머신을 재구축했을 때 지금과 동일한 상태가
  나오는가"** — 아니라면 문서가 뒤처진 것이므로 갱신한다.
- 응답/주석은 한국어 기본.
