# Remote PC(서버) 환경 구축 가이드

> 새 서버에서 이 프로젝트의 연산 쪽 환경을 재구축하는 절차.
> 서버는 Pi 와 달리 **Docker 가 환경 재현을 담당**하므로 절차가 짧다 —
> 호스트에 필요한 건 Docker·GPU 드라이버·Tailscale·방화벽 규칙 정도이고,
> ROS/SLAM/Nav2 등은 전부 컨테이너 이미지(docker/Dockerfile)가 구축한다.

## 구성 요약

```
[호스트: Ubuntu 24.04]
  Docker + NVIDIA Container Toolkit + Tailscale + UFW 규칙
  └─ docker compose
       ├─ zenoh-bridge  : Pi<->서버 통신 (tcp/7447 listen, eclipse/zenoh-bridge-ros2dds)
       └─ remote-pc     : ROS 2 Humble 컨테이너 (SLAM/Nav2/RViz, /overlay_ws=./remote_pc)
```

## 0. 전제 (호스트)

- Ubuntu 24.04 (다른 버전도 무방 — 컨테이너가 22.04/Humble 을 제공)
- NVIDIA 드라이버 + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) (RViz GPU 가속·추후 비전 AI)
- Docker Engine + docker compose v2
- Tailscale 설치·로그인 — Pi 와 **같은 tailnet** 필수. 서버 IP 예: `100.95.193.1`

## 1. 방화벽 (★ git 에 안 남는 설정 — 잊기 쉬움)

UFW 를 쓰는 서버라면 tailscale 인터페이스를 통째로 허용해야 한다.
없으면 "토픽은 보이는데 데이터 0" 증상 (troubleshooting 2026-08-25 참고):

```bash
sudo ufw allow in on tailscale0 comment 'ROS2 / Tailscale trusted overlay'
```

zenoh 의 tcp/7447 도 이 규칙으로 커버된다 (LAN 에서 오는 7447 은 기본 deny —
tailnet 을 통해서만 접근 가능한 것이 의도된 동작).

## 2. 레포 clone + 컨테이너 빌드/기동

```bash
git clone <레포 URL> turtlebot3-slam-nav-vision
cd turtlebot3-slam-nav-vision

docker compose build          # 최초 1회: tb3-remote-pc:overlay 이미지 빌드 (오래 걸림)
docker compose up -d          # zenoh-bridge + remote-pc 기동
docker compose ps             # 두 서비스 Up 확인
```

- 서버 Tailscale IP 가 다르면 갱신할 곳: `robot/config/zenoh-bridge-pi.json5` 의
  connect 주소 (Pi 쪽에 배포되는 파일).
- zenoh-bridge 이미지 태그(현재 1.10.0)는 **Pi 바이너리 버전과 동일**해야 한다.

## 3. 오버레이 워크스페이스 빌드 (컨테이너 안)

`./remote_pc` 가 컨테이너의 `/overlay_ws` 로 마운트된다. 우리 패키지 빌드:

```bash
docker compose exec remote-pc bash
colcon build --symlink-install        # /overlay_ws 에서 (my_slam, my_navigation 등)
exit && docker compose exec remote-pc bash   # 재진입하면 자동 source 됨
```

## 4. RViz 원격 확인 (VNC)

컨테이너 RViz 는 호스트 X 서버(`/tmp/.X11-unix` 마운트)에 표시된다.
서버 물리 세션을 x11vnc 로 미러링해 맥/노트북에서 VNC 로 접속하는 구성:

```bash
# 호스트에서 (물리 세션 :0 또는 :1 미러)
x11vnc -display :1 -forever -shared
# 컨테이너에서 RViz 실행 시
export DISPLAY=:1
```

## 5. 최종 검증

Pi 쪽(bringup·카메라·브리지)이 떠 있는 상태에서, 컨테이너 안:

```bash
ros2 topic list                                            # 브리지 allow 토픽들이 보임
ros2 topic hz /scan                                        # ~5Hz
ros2 topic hz /camera/camera/color/image_raw/compressed    # ~15fps (≈1MB/s)
ros2 launch my_slam slam.launch.py                         # SLAM + RViz
```

## 운영 명령 모음

```bash
docker compose ps                       # 서비스 상태
docker compose logs zenoh-bridge -f     # 브리지 로그 (라우트/원격 브리지 감지)
docker compose restart zenoh-bridge     # "토픽 보이는데 데이터 0" 복구
docker compose exec remote-pc bash      # 작업 셸 진입
ros2 daemon stop && ros2 daemon start   # topic list 에 유령 토픽 보일 때 (캐시 새로고침)
```

## 참고

- 컨테이너 이미지 상세(설치 방침·자동 source 구조)는 `docker/Dockerfile` 주석 참고.
- 통신 아키텍처 전체와 전환 배경: CLAUDE.md 3번, [troubleshooting.md](./troubleshooting.md).
- 로봇 쪽 구축: [pi_setup.md](./pi_setup.md).
