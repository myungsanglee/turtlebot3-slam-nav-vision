# turtlebot3-slam-nav-vision

ROBOTIS TurtleBot3 실물 로봇으로 SLAM · Navigation · Vision AI를 구현하는 개인 포트폴리오 프로젝트입니다.
Raspberry Pi(로봇)와 Remote PC(연산 서버)를 Tailscale로 연결해 역할을 분리하고,
Remote PC의 Docker(ROS 2 Humble) 컨테이너에서 SLAM/Nav2/Vision AI를 개발합니다.

자세한 아키텍처, 네트워크 구성, 기술 스택, 진행 상황은 [CLAUDE.md](./CLAUDE.md)를 참고하세요.

## 레포 구조

```
turtlebot3-slam-nav-vision/
├── docs/                      # 컴포넌트별 상세 문서 (my_slam.md 등)
├── docker/                    # Remote PC 컨테이너 (ROS2 Humble)
├── docker-compose.yml         # discovery-server + remote-pc 서비스
├── config/                    # 파라미터, .rviz, EKF 설정 등
├── remote_pc/src/              # Remote PC 패키지 (my_slam, my_navigation, my_vision)
├── robot/src/                  # Raspberry Pi 패키지 (realsense_bringup)
└── description/                # URDF/xacro (커스텀 로봇 TF 보정)
```

## 시작하기

```bash
docker compose up -d
docker compose exec remote-pc bash
```
