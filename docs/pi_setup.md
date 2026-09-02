# Raspberry Pi(로봇) 환경 구축 가이드

> 새 Pi 에서 이 프로젝트의 로봇 쪽 환경을 처음부터 재구축하는 절차.
> 이 레포는 "우리 코드"만 담고 있으므로, clone 만으로는 동작하지 않는다 —
> 아래 순서대로 서드파티 환경(ROS·librealsense·realsense-ros·zenoh)을 구축해야 한다.
> "왜 이렇게 하는지"의 배경은 [troubleshooting.md](./troubleshooting.md)
> (2026-08-27 RSUSB, 2026-08-29~09-01 Zenoh) 참고.

## 구성 요약

```
/opt/ros/humble            ← ROS 2 Humble (apt)
~/turtlebot3_ws            ← ROBOTIS 패키지 (bringup 등, 소스 빌드)
/usr/local                 ← librealsense (RSUSB 백엔드, 소스 빌드)
~/realsense_ros_ws         ← realsense-ros (소스 빌드) + realsense_bringup(심링크)
~/zenoh-bridge             ← zenoh-bridge-ros2dds 바이너리 (+ systemd 서비스)
~/turtlebot3-slam-nav-vision  ← 이 레포 (git)
```

## 0. 전제

- Ubuntu 22.04 arm64 (Raspberry Pi 4 기준)
- Tailscale 설치·로그인 — 서버와 **같은 tailnet** 필수 (노드 공유로는 DDS 불가,
  troubleshooting 참고). 로봇 IP 예: `100.71.74.81`
- ⚠️ 전원: Pi4 + D435i 는 전력에 민감. `vcgencmd get_throttled` 이 `0x0` 인지
  수시 확인 (bit0 켜지면 카메라/USB 가 오동작 — 2026-09-01(2) 항목)

## 1. ROS 2 Humble + TurtleBot3 기본 셋업

ROBOTIS e-Manual 의 SBC Setup(Humble) 절차를 따른다:
- ROS 2 Humble 설치 (apt)
- `~/turtlebot3_ws` 에 ROBOTIS 패키지(DynamixelSDK, turtlebot3_msgs, turtlebot3) 소스 빌드
- OpenCR 펌웨어 셋업
- LDS(라이다), OpenCR 연결 확인 (`ros2 launch turtlebot3_bringup robot.launch.py`)

## 2. librealsense 소스 빌드 (RSUSB 백엔드)

apt 의 librealsense 는 커널 백엔드라 Pi 에서 실패한다 — **반드시 RSUSB 로 소스 빌드**.

```bash
sudo apt update && sudo apt install -y git wget cmake build-essential libssl-dev \
    libusb-1.0-0-dev libudev-dev pkg-config python3-dev python3-pip python3-numpy \
    v4l-utils libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev

cd ~ && git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo ./scripts/setup_udev_rules.sh          # ⚠️ 카메라 뽑은 상태에서

mkdir build && cd build
cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false \
          -DBUILD_PYTHON_BINDINGS=true
make -j4                                     # 15~20분
sudo make install                            # /usr/local 에 설치

# 동작 확인 (카메라 연결 후)
python3 -c "import sys; sys.path.append('/usr/local/lib'); import pyrealsense2 as rs; p=rs.pipeline(); p.start(); p.wait_for_frames(); print('OK')"
```

## 3. 이 레포 clone

```bash
cd ~ && git clone <레포 URL> turtlebot3-slam-nav-vision
```

## 4. realsense-ros 소스 빌드 (`~/realsense_ros_ws`)

apt 의 `ros-humble-realsense2-camera` 는 커널 백엔드 librealsense 에 링크되므로
설치돼 있으면 제거하고, 소스로 빌드해 `/usr/local` RSUSB 에 링크한다.

```bash
sudo apt remove -y "ros-humble-realsense2-*" ros-humble-librealsense2 2>/dev/null
sudo apt autoremove -y

mkdir -p ~/realsense_ros_ws/src && cd ~/realsense_ros_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development

cd ~/realsense_ros_ws
source /opt/ros/humble/setup.bash
# ★ --skip-keys 필수: 없으면 apt 커널 백엔드 librealsense 가 다시 깔림
rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
                          -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
```

압축 이미지 전송 플러그인도 설치 (원격 전송은 compressed 만 사용):
```bash
sudo apt install -y ros-humble-compressed-image-transport
```

## 5. 레포의 realsense_bringup 패키지 연결

```bash
ln -s ~/turtlebot3-slam-nav-vision/robot/src/realsense_bringup ~/realsense_ros_ws/src/realsense_bringup
cd ~/realsense_ros_ws
colcon build --packages-select realsense_bringup --symlink-install
```
`--symlink-install` 이라 이후 레포를 `git pull` 하면 런치 수정이 자동 반영된다
(새 파일 추가 시에만 이 빌드를 다시 실행).

## 6. zenoh-bridge 설치 + systemd 등록

버전은 **서버(docker-compose 의 이미지 태그)와 반드시 동일** — 현재 1.10.0.

```bash
cd ~ && mkdir -p zenoh-bridge && cd zenoh-bridge
wget https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.10.0/zenoh-plugin-ros2dds-1.10.0-aarch64-unknown-linux-gnu-standalone.zip
unzip zenoh-plugin-ros2dds-1.10.0-aarch64-unknown-linux-gnu-standalone.zip
chmod +x zenoh-bridge-ros2dds
```

systemd 유닛 `/etc/systemd/system/zenoh-bridge.service` (User/경로는 계정에 맞게):

```ini
[Unit]
Description=Zenoh bridge ROS2DDS (Pi <-> Server)
After=network-online.target
Wants=network-online.target

[Service]
User=michael
Environment=ROS_DISTRO=humble
Environment=ROS_LOCALHOST_ONLY=1
Environment=ROS_DOMAIN_ID=30
ExecStart=/home/michael/zenoh-bridge/zenoh-bridge-ros2dds -c /home/michael/turtlebot3-slam-nav-vision/robot/config/zenoh-bridge-pi.json5
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```bash
sudo cp zenoh-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now zenoh-bridge
journalctl -u zenoh-bridge -f     # "New ROS 2 bridge detected" 나오면 서버와 연결됨
```

브리징 토픽은 레포의 `robot/config/zenoh-bridge-pi.json5` allow 리스트로 관리
(서버 IP 변경 시 이 파일의 connect 주소도 갱신).

## 7. `~/.bashrc` 환경 블록

```bash
# ROS 2 기본
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/realsense_ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=1      # DDS 는 Pi 안(loopback)만 — 원격은 zenoh 브리지 담당
# librealsense (/usr/local 소스 설치)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export PYTHONPATH=$PYTHONPATH:/usr/local/lib
```

⚠️ 과거 방식의 잔재(`ROS_DISCOVERY_SERVER`, `FASTRTPS_DEFAULT_PROFILES_FILE`)가
있으면 반드시 제거 — 현 Zenoh 구성과 충돌한다.

## 8. 커스텀 URDF 배포

이 로봇은 센서 위치가 표준과 달라 보정 URDF 를 사용한다 (docs/description.md).

```bash
TARGET=~/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf
cp "$TARGET" "$TARGET.orig"      # 원본 백업
cp ~/turtlebot3-slam-nav-vision/description/urdf/turtlebot3_burger.urdf "$TARGET"
```

## 9. 최종 검증

```bash
# Pi (각각 새 셸)
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch realsense_bringup realsense.launch.py

# 서버 컨테이너에서
ros2 topic hz /scan                                        # ~5Hz
ros2 topic hz /camera/camera/color/image_raw/compressed    # ~15fps
ros2 run tf2_ros tf2_echo base_link base_scan              # -0.100, 0, 0.125 (보정 URDF 확인)
```

문제 발생 시: [troubleshooting.md](./troubleshooting.md) 빠른 참조 표부터.
