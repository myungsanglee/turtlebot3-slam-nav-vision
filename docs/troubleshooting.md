# 트러블슈팅 / 인시던트 로그

> 개발하다 겪은 **비자명한 문제**의 "증상 → 진단 과정 → 근본 원인 → 해결 →
> 재발 방지"를 시간순으로 남긴다. 목적은 두 가지:
>  1. 나중에 같은 문제를 다시 겪을 때 빠르게 복구 (특히 git 에 안 남는 설정 —
>     방화벽·로봇 하드웨어 등 — 은 여기가 유일한 기록).
>  2. 체계적으로 문제를 진단·해결한 과정 자체를 포트폴리오로 남김.
>
> 컴포넌트 "정상 동작 방식"은 각 `docs/<패키지>.md` 에, 각 변경의 what/why 는
> 커밋 메시지에, **"무엇이 왜 고장났고 어떻게 고쳤나"의 여정**은 여기에 남긴다.

## 빠른 참조 (자주 겪는 것)

| 증상 | 원인 | 즉효 |
|---|---|---|
| **[Zenoh 전환 후]** 토픽은 보이는데 데이터 수신 0 (드묾) | 브리지가 비정상 상태의 옛 노드들과 섞여 로컬 DDS 매칭이 꼬임 | **브리지 재시작** — Pi `sudo systemctl restart zenoh-bridge`, 서버 `docker compose restart zenoh-bridge` (2026-09-01 항목) |
| [구 Discovery Server 시절] `ros2 topic list` 에 원격 토픽이 안 보임 | CLI(daemon)가 일반 client → 매칭되는 것만 봄 | (역사적 — 현재 Zenoh 구성에선 불필요) `ROS_SUPER_CLIENT=TRUE` + daemon 재시작 |
| [구 Discovery Server 시절] 토픽 목록엔 보이는데 `ros2 topic hz` 수신율 0 | 디스커버리(메타데이터)는 되나 **유저 데이터 P2P 경로**가 막힘 (방화벽/멀티홈 locator) | 아래 2026-08-25 항목 참조 |
| RViz 에서 `/scan` 이 안 보임 | QoS 불일치 (`/scan` 은 BEST_EFFORT) | 구독 Reliability 를 Best Effort 로 |
| RViz 에서 `/map` 이 늦게/안 보임 | QoS Durability 불일치 | Transient Local 로 |
| bringup 이 `robot_description as yaml` 오류로 안 뜸 | URDF 주석에 콜론+공백/줄끝 콜론 → YAML 파싱 깨짐 | 주석에서 콜론 제거 (아래 2026-08-26 항목) |
| SSH 로 띄운 백그라운드 launch 가 `kill -INT` 로 안 꺼짐, 노드가 고아로 남음 | 비대화형 셸의 `&` 가 SIGINT 를 무시 상태로 물려줌 | 시작 스크립트에 `set -m`, 종료는 `kill -INT -<pgid>` (2026-11-05 항목) |
| 웹 백엔드 종료 시 `CancelledError` traceback / abort(-6) | uvicorn 의 종료 후 신호 재발생 + `ros2 launch` 의 중복 SIGINT | 신호를 main() 한 곳에서 처리, 1초 내 중복 무시, 스트림이 종료 이벤트로 스스로 끝남 (2026-11-10 항목) |
| 로봇이 정지해 있는데 odom 방향(웹 지도 화살표)이 저절로 돈다 | IMU 자이로 영점(바이어스) 틀어짐 — TurtleBot3 는 회전각을 IMU 로 계산 | 로봇 정지 상태로 Pi 에서 `ros2 service call /reset std_srvs/srv/Trigger` (2026-11-12 항목) |
| 통합 런치에서 어떤 노드가 자기 YAML 설정을 무시하고 기본값으로 돈다 | 하위 런치끼리 같은 이름의 인자(`params_file`)가 공유돼 다른 런치의 값이 들어감 | 하위 런치를 `GroupAction(scoped=True)` 로 감싸기, `ros2 param get` 으로 확인 (2026-11-12 항목) |
| 라이다는 돌고 odom·카메라는 오는데 `/scan` 만 끊김 (Pi 안에서도 0) | 미확정 — LDS 드라이버(`hlds_laser_publisher`)가 살아 있는데 발행 정지 | Pi 에서 시리얼 포트를 1초 읽으면 복구됨 → `scan-watchdog` 서비스가 자동 기록·복구 (2026-11-12 라이다 항목) |
| 라이다 수신이 끊기고 bringup 재시작으로도 안 돌아옴, 배터리 값이 한 값에 멈춤 | Pi 5V 저전압 → USB 포트 전체 리셋 → 장치가 ttyUSB1·ttyACM1 로 재연결, 라이다 드라이버 종료·OpenCR 통신 끊김 | 로봇 전원 껐다 켜기 (by-id 포트 적용 후엔 `systemctl --user restart tb3-bringup`). 전원 원인은 `~/.ros/scan_watchdog/power-*.log`·`events.log` 로 추적 (2026-11-12 저전압 항목) |
| 저장 지도로 주행할 때 벽 바깥(미탐사 회색)까지 빈 곳으로 취급됨 | nav2 `map_saver_cli` 기본 `free_thresh 0.25` → 미탐사 픽셀(205)이 불러올 때 0.196 < 0.25 로 '빈 칸' | 저장 시 `--free 0.196` (웹 저장은 적용됨), 옛 지도는 yaml 의 `free_thresh: 0.196` 으로 (2026-11-14 항목) |
| 클릭한 목표보다 20~25cm 앞에서 "도착" 하고 멈춤 | Nav2 도착 판정 반경 `xy_goal_tolerance` 기본 0.25 m | 0.07 로 (goal checker·DWB 둘 다) (2026-11-14 항목) |
| 종료한 노드의 토픽이 `topic list` 에 계속 보임 | ros2 daemon 의 그래프 캐시 (표시 문제일 뿐 실제 잔존 아님) | `ros2 daemon stop && ros2 daemon start` 또는 `ros2 topic list --no-daemon` |
| ROS 카메라 노드만 `UVCIOC_CTRL_QUERY Connection timed out` (pyrealsense2 는 됨) | ROS 래퍼가 apt 커널 백엔드 librealsense 사용, Pi 커널에 패치 없음 | realsense-ros 를 RSUSB 백엔드 librealsense 로 소스 빌드 (아래 2026-08-27 항목) |
| 카메라 토픽은 discover 되는데 데이터 수신 0 (라이다는 됨) | Fast DDS Discovery Server 가 VPN+멀티홈+복잡 참가자(카메라)에서 데이터 전달 실패 | Zenoh 전환 (아래 2026-08-29 항목) |
| 카메라 depth 는 뜨는데 RGB 만 실패 (`set_xu`/`control_transfer` EAGAIN) | Pi4 USB(VL805) 에서 RGB 컨트롤 경로(XU 캘리브레이션 읽기)가 세션 단위로 간헐 불통 | **자체 노드(rs_camera.launch.py)** 사용 — 캘리브레이션 캐시로 XU 독립 + 자기 복구 (2026-09-02, 09-03 항목) |
| 서버에서 토픽이 발행 fps 의 **정수배**(6→12Hz)로 수신, 검출 노드 정체 | 서버 브리지 재시작 뒤 Pi 브리지의 zenoh 세션이 이중으로 남아 샘플을 두 번 전달 | **Pi 브리지 재시작** — `kill $(pgrep -f "zenoh-bridge-ros2dd[s]")` (systemd 자동 재시작, sudo 불필요) (2026-09-08 항목) |
| RViz Image 디스플레이 "No Image" (`compressed_sub does not exist`) | 컨테이너에 image_transport compressed 플러그인 없음 | 이미지 재빌드 (Dockerfile 에 image-transport-plugins 포함됨, 2026-09-08) |
| RViz RobotModel 안 보이고 `Package [my_description] does not exist` | 메시 패키지가 overlay 에 있는데 셸이 `/opt/ros/humble` 만 source | `source /overlay_ws/install/setup.bash` (exec 셸은 .bashrc 가 자동. 최초엔 `colcon build` 필요, server_setup.md 3단계) |
| 카메라가 `lsusb` 에서 사라짐 (No such device) | 열거/start 도중인 카메라 프로세스를 kill 했거나 USB 리셋을 연타함 | 소프트웨어 복구 불가 — **물리 재연결**. 재발 방지는 2026-09-03 항목 |

---

## 2026-08-25 ~ 08-26 — 원격 Pi 에서 /scan 미수신 → SLAM 지도 안 그려짐

### 증상
Remote PC 에서 `my_slam` 을 실행해도 지도가 안 그려짐. `ros2 topic hz /scan`
수신율 0. "Remote PC 가 로봇 라이다 값을 못 받는" 상태.

### 환경·맥락
직전에 서버의 Tailscale 계정이 타인(adipark2)으로 로그인됐다가 본인(lms0577)
계정으로 복구된 이력. 서버 IP 는 원래 값 `100.95.193.1` 로 돌아온 상태.
→ "계정 전환 과정에서 뭔가 틀어졌다"는 의심에서 출발.

### 진단 과정 (단계별로 좁혀감)
1. **기본 점검**: 컨테이너 Up, `tailscale status` 에 로봇(raspberrypi-1,
   100.71.74.81) active·direct, 서버→Pi ping OK, discovery-server 11811 리스닝 OK,
   compose 의 IP 설정도 `100.95.193.1` 로 정상. → 설정·연결은 멀쩡.
2. **토픽은 보이나 데이터는?**: `ROS_SUPER_CLIENT=TRUE` 로 `ros2 topic list` 하니
   `/scan`·`/odom` 등 다 보임 (디스커버리 정상). 그런데 `ros2 topic hz /scan`,
   `/odom` **둘 다 수신율 0**. → **"디스커버리는 되는데 실제 데이터가 안 흐른다"**
   는 전형적 증상으로 판별.
3. **방화벽 의심 → 부분 원인 1 확인**: `ufw status` → active, `deny (incoming)`,
   그런데 tailscale0 허용 규칙 없음. 디스커버리(서버가 먼저 나가서 conntrack 으로
   응답 수신)는 되지만, Pi 가 먼저 밀어넣는 `/scan` 은 새 연결이라 차단됨.
   → `ufw allow in on tailscale0` 추가. **그래도 데이터 안 옴.** (UFW 는 필요조건
   이었지만 충분조건이 아니었음)
4. **tcpdump 로 물리 패킷 확인 → 진짜 원인 발견**:
   `tcpdump -ni tailscale0 host 100.71.74.81` →
   - Pi→서버: **오직 11811(디스커버리)로만** 감. `/scan` 데이터 포트로는 전무.
   - 서버→Pi: 데이터 포트(149xx)로 정상 송신 (서버는 Pi 를 올바른 tailscale IP 로 앎).
   → **비대칭**. 서버는 Pi 를 잘 찾는데 Pi 가 서버로 데이터를 안 보냄.
5. **서버 인터페이스 확인 → 근본 원인 확정**: 서버가 멀티홈
   (`192.168.0.142`, `192.168.20.100`, `docker0 172.17.0.1`, k3s `10.42.x`,
   tailscale `100.95.193.1`). DDS 참가자는 이 IP 를 **전부 locator 로 광고**하는데,
   원격 Pi 가 `/scan` 을 서버의 **LAN IP(192.168.x)** 로 보내려다 자기 홈 LAN 으로
   새어나가 서버에 도달하지 않음.
6. **검증**: 인터페이스 화이트리스트(tailscale+loopback) 프로파일을 만들어
   `ros2 topic hz /scan` 한 노드에만 적용 → **즉시 /scan 수신 시작**. 로컬 노드 간
   통신(talker/listener)도 정상. → 원인·해결 확정.

### 근본 원인 (두 겹이었음)
- **원인 A (서버, 네트워크)**: 멀티홈 서버가 DDS 로 여러 인터페이스를 광고 →
  원격 Pi 가 유저 데이터를 엉뚱한(LAN) locator 로 전송 → 데이터 유실.
  (+ UFW 가 tailscale0 인입을 막고 있던 부분 원인)
- **원인 B (로봇, 하드웨어)**: 위 A 를 고친 뒤에도 한동안 /scan 이 다시 0 이 됨.
  tcpdump 상 Pi 가 스캔 패킷을 전혀 안 보냄. 그런데 Pi 노드
  (`/hlds_laser_publisher` 등)는 전부 살아있음 → **라이다가 스캔을 안 만들어내는**
  상태(LDS 모터 정지). TurtleBot3 는 **배터리 전압이 낮으면 LDS 모터가 멈추는데
  Pi/ROS 는 계속 살아있어** 딱 이 증상이 난다. Pi 로컬에서 `ros2 topic hz /scan`
  으로 확인(로컬도 0이면 라이다 문제) 후 로봇 쪽 조치하니 **깨끗한 5Hz** 복구.

### 해결 (최종)
- **서버 방화벽** (★ git 밖 — 여기가 유일 기록):
  `sudo ufw allow in on tailscale0` (재부팅에도 유지됨)
- **DDS 인터페이스 화이트리스트**: `config/fastdds_iface_whitelist.xml` 신설,
  docker-compose 에서 `FASTRTPS_DEFAULT_PROFILES_FILE=/cfg/fastdds_iface_whitelist.xml`
  + `./config:/cfg` 마운트. (커밋 `3c06ea3`)
- **로봇**: 배터리/라이다 회전 확인 후 bringup 정상화 → /scan 5Hz.

### 재발 방지 · 교훈
- **"토픽은 보이는데 수신율 0" = 디스커버리 문제 아님.** 데이터 P2P 경로(방화벽/
  멀티홈 locator)를 의심하라. 진단의 결정타는 **`tcpdump -ni tailscale0`** 로
  실제 패킷의 방향·목적지 IP·포트를 보는 것.
- **멀티홈 서버 + VPN(Tailscale)** 조합에선 DDS 인터페이스 화이트리스트가 사실상
  필수. 안 그러면 상대가 엉뚱한 사설 IP 로 데이터를 보낸다.
- **노드는 다 살아있는데 특정 센서 토픽만 0** → 하드웨어(배터리/센서) 의심.
  로봇 쪽 로컬에서 `ros2 topic hz` 로 "소스에서 나오는지" 부터 확인해 문제를
  로봇/네트워크로 이분하라.
- **★ 서버 Tailscale IP 가 바뀌면** 다음 3곳을 함께 갱신: `fastdds_iface_whitelist.xml`
  의 address, compose 의 `ROS_DISCOVERY_SERVER`, discovery-server 의 `-l`.
- Discovery Server 환경에서 CLI 가 답답하면 `ROS_SUPER_CLIENT=TRUE` (+ daemon
  재시작). Remote PC 는 compose 에 넣어 상시 적용(커밋 `3c06ea3`).

### 관련
- 개념: [my_slam.md](./my_slam.md) 2장(TF/QoS), CLAUDE.md 3장(네트워크/디스커버리)
- 커밋: `3c06ea3` (DDS 화이트리스트 + super client)

---

## 2026-08-26 — 보정 URDF 배포 후 bringup 실행 실패 (robot_description YAML 파싱)

### 증상
Pi 에 보정 URDF 배포 후 bringup 실행 시:
```
urdf_file_name : turtlebot3_burger.urdf
[ERROR] [launch]: ... Unable to parse the value of parameter robot_description
as yaml. If the parameter is meant to be a string, try wrapping it in
launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
```
로봇 문제가 아니라 **URDF 파일 문제**. 표준 URDF 는 잘 뜨는데 보정판만 실패.

### 근본 원인
bringup(turtlebot3_state_publisher.launch.py)은 URDF 파일 내용을 문자열로 읽어
`robot_description` 파라미터로 넘기는데, 이때 파라미터 시스템이 그 문자열을
**YAML 로 파싱**해 타입을 추론한다. 그런데 보정판에 넣은 한글 주석에
**콜론+공백( ) 이나 줄 끝 콜론**이 있으면, YAML 이 그 줄을 "매핑 키"로 오해해
`mapping values are not allowed here` 로 파싱이 깨진다. 표준 URDF 의 주석/XML
에는 그 패턴이 없어서(예: `xmlns:xacro`, `http://` 는 콜론 뒤 공백이 없음) 문제없었다.

### 진단
호스트에서 그대로 재현:
```bash
python3 -c "import yaml; yaml.safe_load(open('description/urdf/turtlebot3_burger.urdf').read())"
# → mapping values are not allowed here ... line 10 "1) scan_joint (LDS 라이다 위치):"
```
트레일링 콜론 `...위치):` 이 범인.

### 해결
URDF **주석에서 콜론+공백/줄끝 콜론을 전부 제거**(`=>`, `-`, `(...)` 로 대체).
보정값(xyz/rpy)과 XML 본체는 불변. 커밋 이력 참조. 수정 후 재검증:
```bash
python3 -c "import yaml; yaml.safe_load(open('.../turtlebot3_burger.urdf').read())"  # 예외 없어야 함
xacro turtlebot3_burger.urdf > /dev/null                                             # 파싱 OK
```

### 재발 방지 · 교훈
- **URDF/센서 파라미터 파일의 주석에 `콜론+공백` 이나 `줄 끝 콜론` 금지.** 특히
  한글 설명 주석에서 흔히 나온다("항목: 설명", "다음과 같다:" 등).
- URDF 를 배포하기 전 `python3 -c "import yaml; yaml.safe_load(open(...).read())"`
  로 한 번 검사하면 이 에러를 사전에 잡는다.
- "표준은 되는데 내 파일만 안 될" 때는 로봇/네트워크가 아니라 **내가 바꾼 파일의
  내용**을 의심하라.

---

## 2026-08-27 — RealSense ROS 노드만 UVC 컨트롤 타임아웃 (pyrealsense2 는 정상)

### 증상
Pi 에서 `realsense2_camera`(apt, v4.58.3) 실행 시, 카메라는 인식되나 스트림 시작에서
반복 실패 후 죽음:
```
get_xu(...) xioctl(UVCIOC_CTRL_QUERY) failed on control 1 Last Error: Connection timed out
get_xu(...) xioctl(UVCIOC_CTRL_QUERY) failed on control 7 Last Error: No such file or directory
Error starting device: std::exception
```
그런데 **pyrealsense2 로 최소 스트리밍(640x480x15)은 문제없이 됨.** 케이블은 Intel 정품.

### 진단 과정
1. 해상도 낮춤(RGB 1280x720→640x480) → 동일 실패. 대역폭 문제 아님.
2. align_depth·initial_reset 등 무거운 옵션 다 off → 동일. 옵션 문제 아님.
3. **pyrealsense2 최소 스트리밍 테스트는 성공** → 카메라/USB/케이블 정상.
   실패 지점이 `global_timestamp_reader` 폴링과 인트린식 읽기의 **XU(extension
   unit) 컨트롤** = 하드웨어 메타데이터 접근. `control 7 No such file or directory`
   (ENOENT)는 커널에 그 XU 노드가 없다는 뜻.
4. 사용자가 pyrealsense2 를 **소스 빌드(`-DFORCE_RSUSB_BACKEND=true`, `/usr/local`)**
   한 것을 확인 → 결정적 단서.

### 근본 원인
pyrealsense2 와 ROS 노드가 **서로 다른 librealsense** 를 쓴다:
- `pyrealsense2` (소스, `/usr/local`) = **RSUSB(유저스페이스 USB) 백엔드** → 커널 패치 불필요 → OK
- `ros-humble-realsense2-camera` 가 링크한 apt `ros-humble-librealsense2` = **커널(V4L2)
  백엔드** → RealSense 커널 패치(uvcvideo XU/메타데이터) 필요 → 라즈베리파이 커널엔
  없음 → XU 컨트롤 타임아웃.
최소 스트리밍은 XU 를 안 건드려 되지만, ROS 노드는 메타데이터/인트린식을 XU 로 읽어 실패.

### 해결
realsense-ros(ROS 래퍼)를 **`/usr/local` 의 RSUSB librealsense 에 링크되도록 소스 빌드**:
```bash
# 커널 백엔드 apt 패키지 제거
sudo apt remove -y "ros-humble-realsense2-*" ros-humble-librealsense2 && sudo apt autoremove -y
# 소스 받아 워크스페이스에
mkdir -p ~/realsense_ros_ws/src && cd ~/realsense_ros_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd ~/realsense_ros_ws
# ★ 의존성 설치하되 librealsense2 는 skip (안 그러면 apt 커널 백엔드가 다시 깔림)
rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2
# /usr/local librealsense 를 명시해 빌드
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
source install/setup.bash
```
결과: ROS 노드가 RSUSB 백엔드를 써서 pyrealsense2 처럼 XU 문제 없이 뜨고, 서버에서
`/camera/camera/*` 토픽이 보임(빌드 성공).

### 재발 방지 · 교훈
- **라즈베리파이/Jetson 처럼 커널 패치가 어려운 플랫폼에선 librealsense 를 RSUSB
  백엔드로 빌드**하고 realsense-ros 도 그것에 링크한다. apt 패키지는 커널 백엔드라 안 됨.
- `rosdep install` 시 **`--skip-keys=librealsense2`** 로 apt librealsense 재설치를 막는다.
- "같은 카메라인데 SDK 는 되고 ROS 만 안 될" 때는 **두 경로가 다른 라이브러리/백엔드를
  쓰는지** 의심하라.

---

## 2026-08-29 — 카메라 토픽은 discover 되는데 데이터가 서버로 안 옴 (라이다는 정상)

### 증상
realsense-ros 빌드 성공 후, 서버 `ros2 topic list` 에 `/camera/camera/*` 다 보임.
그러나 color image·**작은 camera_info 조차** 수신율 0. 같은 Pi 의 `/scan`(라이다)은
동시각에 깨끗한 5Hz 로 정상 수신.

### 진단 과정 (배제법)
- **대역폭 아님**: 작은 camera_info(수백 byte)도 어떤 QoS 로도 0.
- **QoS 불일치 아님**: `ros2 topic info --verbose` 로 퍼블리셔 QoS(RELIABLE/VOLATILE)
  완전히 받아짐 → 표준 구독자와 매칭 가능.
- **디스커버리 불완전 아님**: 노드명/GID/QoS 까지 서버가 다 앎.
- **인터페이스 화이트리스트 아님**: 서버·Pi 양쪽 적용해도 동일. tcpdump 상 Pi 가
  서버 데이터 포트로 **하트비트는 보내나 데이터 샘플은 안 감**.
- **loopback locator 아님**: 화이트리스트에서 127.0.0.1 제거해도 동일.
- **카메라 특정 문제 확정**: 같은 링크에서 `/scan` 은 정상(5Hz). 차이는 참가자뿐 —
  카메라는 토픽 20+개의 복잡한 참가자, 라이다는 단순·소수.

### 근본 원인 (판정)
**Fast DDS Discovery Server 가 VPN(Tailscale) + 멀티홈 + 복잡한 참가자 조합에서
사용자 데이터 전달에 실패하는 한계.** 디스커버리/매칭은 되지만 실제 샘플 전송이
안 됨. 라이다(BEST_EFFORT·소수 엔드포인트)는 넘고 카메라(RELIABLE 기본·다수
엔드포인트)는 못 넘음. Fast DDS 레벨에서 더 파는 것은 효율 낮다고 판단.

### 해결 방향 (진행 예정)
**Pi↔서버 통신을 Zenoh 로 전환** — ROS 2 를 VPN/WAN 으로, 큰 데이터·멀티홈 환경에
보내기 위해 설계된 표준 해법(디스커버리 트래픽 97~99% 감소 보고). 두 방식:
- `zenoh-bridge-ros2dds` (덜 침습적, 각 머신 내부는 DDS 유지, 브리지만 tailscale 로 연결)
- `rmw_zenoh` (전 노드 RMW 교체)
추가로 카메라는 **compressed image transport + 저해상도**도 병행 필요(원시 이미지
~14MB/s 는 Tailscale 로 무리).

### 재발 방지 · 교훈
- **원격(VPN) ROS 2 에서 큰/복잡한 데이터(카메라·포인트클라우드)는 Fast DDS
  Discovery Server 가 한계.** 소형 센서(라이다)로 검증됐다고 카메라도 될 거라 가정 금지.
- "discover 는 되는데 데이터 0"에서 대역폭·QoS·디스커버리·화이트리스트를 다
  배제하면 **미들웨어 자체의 한계**를 의심하고 Zenoh 를 검토하라.
- 진단은 **가장 작은 메시지(camera_info)** 부터 확인하면 대역폭 문제를 빠르게 배제한다.

---

## 2026-09-01 — Zenoh Bridge 전환 완료 (카메라 15fps 성공) + 브리지 시작 순서 quirk

### 전환 결과
zenoh-bridge-ros2dds(1.10.0)로 전환 후 엔드투엔드 검증 성공:
- `/scan` 5Hz (지터 역대 최소), `/odom` 20Hz, `/tf`·`camera_info` 정상
- **카메라 color compressed 15fps, 프레임당 ~70KB ≈ 1.05MB/s** (raw 였다면 14MB/s)
- CLI(`ros2 topic list/hz`)가 super client 없이 그냥 동작. 구성 대폭 단순화
  (Discovery Server·인터페이스 화이트리스트·SUPER_CLIENT 전부 제거)

구성 요약은 CLAUDE.md 3번, 설정 파일은 `config/zenoh-bridge-server.json5`(서버,
compose 서비스) / `robot/config/zenoh-bridge-pi.json5`(Pi, systemd 서비스).

### 겪은 문제 — 첫 브리지 인스턴스에서 데이터만 0 (로컬 DDS 매칭 꼬임)
**증상**: 브리지 연결·라우트 생성·allow 매칭까지 전부 정상 로그인데 데이터만 0.
tcpdump 로 보면 Pi loopback 에 DDS 유저 데이터가 전혀 없음 (Fast DDS 끼리는
SHM 으로 통신해서 로컬 `ros2 topic hz` 는 정상으로 보임 — 함정).

**원인(검증됨)**: 처음엔 "브리지를 노드보다 먼저 켜면 안 된다"(순서 quirk)로
추정했으나, 이후 실증 테스트로 반박됨 — **브리지가 오래 떠 있는 상태에서 새로
시작한 노드(static_transform_publisher)의 데이터가 즉시 서버에 도착**함.
실제 원인은 전환 작업 중 첫 브리지가 **구버전 env(loopback 빠진 화이트리스트)의
옛 노드들과 공존하며 로컬 DDS 매칭 상태가 꼬인 것.** 정상 상태에선 시작 순서 무관.

**해결/운영 규칙**: 평상시엔 systemd/compose 로 브리지가 먼저 떠 있어도 된다.
드물게 "토픽은 보이는데 데이터 0" 증상(env 갈아엎기 등 비정상 상황 후)이 나면
브리지만 재시작하면 복구:
- Pi: `sudo systemctl restart zenoh-bridge`
- 서버: `docker compose restart zenoh-bridge`

### 진단 도구 메모 (이번에 유효했던 것)
- 프로세스의 **실제 env** 확인: `tr "\0" "\n" < /proc/<PID>/environ | grep ROS`
  (셸에서 echo 로 보는 값과 실행 중 프로세스의 값은 다를 수 있다)
- 브리지 debug 로그: `RUST_LOG=zenoh_plugin_ros2dds=debug` — 라우트 생성/매칭 상태가 다 보임
- 링크에 데이터가 실제로 흐르는지: `tcpdump -ni tailscale0 "tcp port 7447"`
- 로컬 DDS 유저 데이터: `tcpdump -ni lo "udp and greater 500"` (Fast DDS 끼리는
  SHM 이라 안 보일 수 있음에 유의)

---

## 2026-09-01 (2) — Pi 재부팅 후 카메라만 스트림 실패 (전원 부족)

### 증상
Pi 재부팅 후 서버 `topic list` 는 다 보이고 `/scan` 도 정상인데 **카메라 데이터만 0**.
Pi 로컬에서도 스트림 없음. 카메라 로그에 `RS2_USB_STATUS_BUSY`,
`control_transfer ... Resource temporarily unavailable` 연발. depth 는 열리는데
RGB 에서 실패. 브리지/네트워크는 이번엔 전부 정상이었음.

### 근본 원인
**Pi 전원 부족(undervoltage).** `vcgencmd get_throttled` → `0x50005`
(bit0=현재 전압 부족, bit2=스로틀링 중). RGB 센서 기동 시 추가 전류를 못 끌어와
USB 컨트롤 전송부터 실패. "재부팅 전엔 됐는데" = 코드가 아니라 전원 환경 변화.
※ 2026-08 라이다 정지 사건(배터리 → LDS 모터 멈춤)과 같은 계열.

### 진단 순서 (재사용 가능)
1. `vcgencmd get_throttled` — **bit0 이 켜져 있으면 그 이상 볼 것 없이 전원부터**
2. 카메라 프로세스 중복 확인 — 옛 인스턴스가 USB 를 잡고 있으면 BUSY
3. 소프트 USB 리셋 (sudo 불필요 — realsense udev 규칙 덕에 노드 권한 열려 있음):
   `python3` 로 `/dev/bus/usb/BUS/DEV` 에 `USBDEVFS_RESET` ioctl
4. 안 되면 물리 재연결 → 그래도 안 되면 전원이 원인일 확률 높음

### 곁가지 함정들 (이번에 겪음)
- **`pkill -f` 자기살해**: SSH 원격 명령 문자열에 패턴이 포함되면 자기 세션이
  죽는다 → 옛 프로세스가 살아남아 새 인스턴스와 USB 충돌(BUSY)로 이어졌음.
  bracket 트릭 사용: `pkill -f "realsense[2]_camera_node"`
- `kill -9` 뒤 남는 `<defunct>` 좀비는 무해 (USB 는 이미 해제됨)
- **죽인 노드의 토픽이 서버 `topic list` 에 계속 보이는 이유**:
  ① 서버 ros2 daemon 의 그래프 캐시 (`ros2 daemon stop && start` 로 새로고침)
  ② `kill -9` 는 undeclare 없이 죽어서 DDS liveliness lease(~20s) 만료까지
     브리지가 라우트를 회수하지 못함 (곱게 끄면 즉시 회수)

### 해결/현재 상태
전원 조치 + 재부팅 후 카메라 15fps 복구, 서버 수신 정상. 단 **undervoltage 는
여전히 지속**(0x50005) — 지금은 마진으로 동작하는 상태라 재발 위험 있음.
근본 대책: 정격 전원(라즈베리파이4 기준 5V/3A+) 또는 배터리 충전 관리,
필요 시 카메라용 유전원(powered) USB 허브.

---

## 2026-09-02 — 카메라 RGB(컬러) 스트림이 ROS 노드에서 간헐 실패 (RSUSB 컨트롤 전송 불안정)

### 증상
카메라 실행 시 **depth 는 매번 열리는데 RGB(컬러) 는 자주 실패.** 로그에
`set_xu(ctrl=1) failed! Numerical argument out of domain` (RGB 인트린식 읽기) 후
`control_transfer returned error, index: 768, Resource temporarily unavailable`
(EAGAIN)가 수백~수천 개 쌓이며 컬러 스트림이 안 열린다. **재시도하면 뜰 때도 있다.**

### ⚠️ 처음 오진 → 정정
처음엔 "SDK 도 컬러에서 멈춘다"고 보고 **하드웨어(RGB 센서) 문제로 오판**했다.
그러나 그 SDK 테스트는 **이전 실패 런의 좀비(kill -9 잔재)가 USB 를 쥔 오염된
상태**에서 돌린 것이었다. 깨끗한 상태에서 재검증하니:
- **pyrealsense2 SDK (color+depth 640x480x15) → 30프레임 항상 성공** = 하드웨어 정상.
- **동일 조건 ROS 노드 → 1회차 실패, 2회차 성공(에러 1개)** = 간헐적.
→ 하드웨어 아님. **교훈: 디버깅 중 kill -9 남발이 장치 상태를 오염시켜 오진을
   유발했다. 격리 테스트는 반드시 깨끗한 장치 상태에서.**

### 배제한 원인
- 전원(외부 어댑터, throttled bit0 clear), USB 대역폭(저해상도도 동일),
  align_depth(off 도 동일), 라이브러리(/usr/local RSUSB 2.58.2 하나뿐, apt 없음,
  ROS·SDK 동일 lib 사용).

### 근본 원인 (판정)
**Pi4 USB 컨트롤러(VL805)에서 RSUSB(유저스페이스 libusb) 백엔드의 컨트롤 전송이
간헐적으로 불안정.** 최소 스트리밍만 하는 SDK 는 컨트롤 전송이 적어 안정적이나,
ROS 노드는 시작 시 RGB 인트린식을 XU 컨트롤로 읽는 등 컨트롤 전송을 많이 해서,
그 순간 EAGAIN 이 몰리면 RGB 시작에 실패한다. 잘 풀리는 판엔 성공.

### 대응
1. **재시도** — 간헐적이므로 그냥 카메라 런치 재시작하면 대개 뜬다(가장 빠름).
2. **유전원 USB3 허브** — RSUSB 컨트롤 전송에 안정적 USB 경로 제공(근본 완화).
   SBC + RealSense 의 정석 대책.
3. 부하/컨트롤 접근 줄이기(해상도 다운 등)도 보조.
   ※ 이건 전원 문제가 아니라 **USB 컨트롤 계층 안정성** 문제다.

### 진단 도구 메모
- **SDK 격리 테스트는 깨끗한 장치 상태에서** (실패 런 좀비 정리 + USB 리셋 후).
  스크립트를 파일로 올려 실행(heredoc over SSH 는 자주 끊김).
- 로드된 librealsense 확인: `ldd <node>` / `dpkg -l | grep realsense` /
  `ls -l /usr/local/lib/librealsense2.so*`, 로그의 "Running with LibRealSense vX".
- `throttled` 는 bit0(현재) vs bit16(이력) 구분 — 0x50000 은 이력만.
- ⚠️ 원격 디버깅 시 `kill -9` 남발 자제 — 장치 상태 오염으로 오진 유발.

---

## 2026-09-03 — 공식 카메라 노드를 자체 pyrealsense2 노드로 대체 (개발 중 실측한 USB 특성 3가지)

### 배경
09-02 항목의 "RGB 간헐 실패"는 재시도로 넘어갈 순 있었지만, 실패 시 노드가 hang 이라
respawn 도 못 잡고, align_depth·camera_info 가 그 실패 지점(XU 캘리브레이션 읽기)에
직접 의존해 Vision 파이프라인의 전제가 흔들렸다. 순수 pyrealsense2 스트리밍은 항상
됐으므로 **그 최소 경로 위에 ROS publish 만 얹은 자체 노드**를 만들기로 했다.
정상 동작 원리는 [realsense_bringup.md](./realsense_bringup.md). 여기엔 만들면서
부딪힌 것들을 남긴다 — 하나같이 문서에 없고 실측으로만 알 수 있던 것들이다.

### 발견 1 — 불안정한 건 정확히 "RGB 컨트롤 경로", 세션 단위
자체 노드도 첫 시도에 `get_xu(ctrl=1) failed` 를 냈다. 우리 코드가 XU 를 건드리는 곳은
`get_intrinsics()`(camera_info) 하나 → 순수 SDK 테스트가 항상 됐던 이유는 **그걸 호출
안 해서**였다. 이어서 `rs.align` 도 같은 이유로 실패했다(내부에서 인트린식·익스트린식을
XU 로 읽음). 경로 상태는 장치 open 세션 단위로 갈리며, 좋은 세션에선 전부 즉시 읽힌다.
→ **해법**: 캘리브레이션은 공장값이라 불변 → 좋은 세션에서 읽어 파일 캐시, 나쁜 세션은
캐시로 numpy 수동 정렬. 한 번 성공하면 영구히 XU 독립.

### 발견 2 — hang 은 GIL 을 쥐고 멈춘다 → 스레드 감시 불가
XU 실패 후 파이프라인을 내렸다 다시 `start()` 하면 멈추는 경우가 있었고, 이때 같은
프로세스에 넣어둔 20초 감시 타이머가 **발동하지 않았다**. `start()` 가 C++ 안에서
GIL 을 쥔 채 블록되어 파이썬 인터프리터 전체가 얼어붙은 것. → **캡처를 별도 프로세스로
분리**(multiprocessing spawn). 부모는 자식 GIL 과 무관하게 살아 있고 SIGKILL 로 확실히
회수 가능. 실제로 hang 감지 → kill → 복구가 동작함을 확인.

### 발견 3 — 공격적 복구는 카메라를 USB 버스에서 떨어뜨린다 (★ 두 번 재연결 대가로 배움)
- (a) 복구 루프에서 USB 소프트 리셋(USBDEVFS_RESET ioctl)을 **11초 간격으로 연타**하자
  `No such device` → `lsusb` 에서 카메라가 사라짐. 리셋은 한 번은 약, 연타는 독.
- (b) 리셋을 온화하게 바꾼 뒤에도 또 사라졌는데, 이번엔 리셋 실행 전이었다. 직전 행위는
  **12초 타임아웃으로 `start()` 도중인 자식을 SIGKILL** 한 것. USB 트랜잭션 한가운데서
  프로세스가 죽으면 장치 USB 상태가 꼬여 de-enumerate 된다. 오늘 장치가 사라진 매 순간이
  정확히 이 직후였다.
- 단계 로그를 넣어 재보니 **콜드 스타트는 장치 열거(`rs.pipeline()`, RSUSB 프로빙)에만
  ~11초**, `start()` 는 0.7초. 12초 타임아웃은 정상 시작을 딱 그 지점에서 죽이고 있었다.
  → 시작 타임아웃 45초(단계마다 리셋), 리셋은 연속 실패 2회 후·최소 60초 간격·장치가
  버스에 있을 때만, 재시작은 지수 백오프, 장치 없으면 재연결만 대기. 이후 조기 kill 없이
  두 번째 세션에서 스스로 수렴(15fps)하는 것 확인.

### 결과
자체 노드 15.0fps 안정, color 36KB + 정렬 depth(PNG16) 93KB ≈ 1.9MB/s 서버 수신 확인,
캘리브레이션 캐시 생성(depth→color 14.9mm — D435i 물리 배치와 일치). full_bringup 기본
드라이버를 `custom` 으로 전환, 공식 노드는 `camera_driver:=realsense2` 로 유지.

### 재발 방지 · 교훈
- **카메라 프로세스는 열거/start 도중 kill 금지.** 멈춘 것 같아도 콜드 스타트 11초는 정상.
  손으로 정리할 땐 로그에 fps 가 찍히는(스트리밍 중) 상태에서.
- USB 리셋은 진단 도구지 루프에 넣을 것이 아니다. 넣는다면 간격·횟수 제한 필수.
- "SDK 는 되는데 ROS 는 안 된다"는 결론을 내리기 전에 **SDK 테스트가 같은 호출을 하는지**
  확인하라 (이번엔 `get_intrinsics` 하나 차이였다).
- 감시자는 감시 대상과 **프로세스를 분리**하라. 같은 인터프리터 안의 타이머는 GIL 에 묶인다.
- 원격 디버깅 시 SSH 명령줄에 패턴 문자열이 있으면 `pgrep -f` 가 자기 자신을 잡는다 —
  bracket 트릭(`rs_camera[_]node`) 없이는 "잔존" 오탐이 난다 (09-01(2) 항목과 같은 함정).

---

## 2026-09-03(2) — 카메라 fps 15→8: 부하 시에만 나타나는 전압 처짐 (CPU 600MHz 스로틀)

### 증상
설치 패키지 런치로 전환한 뒤 자체 노드가 **8fps** 로만 나옴 (같은 날 아침 /tmp 실행은 15fps).
서버 수신도 7.4Hz 로 일치 → 네트워크 아님. 부모·자식 프로세스 둘 다 CPU 100% 포화.

### 진단
- `vcgencmd get_throttled` = **`0x50005`** (bit0 현재 전압 부족 + bit2 현재 스로틀링) 이고
  `measure_clock arm` = **600MHz** (정격 1500). 온도 66.7°C 는 열 스로틀(80°C) 아래 → 전압.
- **카메라를 끄니** `0x50000`(이력만) + 1200~1300MHz 로 즉시 회복, 다시 켜니 재발.
  → 정적 전원 부족이 아니라 **D435i 스트리밍 + CPU 부하에서만 5V 가 처지는** 경로 문제
  → **확정 원인**: 이날 아침 Pi 전원을 외부 어댑터 직결에서 **OpenCR 5V 출력으로 되돌린 것**.
  OpenCR 5V 는 Pi4 단독은 버티지만 D435i 스트리밍 부하가 얹히면 처진다 (09-01(2) 항목과 동일 결론).
  → 이후 **5V/3A 어댑터 USB-C 직결**로도 재측정: 스트리밍 중 `0x50005`/600MHz 와 1800MHz 를
  오가며 fps 가 5~13 으로 출렁임. 3A 는 Pi4 단독 정격 최소치라 D435i(USB +0.7~1A) 까지는
  부족 — 전원 보강(CLAUDE.md 다음 할 일 3번, 5V/5A 급) 전까지 fps 저하는 전원 탓으로 볼 것.

### 교훈 (09-02 항목의 "전원 아님" 판정을 정정)
- 09-02 에 "`0x50000` 은 이력일 뿐 라이브 정상"이라 한 것은 **유휴 상태 측정**이었다.
  전압 판정은 **부하(카메라 스트리밍) 중에** 해야 한다. 그때 bit0 이 켜지면 전원 경로가 부족한 것.
- fps 가 정확히 절반 근처로 떨어지면 코드보다 `get_throttled` 부터 볼 것 (클럭 1500→600).
- 전원 보강(CLAUDE.md 다음 할 일 3번: 배터리→5V/5A 컨버터→Pi 직결)의 필요성이 라이브
  데이터로 재확인됨. 개발 중엔 굵은 USB-C 케이블 + 정격 어댑터를 Pi 에 직결.

---

## 2026-09-08 — 통합 런치 구성 중 두 가지: 브리지 세션 이중화(메시지 2배 수신) / RViz "No Image"

### (1) 서버에서 카메라 토픽이 정확히 2배 속도로 수신, 검출 노드 정체
- **증상**: 서버 `ros2 topic hz /camera/color/compressed` 가 6fps 카메라인데 ~12Hz. stamp 별로 메시지가
  `CCDD` 로 두 번씩 도착(진단 스크립트로 확인). 검출 노드는 살아 있는데 처리 로그·출력이 끊김.
- **진단**: Pi 로컬은 stamp 당 1회(정상), Pi 카메라 퍼블리셔 1개, 서버 DDS 에서도 브리지 writer 1개 →
  중복은 **브리지 구간**. 서버 브리지 재시작으론 그대로, **Pi 브리지 재시작으로 해소**.
- **원인(추정)**: 서버 브리지(컨테이너)를 재시작/재생성했을 때 Pi 브리지가 옛 zenoh 세션을 정리하지
  못하고 새 세션을 추가로 맺어, 같은 샘플을 두 세션으로 두 번 전달한 것.
- **해결**: Pi 브리지 재시작. systemd `Restart=always` 라 **프로세스만 죽이면 3초 뒤 자동 재시작 —
  sudo 불필요**: `kill $(pgrep -f "zenoh-bridge-ros2dd[s]")`. (정석은 `sudo systemctl restart zenoh-bridge`)
- **재발 방지**: 서버 브리지를 재시작/재생성했으면 **Pi 브리지도 같이 재시작**하는 습관. 검출 노드의
  FramePairer 에 "이미 처리한 stamp 이하 무시"를 넣어 중복 전달이 와도 정체되지 않게 함.
- 빠른 판별: `ros2 topic hz` 가 발행 fps 의 정수배로 나오면 중복 전달을 의심.

### (2) RViz Image 디스플레이 "No Image" (`image_transport/compressed_sub does not exist`)
- **원인**: 컨테이너에 `image_transport` 의 compressed 플러그인이 없음 — RViz 가 `/…/compressed` 토픽을
  풀 수 없다 (ros-humble-desktop 에 미포함).
- **해결**: Dockerfile vision 스테이지에 `ros-humble-image-transport-plugins` 추가 (별도 apt 레이어).
  아울러 자체 카메라 노드·검출 노드의 `format` 문자열을 image_transport 표준
  `"bgr8; jpeg compressed bgr8"` 로 통일 (디코더 호환, 소비 측 imdecode 는 영향 없음).
- 참고: RViz Image 디스플레이는 설정의 Reliability 와 무관하게 Reliable 로 구독을 시도해 best effort
  퍼블리셔와 "incompatible QoS" 경고가 뜰 수 있으나, 실제로는 표시된다 (경고는 무해).

### 교훈 (반복)
- 정지와 실행을 한 명령에 넣고 `pkill -f 패턴` 을 쓰면 실행 줄의 문자열에 매치돼 셸이 자살한다 —
  오늘도 `rviz[2]` 패턴이 `rviz2 -d …` 줄에 걸렸다. 정지는 별도 호출로.

---

## 2026-11-05 — Pi 전원 보강: 배터리 분기 → UBEC 5V/5A → GPIO (부하 시 저전압 해소)

### 배경 (보강 전 실측)
09-03(2) 항목의 부하 의존 저전압의 원인을 멀티미터로 분해했다 (카메라 부하 상태).

| 측정 위치 | 전압 | 해석 |
|---|---|---|
| OpenCR 5V 출력 단자 | 4.94V | 출발점부터 Pi 권장 5.1V 보다 낮음 (레귤레이터 한계) |
| Pi GPIO 2번–9번 핀 | 4.78V | 저전압 판정 4.63V 까지 여유 0.15V 뿐 |
| 선·핀 손실 | 0.16V | 정상(≤0.05V)의 3배 — 가는 전원선 |

평균이 4.78V 면 카메라·라이다의 순간 전류에 4.63V 아래로 빠진다 → `0x50005`, 600MHz.
스펙상 OpenCR 5V 는 4A 지만 Pi4 + LDS(Pi USB 경유) + D435i(Pi USB 경유) 가 전부 이 하나에
걸려 정격 근처에서 전압이 처진다. "4A 면 충분" 은 전류 기준이고, 문제는 **전압 유지력**이었다.

### 조치 (배선, git 에 안 남음)
```
배터리 11.1V (T-plug) ─┬─ OpenCR (기존)
                      └─ UBEC 5V/5A (출력 5.25V, 연속 5A) ─ 서보 플러그 ─ Pi GPIO 4번(5V)·6번(GND)
```
- OpenCR → Pi GPIO 전원선은 제거 (두 전원 동시 연결 금지). OpenCR–Pi USB(데이터)는 유지 → GND 공유.
- 연결 전 확인: UBEC 출력 ≤5.3V (GPIO 는 Pi 입력 보호회로를 우회), 서보 플러그 방향
  (빨강=4번, 검정=6번, 빈 칸=2번 — 뒤집으면 5V 레일 단락).
- ⚠️ 분기 구성에서 UBEC 쪽은 **OpenCR 전원 스위치를 거치지 않는다** → OpenCR 를 꺼도 Pi 는 켜진 채
  배터리를 소모하고 OpenCR 의 저전압 경고음도 멈춘다. 사용 후 배터리 커넥터를 뽑을 것
  (또는 UBEC 입력에 스위치/LiPo 경보기 추가).

### 결과 (보강 후, 같은 날)
- Pi GPIO 전압 4.91V (사용자 멀티미터 실측, 보강 전 4.78V).
- 부하 시험 (Pi 에서 `vcgencmd` 1초 간격 기록, full_bringup = 모터·라이다·카메라):

| 조건 | 시간 | `get_throttled` | ARM 클럭 평균 / 최저 | 카메라 fps | 최고 온도 |
|---|---|---|---|---|---|
| 보강 전 (09-03(2)) | — | `0x50005` 반복 | 600MHz 고착 구간 | 5~13 출렁임 | 66.7°C |
| 6fps (기본) | 240s | **0x0 전 구간** | 1656 / 1100 MHz | **6.0 고정** | 64.7°C |
| 15fps | 240s | **0x0 전 구간** | 1719 / 1100 MHz | **15.0 고정** | 73.5°C |

  /odom 20Hz, /scan 5Hz 정상. 클럭 1100~1800 변동은 ondemand 거버너 (전압·열 스로틀 아님).
- 한 번 예외: 노드를 정리하던 21:49:52 에 커널 `Undervoltage detected!` → 4초 뒤 `Voltage normalised`
  (`0x50000` 이력 플래그 남음). 측정 중엔 없었고, 같은 절차로 재시작·종료했을 땐 재현되지 않음 →
  원인 미확정 (종료 시 순간 과도현상 또는 배터리 잔량). 주행 시험 때 재관찰할 것.
- 15fps 에서 73.5°C — 열 스로틀(80°C) 아래지만 장시간 15fps 면 방열판/팬 권장.

### 덤: 원격 백그라운드 launch 가 Ctrl+C(SIGINT)로 안 꺼짐
- 증상: SSH 로 `nohup ros2 launch … &` 한 launch 에 `kill -INT` 해도 무반응, 새로 띄운 launch 와
  겹쳐 turtlebot3_ros 두 개가 같은 시리얼(/dev/ttyACM0)을 잡고 한쪽이 `stack smashing` 으로 죽음.
- 원인: **잡 컨트롤 없는(비대화형) 셸의 `&` 는 자식의 SIGINT 를 SIG_IGN 으로 물려준다.** launch 도,
  launch 가 자식에 보내는 SIGINT 도 무시됨. launch 에 SIGTERM 을 주면 launch 만 죽고 노드는 고아로 남음.
- 해결: 시작 스크립트에 `set -m` (백그라운드 잡이 SIGINT 를 무시하지 않음) + 프로세스 그룹 ID 저장,
  종료는 `kill -INT -<pgid>` → 2초 만에 전 노드 정상 종료 확인. 그리고 **종료 확인 전엔 재시작하지 말 것**
  (이번에 확인 루프가 끝나기 전에 재시작해 카메라 둘이 장치를 다투고 USB 소프트 리셋까지 갔다 — 카메라는 무사).

---

## 2026-11-10 — 웹 백엔드(FastAPI+rclpy) 종료 시 traceback·abort: uvicorn 의 신호 재발생과 중복 SIGINT

### 증상
`my_web_api` 를 Ctrl+C(SIGINT)로 끄면 0.4초 만에 내려가긴 하지만 매번 긴 traceback 이 찍혔다.
MJPEG 영상 연결이 열린 상태에선 `asyncio.exceptions.CancelledError` 가 줄줄이 나오고, 한 번은
`terminate called without an active exception` 과 함께 **exit code -6(abort)** 으로 죽었다.

### 진단
1. 처음 traceback 의 뿌리는 `uvicorn/server.py capture_signals → signal.raise_signal(...)` → `KeyboardInterrupt`.
   uvicorn(0.34)은 **정상 종료를 마친 뒤 받았던 신호를 다시 발생**시킨다(상위 프로그램에 신호를 돌려주려는 의도).
   이 KeyboardInterrupt 가 이벤트 루프를 뚫고 나오면서 끝나지 않는 MJPEG 응답 태스크가 강제 취소됐다.
2. `main()` 에서 KeyboardInterrupt 를 잡아 넘기자, 이번엔 ROS 정리(`executor.shutdown` → `rclpy.shutdown`)가
   이상한 상태에서 돌며 C++ 쪽이 abort 했다.
3. 신호를 직접 받도록 바꾼 뒤에도 lifespan 태스크 취소 traceback 이 남아 로그를 보니
   **`종료 요청` 로그가 두 번** 찍혀 있었다: 프로세스 그룹 전체에 간 SIGINT 1번 + `ros2 launch` 가 자식에게
   **한 번 더 전달**한 SIGINT. uvicorn 은 두 번째 신호를 "강제 종료"로 처리해 정리 단계를 건너뛴다.
   (터미널 Ctrl+C 도 포그라운드 그룹 전체에 가므로 똑같이 두 번이 된다.)

### 근본 원인
끝나지 않는 응답(MJPEG·WebSocket)이 있는 서버에서, 종료 신호의 **주인이 여럿**(uvicorn·rclpy·launch 전달)이고
uvicorn 이 신호를 재발생시키는 동작이 겹친 것.

### 해결 (`remote_pc/src/my_web_api/my_web_api/server.py`)
- rclpy 의 SIGINT 처리기 끔(`SignalHandlerOptions.NO`), uvicorn 의 `capture_signals` 를 아무것도 안 하게 재정의 →
  **신호는 main() 한 곳에서만** 받는다.
- 신호 처리기: `stop` 이벤트 set(열린 MJPEG·WebSocket 루프가 스스로 끝남) → `server.handle_exit()`(정상 종료 요청).
  **1초 안에 다시 온 신호는 중복으로 무시**(그 뒤의 신호는 사람이 한 번 더 누른 강제 종료로 전달).
- `server.run()` 이 정상 반환한 뒤에 executor·노드·rclpy 순서로 정리.
- 결과: MJPEG 연결이 열린 상태에서 SIGINT → `Application shutdown complete` → `process has finished cleanly`,
  traceback 0 (2회 반복). compose 는 `stop_signal: SIGINT` 로 같은 경로를 탄다.

### 덤: 컨테이너에서 pytest 가 시작도 못 함
`ModuleNotFoundError: No module named '_pytest.scope'` — apt 의 pytest 6.x 와 pip 로 들어온 anyio 4.x 의
pytest 플러그인이 호환되지 않음. 이 패키지는 비동기 테스트가 없으므로 `setup.cfg [tool:pytest] addopts = -p no:anyio`.

### 재발 방지
- 끝나지 않는 응답을 가진 서버는 **종료 이벤트를 응답 루프가 직접 확인**하게 만든다 (강제 취소에 기대지 않기).
- `ros2 launch` 아래에서 도는 프로세스는 SIGINT 를 **두 번 받을 수 있다**고 가정한다.

---

## 2026-11-12 — 로봇이 가만히 있는데 웹 지도의 방향 화살표가 저절로 돈다: IMU 자이로 영점 틀어짐

### 증상
SLAM 없이 웹 콘솔(odom 기준 표시)을 보면 멈춰 있는 로봇의 방향 화살표가 사선으로 가 있고 조금씩 돌아간다 (사용자 발견).

### 진단
로봇 정지 상태에서 60초 측정 (서버에서 /odom, /imu, /joint_states 구독):

| 항목 | 값 |
|---|---|
| 바퀴 각도 변화 / 직진 속도 | 0.0015 rad / 0.0005 m/s (실제로 안 움직임) |
| IMU 자이로 z 평균 | **−0.0041 rad/s** (정지 시 0 이어야 함) |
| odom yaw 변화 · IMU orientation yaw 변화 | 둘 다 **−15°/분** (24분이면 한 바퀴) |

- odom 방향이 IMU 방향과 똑같이 변한다 → turtlebot3_node 가 `use_imu: true` 라 **회전각을 IMU 로 계산**하기 때문
  (odometry.cpp — 이동거리는 바퀴, 회전각은 IMU 변화량). 바퀴는 정지 → 원인은 자이로 쪽.
- turtlebot3_node 는 시작할 때(`check_device_status`) OpenCR 에 `imu_re_calibration`(control table 59)을 써서 5초 동안
  자이로 영점을 잡는다. 이 bringup 은 그날 아침 부팅 때 자동 시작됐고, 이후 Pi 온도는 75°C.

### 근본 원인 (추정 → 보정으로 원인 확정)
자이로 영점(바이어스)이 틀어진 상태. 영점을 잡을 때 진동이 있었거나, 잡은 뒤 온도가 올라 바이어스가 변한 것으로 추정.
→ 아래 재보정으로 즉시 사라졌으므로 **센서 고장이 아니라 영점 문제**로 확정.

### 해결
로봇을 세워 둔 채 TurtleBot3 의 `/reset` 서비스 호출 (자이로 재보정 5초 + odom 0 초기화, 로봇은 움직이지 않음):
```bash
ros2 service call /reset std_srvs/srv/Trigger   # Pi 에서 — Trigger_Response(success=True, 'Calibration End, Odom reset requested')
```
재측정 (60초, 정지): 자이로 z 평균 **0.00000 rad/s**, odom yaw 변화 **+0.03°/분** (보정 전의 약 1/500).

### 재발 방지
- bringup 시작(부팅·서비스 재시작) 후 5초는 로봇을 건드리지 않는다 (pi_setup.md 9단계에 명시).
- 웹 지도에서 멈춘 로봇의 화살표가 돌면 이 항목의 `/reset` 으로 재보정. SLAM 실행 중엔 odom 이 0 으로 튀므로 하지 말 것.
- 영향: SLAM 중엔 라이다 매칭이 map 방향을 보정하지만, 제자리 회전 정확도·지도 품질·정지 후 출발 방향에 나쁘다 →
  실주행 전 확인 항목. EKF(robot_localization)로는 해결되지 않는다 (같은 자이로 값을 그대로 믿음).
- 온도 상승으로 다시 틀어지는지는 보정 후 30분 관찰로 확인 (아래 후속 기록).

### 후속: 보정 후 30분 관찰 (5분 간격, 각 60초 창, 로봇 정지)

| 경과 | 자이로 z 평균 | odom yaw | Pi 온도 |
|---|---|---|---|
| 0분 | +0.00°/분 | +1.12° | 75.9°C |
| 10분 | +0.00°/분 | +1.30° | 75.4°C |
| 20분 | +0.00°/분 | +1.46° | 75.9°C |
| 30분 | +0.00°/분 | +1.69° | 75.4°C |

30분간 0.57° (약 0.02°/분) — 보정 전 15°/분 대비 사실상 정지. 다만 이번 보정은 **이미 75°C 로 데워진 상태**에서 했고
관찰 중 온도도 일정했으므로, "차가울 때(부팅 직후) 영점을 잡고 데워지며 틀어졌다"는 가설은 아직 확인되지 않았다.
다음 콜드 부팅 때 부팅 직후와 30~60분 뒤의 정지 상태 드리프트를 비교하면 판가름 난다. 확인되면 대책은
"데워진 뒤 한 번 더 `/reset`" 또는 정지 감지 시 자동 영점 보정.

---

## 2026-11-12 — 통합 런치에서 Vision 이 자기 설정을 못 읽고 기본값(torch)으로 돌았다: launch 인자 이름 충돌

### 증상
웹 모드 관리자로 `system.launch.py` 를 띄우자 검출 노드 로그가 `백엔드: torch`, 추론 12ms. `vision_params.yaml` 의
`backend: tensorrt`(사용자가 2026-09 에 바꾼 기본값)가 적용되지 않았다. `ros2 param get /detector backend` → `torch`.

### 진단
- 설치된 `vision_params.yaml` 은 `tensorrt` — 파일은 맞음.
- 실행 중인 detector 프로세스의 명령줄을 보니 `--params-file` 이 **`nav2_params.yaml`** (+ 임시 파일)이었다.
- `navigation.launch.py` 와 `vision.launch.py` 가 **둘 다 `params_file` 이라는 이름의 인자**를 쓴다. ROS 2 launch 의 인자
  (LaunchConfiguration)는 `IncludeLaunchDescription` 사이에 **공유**되고, `DeclareLaunchArgument` 의 기본값은 "아직 값이
  없을 때만" 적용된다 → 먼저 포함된 navigation 이 `params_file = nav2_params.yaml` 로 정한 뒤, vision 의 기본값은 무시됐다.
  nav2_params.yaml 엔 `detector:` 섹션이 없으니 detector 는 코드 기본값(`backend='torch'` 등)으로 동작.

### 근본 원인
여러 런치를 한 런치에 포함할 때 같은 이름의 인자가 범위 없이 섞인 것. `system.launch.py` 를 만든 2026-09-08 부터
있었고, 그때 "통합 실행도 TensorRT 인가" 라는 질문에 확인 없이 그렇다고 답했다 (통합 런치 실기 검증은
`backend:=tensorrt` 를 직접 넘겨서 해 차이를 못 봤다).

### 해결
`system.launch.py` 에서 각 하위 런치를 `GroupAction([...], scoped=True)` 로 감쌌다 — 안에서 정한 인자가 밖으로 새지 않는다.
결과: detector `backend=tensorrt`·`target_frame=map`, TensorRT 엔진 로드·추론 6ms, Nav2 `controller_frequency=10.0` 유지.

### 재발 방지
- 런치를 조합할 땐 하위 런치를 scoped GroupAction 으로 감싼다. 특히 `params_file`·`use_sim_time` 같은 흔한 이름.
- 설정이 적용됐는지는 파일이 아니라 **실행 중인 노드**로 확인한다: `ros2 param get <노드> <파라미터>`.

## 2026-11-12 — 라이다 `/scan` 만 끊김: 드라이버는 살아 있는데 발행 정지 (원인 조사 중)

### 증상
- 밤새 켜 둔 로봇을 아침에 보니 웹 콘솔의 라이다 수신이 없었다. 라이다는 돌고 있었다. 아침에 로봇을 껐다 켰는데,
  4시간쯤 뒤(09:01 부팅 → 13:08) 다시 `/scan` 만 끊겼다. odom 20Hz·카메라 6fps·배터리는 정상.

### 진단
1. **어디서 끊겼나** — Pi 안에서 직접 구독해도(`ros2 topic echo /scan --qos-reliability best_effort`) 6초간 0 개.
   → zenoh·Tailscale·서버 문제가 아니라 **Pi 의 라이다 쪽**.
2. **프로세스** — `tb3-bringup` 서비스·`hlds_laser_publisher` 살아 있음, 드라이버 로그에 오류 없음.
3. **하드웨어** — 커널 로그에 USB 끊김 없음, `get_throttled=0x0` (이번 부팅 저전압 없음).
   시리얼을 1초 직접 읽어 보니 12,474 B, 패킷 헤더 60종(0xA0~0xDB) 전부, 프레임 시작(`FA A0`) 초당 5회
   → **LDS-01 은 정상 프레임을 5Hz 로 보내고 있었다.**
4. **드라이버 코드** (ROBOTIS `hls_lfcd_lds_driver` humble, `hlds_laser_publisher.cpp`) — 메인 루프는
   "시리얼에서 `FA A0` 찾기 → 나머지 2518 B 읽기 → 바로 `publish`" 뿐. 정상 프레임이 들어오는데 발행이 멈출 경로가
   코드상으로는 보이지 않는다.
5. **복구** — 위 3번에서 시리얼을 1초 같이 읽은 직후 `/scan` 이 5Hz 로 돌아왔다 (32분 정지 후, 수십 초 안에 복구).
6. 단서: 정지 중 1초 같이 읽기는 12,474 B(=거의 전부)를 가져갔고, 정상일 때 같은 시험은 210~274 B 뿐이었다
   (드라이버가 먼저 읽어 가므로). → **정지 중엔 드라이버가 시리얼을 실제로 읽지 않고 있었던 것**으로 보인다.
   반면 `/proc/<pid>/io` 의 rchar 는 정지 중에도 ≈12.6 KB/s 로 늘고 있어 서로 맞지 않는다 — 다음 발생 때 확인할 점.
   (메인 스레드 wchan=`do_poll` — 읽기 대기. sudo 없이는 `strace -p` 불가: `ptrace_scope=1`)

### 근본 원인
**미확정.** 하드웨어·네트워크는 배제됐고, 드라이버 프로세스가 시리얼 읽기 대기 상태에서 데이터를 받지 못하는(또는
받고도 발행 못 하는) 상태로 멈춘 것까지 확인. 밤새 실행 때 건은 나중에 커널 로그로 보니 **다른 원인(전원 저하)**
이었다 — 아래 "저전압 → USB 전체 재연결" 항목. 즉 이 항목(드라이버 정지)은 지금까지 13:08 한 번.
ROBOTIS 이슈·ROS Answers 에 정확히 같은 사례는 찾지 못했다.

### 해결 (대응)
Pi 사용자 서비스 **`scan-watchdog`** (`robot/tools/scan_watchdog.py`, `robot/systemd/scan-watchdog.service`) 추가 —
bringup 은 건드리지 않고 옆에서 `/scan` 을 구독하다가 드라이버가 살아 있는데 10초 이상 끊기면:
1. 진단 기록 → `~/.ros/scan_watchdog/incidents.log`: 드라이버 rchar 속도, 스레드별 상태·wchan, 전압 플래그,
   bringup 로그, **시리얼 입력 큐에 쌓인 바이트(FIONREAD)**, 같이 읽기로 가져온 바이트
2. 복구 시도: 시리얼 1초 같이 읽기 (최대 3회, 60초 간격). 실패하면 "수동 조치 필요"를 기록
   (bringup 자동 재시작은 하지 않음 — 자이로 영점·odom 초기화·카메라 재열거를 동반하므로 사람이 판단)

검증: Pi 에서 사고 처리 경로를 강제로 실행 — 기록 항목 전부 수집, 정상 기준값(큐 0 B, 같이 읽기 210 B) 확인.

### 재발 방지 / 다음 발생 때
- `cat ~/.ros/scan_watchdog/incidents.log` 로 기록 확인. 큐가 수 KB 쌓여 있었다면 "드라이버가 시리얼을 안 읽음"이 확정.
- 자동 복구가 안 되면 sudo 로 `strace -f -p $(pgrep -f '[h]lds_laser_publisher')` 를 몇 초 떠서 메인 스레드가
  어떤 fd 에서 무엇을 기다리는지 확인 (사용자 입회).

## 2026-11-12 — 라이다가 또 끊김: Pi 저전압 → USB 전체 재연결 (전원 원인 조사 중)

### 증상
- 18:46 쯤 웹 콘솔의 라이다 수신이 다시 0. 이번엔 bringup 서비스는 '실행 중'인데 odom 도 오지 않았고,
  배터리 값은 11.78V 에서 움직이지 않았다 (서버에서 66 개 연속 같은 값 — 끊기기 직전 값이 브리지에 남은 것).
- 위 13:08 건(드라이버 정지)과 달리 `scan-watchdog` 의 incidents.log 에 아무것도 없었다.

### 진단
1. **커널 로그** (`journalctl -k`, michael 은 adm 그룹이라 sudo 불필요):
   - 09:01 부팅 후 8시간 동안 저전압 0 → **17:16 부터 `hwmon hwmon1: Undervoltage detected!` 68회**.
   - **18:46:46 `usb usb2-port1: over-current change`** → `usb 1-1.3: USB disconnect` → `cp210x ... disconnected from ttyUSB0`
     → 라이다·OpenCR·카메라가 **`ttyUSB1`·`ttyACM1`** 로 다시 잡힘 (옛 이름은 죽은 연결을 쥔 프로세스가 붙잡고 있어서).
   - `vcgencmd get_throttled` = `0x50000` (부팅 후 저전압·스로틀이 있었음).
2. **bringup 로그**:
   - `hlds_laser_publisher`: `terminate called after throwing ... boost::system::system_error what(): write: Input/output error`
     → 프로세스 종료(exit -6). 사라진 포트에 쓰다가 예외로 죽음.
   - `turtlebot3_ros`: `Failed to read[[TxRxResult] Failed transmit instruction packet!]` 를 계속 찍으며 멈춤.
   - 자체 카메라 노드: 자기 복구 로직으로 다시 붙음 (docs/realsense_bringup.md).
3. **어젯밤 건도 같은 패턴**: 지난 부팅(01:19)에서 02:30 부터 저전압 **1,055회**, 06:01 부터 OpenCR `There is no status packet!`.
4. **감시가 놓친 이유**: 첫 `scan-watchdog` 은 "드라이버가 살아 있는데 멈춤"만 판정했다. 드라이버가 죽으면 pid 가 사라져
   '판정 안 함' 상태가 됐다.
5. **bringup 재시작만으론 안 되는 이유**: `turtlebot3_bringup/robot.launch.py` 가 `/dev/ttyACM0`·`/dev/ttyUSB0` 를 쓰는데
   장치가 1번으로 다시 잡혔다. → 로봇 전원을 껐다 켜서 복구.

### 근본 원인
**Pi 5V 공급 부족 → USB 허브 과전류/리셋.** 왜 몇 시간 뒤에 시작되는지는 **미확정**:
- 끊기기 직전 배터리 11.78V(셀당 3.93V)라 배터리 방전은 아님.
- 후보 ① UBEC → GPIO 배선의 전압 강하: UBEC 출력 5.25V 인데 GPIO 실측 4.91V (Pi 경고 기준 ≈4.63V 까지 여유 0.28V).
  오래 켜 두면 접점·UBEC 발열로 순간 부하(카메라·USB) 때 기준 밑으로 내려갈 수 있음.
- 후보 ② 충전기를 꽂은 채 계속 운용 (사용자 확인: 두 건 모두 충전기 연결 상태). 충전기가 충전 단계를 바꿀 때
  공급 전압이 흔들릴 가능성. 전압 기록이 없어 아직 구분 불가.
- 추가 관찰 (22:38, 전원 기록 첫날): 전원 껐다 켠 뒤 **bringup 재시작 직후(카메라 시작 구간) 저전압 2초** — 배터리 11.76V,
  몇 시간 켜 둔 상태가 아닌데도 순간 부하만으로 기준선에 닿음 → 5V 여유가 원래 얇다는 쪽(후보 ①)에 무게.

### 해결 (대응)
1. **전원 기록** — `scan-watchdog` 확장 (`robot/tools/scan_watchdog.py`). 기록 위치 `~/.ros/scan_watchdog/`:
   - `power-YYYY-MM-DD.log`: 10초마다 `batt`(OpenCR 배터리 V), `uv_now`(Pi 저전압 센서 `rpi_volt/in0_lcrit_alarm`),
     `uv`(구간 내 커널 저전압 경고 수), `throttled`, CPU 온도, `/scan` Hz. 14일 보관.
   - `events.log`: 저전압 시작(그때 배터리 V)·해소, 커널의 USB 과전류·끊김·재연결, 시리얼 장치 목록 변경,
     라이다 드라이버 비정상 종료(정상 정지와 구분 — bringup 서비스 상태로), OpenCR 배터리 수신 끊김/재개, /scan 정지·복구.
   - `incidents.log`: 드라이버 비정상 종료 때 bringup 로그(라이다·OpenCR 줄)와 최근 3분 커널 로그도 남김.
   - 검증: Pi 에서 임시 경로로 실행(10초 기록·장치·배터리), 커널 줄 해석은 가짜 줄(저전압·과전류·끊김)로 시험.
2. **시리얼 고정 이름** — `full_bringup.launch.py` 가 OpenCR·LDS 를 `/dev/serial/by-id/...` 로 연다
   (udev 가 기본으로 만드는 장치 고유 이름이라 sudo·udev 규칙 불필요). `turtlebot3_bringup` 은 수정하지 않고
   - OpenCR: robot.launch.py 의 공식 인자 `usb_port`
   - LDS: robot.launch.py 의 공식 인자 `lidar_pkg_dir` 를 `realsense_bringup/launch/lds/` 로 → 같은 이름의 대체 런치가
     포트만 `lds_port` 로 바꿔 같은 드라이버 노드를 띄움 (robot.launch.py 는 라이다 포트를 `/dev/ttyUSB0` 로 고정해 넘기므로)
   - 검증: Pi 에서 launch 트리를 실행 없이 펼쳐 `hlds_laser_publisher port=/dev/serial/by-id/usb-Silicon_Labs_CP2102...`,
     `turtlebot3_ros -i /dev/serial/by-id/usb-ROBOTIS_OpenCR...`, `frame_id=base_scan` 확인. 인자로 옛 이름도 지정 가능.
   - 효과: USB 가 다시 잡혀도 **`systemctl --user restart tb3-bringup`** 으로 복구 (전원 껐다 켤 필요 없음).
     단 원인이 전원이면 다시 일어나므로 근본 해결은 아님.

### 재발 방지 / 다음 발생 때
- `cat ~/.ros/scan_watchdog/events.log` → 저전압이 몇 시에, 배터리 몇 V 에서 시작됐는지.
  `grep -v 'uv=0 ' ~/.ros/scan_watchdog/power-<날짜>.log` 로 저전압 구간 전후 전압 흐름.
  배터리가 높은데 저전압이면 UBEC·배선 쪽(후보 ①), 충전 상태 변화와 맞물리면 충전기 쪽(후보 ②).
- 오래 켜 둔 상태에서 GPIO 5V 를 멀티미터로 재기 (부팅 직후 4.91V 와 비교).
- 여유 확보 방법 후보: UBEC→Pi 선을 굵고 짧게(또는 USB-C 입력으로), 출력 조정 가능한 UBEC 면 5.1~5.2V 를 GPIO 에서 맞추기.

---

## 2026-11-14 — 첫 바닥 시험에서 나온 세 가지: 목표 앞 정지 · 미탐사=빈 칸 지도 · 아무도 몰랐던 AMCL 위치 잃음

첫 실주행(지도 만들기 → 저장 → 저장 지도 주행 → 초기 위치 → 클릭 주행)은 전부 동작했다. 사용자가 두 가지를 짚었고,
자동 위치 찾기(my_localization)를 만들며 하나가 더 나왔다.

### ① 클릭한 곳까지 안 가고 그 앞에서 멈춤

- **증상**: 목표 부근까지 가서 멈춘다. 정확히 찍은 곳에 서지 않는다.
- **진단**: launch 로그(`/tmp/web_api_mode_launch.log`)의 bt_navigator 기록 — 목표 (1.36, 2.05), 다음 목표를 보낼 때의
  출발 위치 (1.46, 1.80) → **27cm 앞**에서 "Reached the goal!". `nav2_params.yaml` 의
  `general_goal_checker.xy_goal_tolerance: 0.25` (Nav2·ROBOTIS 예제 기본값) — 목표 25cm 안이면 도착.
- **해결**: `xy_goal_tolerance` 0.07 (goal checker 와 DWB `FollowPath` 둘 다 — DWB 의 GoalDist·RotateToGoal critic 도 이 값을 본다),
  `yaw_goal_tolerance` 0.25 → 0.15 rad. AMCL 오차(3~5cm)보다는 크게 — 더 작으면 목표 근처에서 맴돌 수 있다.
- **남은 것**: 바닥에서 맴돎 없이 서는지 확인. 벽·장애물 바로 옆을 찍으면 planner `tolerance 0.5` 로 갈 수 있는 가장 가까운 점까지만 간다 (정상).

### ② 저장 지도의 미탐사 영역이 '빈 칸' 으로 읽힘

- **증상**: 자동 위치 찾기의 후보 칸 수가 지도의 흰 칸(1270)보다 훨씬 많은 2692. `/map` 값 분포 `{0: 3283, 100: 469}` — **-1(미탐사)이 하나도 없다.**
- **근본 원인**: nav2 `map_saver_cli` 는 미탐사를 회색 205 로 쓰고 yaml 에 `free_thresh: 0.25` 를 적는다.
  map_server 는 불러올 때 `occ = (255 − 205)/255 = 0.196 < free_thresh 0.25` → **빈 칸**. 저장·불러오기 기본값끼리 어긋나는 것
  (ROS 1 map_saver 는 0.196 을 썼다).
- **영향**: 벽 바깥 미탐사 영역까지 '빈 곳' — AMCL·전역 경로 계획이 벽 너머를 빈 땅으로 본다(벽은 막혀 있어 실주행 영향은 작았음),
  위치 찾기 후보가 벽 바깥까지 늘어 오답 위험 증가.
- **해결**: 웹 지도 저장(`maps.py`)에 `--free 0.196` (README·my_slam.md 수동 명령도). 기존 지도 `map_20261113_1833.yaml` 은
  `free_thresh: 0.196` 로 고침 → `{-1: 2013, 0: 1270, 100: 469}` (실측), 후보 칸 985.
- **재발 방지**: 지도를 새로 저장한 뒤 `/map` 에 -1 이 있는지 (위치 찾기 로그의 "후보 칸" 수가 흰 영역과 비슷한지) 확인.

### ③ AMCL 이 위치를 잃은 채 아무도 몰랐다

- **발견**: 자동 위치 찾기 첫 실기 시험 직전, 시험 주행 후 정지해 있던 로봇의 AMCL 위치 (1.03, 1.92) −168°.
  자동 위치 찾기가 찾은 위치는 (0.95, 1.80) −69° — 방향이 99° 다르다. `tools/score_pose.py` 로 지금 스캔의 일치 점수를 비교:
  AMCL 쪽 **0.08**, 찾은 쪽 **0.93**. AMCL 이 이미 틀린 곳에 수렴해 있었다.
- **원인**: 확정 못 함 — 손으로 찍은 초기 위치 오차, 주행 뒤 로봇을 옮김 등. AMCL 은 스스로 "틀렸다" 를 알리지 않는다.
- **대응**: my_localization 이 1초마다 추적 점수를 계산해 4초 동안 0.3 미만이면 '위치 잃음' → 지도 전체 검색으로 바로잡는다.
  웹 지도 머리줄 칩에 상태·점수 표시 (docs/my_localization.md).
