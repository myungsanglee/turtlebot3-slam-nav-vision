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
| 종료한 노드의 토픽이 `topic list` 에 계속 보임 | ros2 daemon 의 그래프 캐시 (표시 문제일 뿐 실제 잔존 아님) | `ros2 daemon stop && ros2 daemon start` 또는 `ros2 topic list --no-daemon` |
| ROS 카메라 노드만 `UVCIOC_CTRL_QUERY Connection timed out` (pyrealsense2 는 됨) | ROS 래퍼가 apt 커널 백엔드 librealsense 사용, Pi 커널에 패치 없음 | realsense-ros 를 RSUSB 백엔드 librealsense 로 소스 빌드 (아래 2026-08-27 항목) |
| 카메라 토픽은 discover 되는데 데이터 수신 0 (라이다는 됨) | Fast DDS Discovery Server 가 VPN+멀티홈+복잡 참가자(카메라)에서 데이터 전달 실패 | Zenoh 전환 (아래 2026-08-29 항목) |

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
