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
| `ros2 topic list` 에 원격 토픽이 안 보임 (`/parameter_events`, `/rosout` 만) | Discovery Server 환경에서 CLI(daemon)가 일반 client → 매칭되는 것만 봄 | `export ROS_SUPER_CLIENT=TRUE` 후 `ros2 daemon stop && ros2 daemon start` |
| 토픽 목록엔 보이는데 `ros2 topic hz` 수신율 0 | 디스커버리(메타데이터)는 되나 **유저 데이터 P2P 경로**가 막힘 (방화벽/멀티홈 locator) | 아래 2026-08-25 항목 참조 |
| RViz 에서 `/scan` 이 안 보임 | QoS 불일치 (`/scan` 은 BEST_EFFORT) | 구독 Reliability 를 Best Effort 로 |
| RViz 에서 `/map` 이 늦게/안 보임 | QoS Durability 불일치 | Transient Local 로 |
| bringup 이 `robot_description as yaml` 오류로 안 뜸 | URDF 주석에 콜론+공백/줄끝 콜론 → YAML 파싱 깨짐 | 주석에서 콜론 제거 (아래 2026-08-26 항목) |

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
