# description — 커스텀 로봇 URDF 보정

> 로봇 분야를 전혀 모르는 사람도 이 문서와 코드를 같이 보면
> "무엇이 어떻게 계산되고, 그래서 값을 왜 이렇게 바꿨는지"를
> 이해할 수 있도록 쓴 문서다. 좌표계/TF 기초는
> [my_slam.md](./my_slam.md) 2장을 먼저 읽는 것을 권장.

## 1. 이게 왜 필요한가 (한 줄 요약)

**이 로봇은 ROBOTIS 표준 burger 와 센서 위치가 다르다. 표준 URDF 를 그대로
쓰면 "라이다가 로봇 어디에 붙어 있는지"를 잘못 알아서 SLAM 지도가 어긋난다.**
그래서 실측값으로 센서 위치를 보정한 URDF 를 만들어 둔다.

## 2. URDF 가 하는 일 — "부품 조립도"가 "좌표 변환"이 된다

URDF(Unified Robot Description Format)는 로봇을 **링크(link, 뼈대=좌표계)** 와
**조인트(joint, 두 링크의 연결)** 로 기술하는 XML 이다. 각 fixed 조인트의

```xml
<origin xyz="x y z" rpy="roll pitch yaw"/>
```

한 줄은 **"자식 링크가 부모 링크 기준으로 (xyz)만큼 떨어져 (rpy)만큼 회전돼 있다"**
는 뜻이고, 이것이 그대로 `/tf_static` 좌표 변환이 된다. 즉 URDF 의 숫자 = TF 트리의
고정 변환값이다. (움직이는 변환인 odom→base_footprint 는 URDF 가 아니라 바퀴
오도메트리가 만든다 — my_slam.md 참고.)

**좌표 규약(ROS REP-103)**: 단위는 미터(m)·라디안(rad). x=앞(+)/뒤(−),
y=왼쪽(+)/오른쪽(−), z=위(+)/아래(−). rpy 중 yaw 는 z축 회전(위에서 본 좌우 돌기),
**위에서 봤을 때 반시계가 +, 시계가 −**.

## 3. 표준 대비 무엇을 바꿨나

기준점 `base_link` 는 **좌우 바퀴 축의 중심**에 있다(바퀴가 x=0 에 있으므로).
따라서 모든 센서 위치는 "바퀴 축 중심"에서 잰 값이다.

| 조인트 | 항목 | 표준값 | **보정값(실측)** | 왜 바꿨나 |
|---|---|---|---|---|
| `scan_joint` | LDS 위치 xyz | `-0.032, 0, 0.172` | **`-0.100, 0, 0.125`** | 라이다를 뒤로·낮게 개조 |
| `imu_joint` | IMU 회전 rpy | `0, 0, 0` | **`0, 0, -1.57`** | OpenCR 을 시계방향 90° 돌려 장착 |
| `camera_mount_joint` (신규) | RealSense 마운트 나사 구멍 xyz | (표준엔 없음) | **`0.0475, 0, 0.048`** | D435i 를 앞쪽 위에 장착 (2026-09-08 실측) |

바꾸지 않은 것: `wheel_*_joint`/`base_joint`(바퀴 폭이 표준과 동일 → 오도메트리
파라미터 유지), `caster_back_joint`(센싱 무관), 각 링크의 visual/collision/inertial
(RViz 모델 형상·시뮬레이션용이라 실물 SLAM/Nav 계산엔 무관).

### 3.1 scan_joint — LDS 라이다 위치 (SLAM 품질의 핵심)

보정값 `xyz="-0.100 0 0.125"` 의 의미:
- **x = −0.100**: 바퀴 축(회전 중심)에서 **뒤로 100mm**. (표준은 32mm)
- **y = 0**: 로봇 중심선상 (좌우 치우침 없음).
- **z = 0.125**: base_link 기준 **125mm 위**. base_link 가 바닥에서 10mm 위에
  있으므로, LDS 빔 평면은 **바닥에서 약 135mm** (표준 ~182mm 보다 낮음).

**왜 x 가 특히 중요한가 (회전 시 벽 이중선의 원리)**: 로봇이 제자리 회전하면
회전 중심은 바퀴 축이다. LDS 가 실제로는 축에서 100mm 뒤에 있는데 URDF 엔
32mm 로 적혀 있으면, 회전할 때마다 스캔이 실제와 다른 반경으로 원을 그리며
어긋난다. slam_toolbox 는 같은 벽을 조금씩 다른 위치에 여러 번 그려서 **벽이
두 겹으로 번지거나 지도가 밀린다.** 이 값을 실측으로 맞추는 것이 보정의 핵심.

(z 높이는 2D SLAM 정확도엔 상대적으로 영향이 작지만 — 스캔이 2D 로 투영되므로 —
추후 카메라/센서 정합과 3D 일관성을 위해 정확히 반영해 둔다.)

### 3.2 imu_joint — IMU(OpenCR) 장착 회전

OpenCR 보드를 **위에서 봤을 때 시계방향 90°** 회전하여 고정했으므로 yaw = −90°
= **−1.57 rad**. IMU 데이터(방향/각속도)는 imu_link 프레임 기준으로 해석되는데,
이 프레임이 실제 장착 방향과 어긋나면 융합 결과가 틀어진다.

- ★ **위치(xyz)는 아직 미실측**이라 표준값(`-0.032 0 0.068`)을 유지했다.
  IMU 는 현재 SLAM 에서 쓰지 않으므로 당장 문제는 없지만,
  **robot_localization(EKF) 단계 전에 IMU 위치도 실측하여 교체**해야 한다.

### 3.3 카메라 — 실측은 한 점, 렌즈 위치는 인텔 공식 오프셋

RealSense D435i 는 렌즈가 여럿(RGB, 왼쪽/오른쪽 IR, 프로젝터)이라 렌즈를 직접 재면 오차와
혼동이 생긴다. 그래서 **바닥의 1/4인치 마운트 나사 구멍(=몸체 정중앙)** 한 점만 실측하고,
그 아래는 인텔 `realsense2_description` 의 `_d435.urdf.xacro` 상수를 그대로 옮겼다:

```
base_link ─(실측 47.5, 0, 48mm)─▶ camera_bottom_screw_frame
   ─(+10.6, +17.5, +12.5)─▶ camera_link            = 왼쪽 IR 이미저 = depth 원점
        ├─(0, 0, 0)─▶ camera_depth_frame ─(광학 회전)─▶ camera_depth_optical_frame
        └─(0, +15, 0)─▶ camera_color_frame ─(광학 회전)─▶ camera_color_optical_frame  ★ 우리 노드의 frame_id
```
- 카메라 시점에서 왼쪽→오른쪽 순서는 **RGB — 왼쪽 IR — 프로젝터 — 오른쪽 IR** 이고 RGB 는 왼쪽 IR 의
  15mm 왼쪽이다. 우리 카메라의 공장 익스트린식(캘리브레이션 캐시, depth→color 14.9mm)과 일치.
- **광학 프레임(optical)**: 이미지 처리 규약(z 앞, x 오른쪽, y 아래)으로 축만 돌린 프레임. 자체 카메라
  노드가 내보내는 정렬 depth·camera_info·검출 3D 위치는 전부 `camera_color_optical_frame` 기준이다.
  URDF 의 `rpy="-1.5708 0 -1.5708"` 이 ROS 몸체 규약(x 앞, y 왼쪽, z 위)을 광학 규약으로 바꾸는 회전.
- 결과 (base_link 기준): RGB 렌즈 = (58.1, 32.5, 60.5) mm. 수평 장착 가정(pitch 0) — 기울어져
  있으면 `camera_mount_joint` 의 pitch 하나만 고치면 된다.
- 이 TF 가 있어야 Vision 노드의 "카메라 좌표 3D 위치"를 `base_link`/`map` 좌표로 옮길 수 있다.

## 4. 배포 방법 — bringup 은 건드리지 않는다

TF 를 publish 하는 `robot_state_publisher` 는 **Pi 의 turtlebot3_bringup** 이
띄우며, `turtlebot3_description` 패키지의 `turtlebot3_burger.urdf` 를 로드한다.
bringup 의 코드/런치는 수정하지 않는 것이 원칙이므로, **로드되는 URDF 파일만**
이 보정판으로 교체한다:

```bash
# Pi 에서 (git pull 로 이 레포를 받은 뒤) — 이 Pi 는 소스 빌드 + symlink-install
# 이라 src 의 원본 파일을 교체하면 install 에 자동 반영된다 (pi_setup.md 8단계)
TARGET=~/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf
cp "$TARGET" "$TARGET.orig"      # 원본 백업
cp ~/turtlebot3-slam-nav-vision/description/urdf/turtlebot3_burger.urdf "$TARGET"

# 이후 bringup 을 (재)실행하면 보정된 TF 가 나온다
ros2 launch turtlebot3_bringup robot.launch.py
```

> `TURTLEBOT3_MODEL=burger` 를 그대로 쓰므로 모델명 변경은 필요 없다.
> ⚠️ URDF **주석에 콜론+공백/줄끝 콜론 금지** — bringup 이 robot_description 을
> YAML 로 파싱하다 깨진다 (troubleshooting.md 2026-08-26 사건). 배포 전
> `python3 -c "import yaml; yaml.safe_load(open('<파일>').read())"` 로 검사.

## 5. 검증 기록

**파싱·로드 검증 (2026-07-22, Remote PC 컨테이너)**
- `xacro` 파싱 성공, `robot_state_publisher` 로드 성공 — 모든 세그먼트 인식
- 파싱 결과에서 보정값 확인:
  `scan_joint origin xyz="-0.100 0 0.125" rpy="0 0 0"`,
  `imu_joint origin rpy="0 0 -1.57"`

**Pi 배포 + TF 실기 검증 (2026-08 완료)**
- 배포 중 주석 콜론으로 bringup YAML 파싱 실패 발생 → 주석 수정으로 해결
  (troubleshooting.md 2026-08-26)
- 배포 후 서버에서 실기 확인:
  `tf2_echo base_link base_scan` → Translation `[-0.100, 0.000, 0.125]` ✓
  `tf2_echo base_link imu_link` → RPY `[0, 0, -1.570]` ✓
- SLAM 대략 주행에서 지도 정상 생성 확인. **제자리 회전 정밀 검증(벽 이중선
  여부)은 공간 확보 시 진행** — 벽이 회전돼 보이면 scan_joint 의 yaw 를
  경험적으로 보정.

**IMU 회전(yaw=-1.57) 물리 검증 (2026-09-02 완료)**
- 정지 상태: 가속도 z=+9.97 m/s² (중력) → 보드 수평(roll/pitch=0) 확인.
  y 축에 +0.7 바이어스 관찰 — 약 4° 기울기 또는 센서 오프셋, EKF 단계에서
  바이어스 보정으로 처리 (기록용).
- 축 반응 테스트 (rqt_plot 으로 /imu/linear_acceleration 관찰):
  로봇을 **앞으로 밀면 y 축**이, **좌우로 흔들면 x 축**이 반응 —
  "위에서 봤을 때 시계방향 90° 회전 장착"과 정확히 일치.
  → **URDF yaw=-1.57 이 실물과 부합함을 물리적으로 확정.**
- 남은 것: IMU **위치(xyz)** 는 여전히 표준값(미실측) — EKF 전 실측 교체.

**RealSense 카메라 TF (2026-09-08 실측·추가)**
- base_link → 마운트 나사 구멍 실측 x +47.5, y 0, z +48 mm. xacro·check_urdf 통과, 합성 변환 확인
  (RGB 렌즈 58.1, 32.5, 60.5 mm / optical z 축이 로봇 앞을 향함).
- Pi 배포(2026-09-08) 후 bringup 정상 기동, 서버에서 실기 확인:
  `tf2_echo base_link camera_color_optical_frame` → Translation `[0.058, 0.033, 0.060]`,
  RPY `[-1.571, 0, -1.571]` ✓ / `camera_link` → `[0.058, 0.018, 0.060]` ✓ / `base_scan` 기존값 유지 ✓
  (기울기는 수평 가정 — 실측 시 pitch 반영).

## 6. 다음 단계

1. **제자리 회전 정밀 검증** — 공간 확보 시 (위 검증 기록 참고).
2. **카메라 기울기 확인** — 수평 가정. 숙여 장착돼 있으면 `camera_mount_joint` pitch 반영.
3. **IMU 위치(xyz) 실측 교체** — robot_localization(EKF) 전.
4. **Nav2 footprint 실측 반영** — 로봇 외형이 표준과 다르므로 nav2_params.yaml 의
   robot_radius(임시 0.105) 를 실측 다각형 footprint 로 교체.
