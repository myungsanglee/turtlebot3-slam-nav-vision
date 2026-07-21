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

## 4. 배포 방법 — bringup 은 건드리지 않는다

TF 를 publish 하는 `robot_state_publisher` 는 **Pi 의 turtlebot3_bringup** 이
띄우며, `turtlebot3_description` 패키지의 `turtlebot3_burger.urdf` 를 로드한다.
bringup 의 코드/런치는 수정하지 않는 것이 원칙이므로, **로드되는 URDF 파일만**
이 보정판으로 교체한다:

```bash
# Pi 에서 (git pull 로 이 레포를 받은 뒤)
#  설치 위치는 환경에 따라 다름 — 아래 중 실제 존재하는 경로로 복사
#   - apt 설치:   /opt/ros/humble/share/turtlebot3_description/urdf/
#   - 소스 빌드:  ~/turtlebot3_ws/install/turtlebot3_description/share/turtlebot3_description/urdf/
cp description/urdf/turtlebot3_burger.urdf \
   /opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf

# 이후 bringup 을 (재)실행하면 보정된 TF 가 나온다
ros2 launch turtlebot3_bringup robot.launch.py
```

> 원본을 덮어쓰므로, 교체 전 원본을 백업(`*.orig`)해 두길 권장한다.
> `TURTLEBOT3_MODEL=burger` 를 그대로 쓰므로 모델명 변경은 필요 없다.

## 5. 검증 기록 (2026-07-22)

Remote PC 컨테이너에서 파싱·로드 검증 완료:
- `xacro` 파싱 성공 (XML 문법/구조 정상)
- `robot_state_publisher` 로 로드 성공 — 모든 세그먼트 인식
  (base_footprint, base_link, base_scan, imu_link, wheel_left/right, caster_back)
- 파싱 결과에서 보정값 확인:
  `scan_joint origin xyz="-0.100 0 0.125" rpy="0 0 0"`,
  `imu_joint origin rpy="0 0 -1.57"`
- ★ 실물 로봇에 배포 후 "제자리 회전 시 벽 이중선이 사라지는지" 실주행 검증은
  Pi 배포 후 진행 예정.

## 6. 다음 단계

1. **실물 배포 + 실주행 검증** — Pi 에 URDF 교체 후 제자리 회전시켜 지도 번짐 확인.
   여전히 벽이 회전돼 보이면 scan_joint 의 yaw 를 경험적으로 보정.
2. **IMU 위치(xyz) 실측 교체** — robot_localization(EKF) 전.
3. **RealSense camera_link 추가** — base_link → camera_link static transform 을
   실측값으로 이 URDF 에 추가 (카메라 도입 시).
4. **Nav2 footprint 실측 반영** — 로봇 외형이 표준과 다르므로 nav2_params.yaml 의
   robot_radius(임시 0.105) 를 실측 다각형 footprint 로 교체.
