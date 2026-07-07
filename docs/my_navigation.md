# my_navigation — Nav2 기반 자율주행 패키지

> 로봇 분야를 전혀 모르는 사람도 이 문서와 코드를 같이 보면
> "무엇이 어떻게 계산되고, 그래서 코드를 왜 이렇게 작성했는지"를
> 이해할 수 있도록 쓴 문서다. 좌표계/TF/QoS 등 기초 개념은
> [my_slam.md](./my_slam.md) 2장을 먼저 읽는 것을 권장.

## 1. 이 패키지가 하는 일 (한 줄 요약)

**RViz 에서 지도 위 한 점을 찍으면, 로봇이 장애물을 피해 그 지점까지 스스로
찾아간다.** 기본 모드는 SLAM(my_slam)과 동시 실행이라 **지도를 만들면서
동시에 주행**한다 — 이 프로젝트의 최종 지향점(로봇청소기)의 뼈대다.

SLAM 과 Navigation 의 관계를 사람에 비유하면:

- **SLAM (my_slam)** = "여기가 어디고, 나는 지금 어디에 있는가" (지도 + 내 위치)
- **Navigation (my_navigation)** = "그래서 저기까지 **어떻게** 가는가" (경로 + 조종)

```
[입력]                          [Nav2 (이 패키지)]                    [출력]
/map  ← my_slam 이 만드는 지도 ──▶  코스트맵 생성                  /cmd_vel ──▶ 로봇 바퀴
/scan ← 라이다 (실시간 장애물) ──▶  전역 경로 계획 (planner)
TF    ← 지도 기준 내 위치      ──▶  실시간 조종     (controller)
목표점 ← RViz 에서 클릭        ──▶  막히면 복구     (behaviors)
```

## 2. 사전 지식 — Nav2 의 핵심 개념 5가지

### 2.1 코스트맵(costmap): "지도"를 "주행 관점의 위험도 지도"로 변환

SLAM 이 만드는 지도는 "벽이 있다/없다"만 담는다. 주행하려면 여기에
**"로봇이 여기로 지나가도 되는가"** 라는 판단이 얹혀야 한다. 그것이 코스트맵이다.
각 픽셀에 0(안전)~254(충돌) 비용을 매긴 지도이며, 레이어를 겹쳐 만든다:

```
static_layer    : SLAM/저장 지도의 벽          → 비용 254 (절대 불가)
obstacle_layer  : 라이다에 지금 보이는 장애물   → 비용 254 (지도에 없던 의자, 사람 등)
inflation_layer : 위 장애물 주변을 부풀림       → 벽에서 멀수록 비용 감소
```

**inflation(부풀림)이 필요한 이유**: 로봇은 점이 아니라 크기가 있는 물체다.
벽에 딱 붙은 경로를 만들면 몸통이 긁는다. 그래서 장애물 주변 `inflation_radius`
(0.55m) 안에 "가능하면 피하라"는 완충 비용을 깐다. 경로는 자연히 벽에서
여유를 두고 그려진다.

**footprint**: 코스트맵이 충돌 판정에 쓰는 "로봇의 외형"이다. 현재는 표준
burger 반경 0.105m 원으로 **임시** 설정했다(★). 이 로봇은 낮고 길게 개조되어
있으므로 실측 후 다각형으로 교체해야 한다 — 외형보다 작게 잡으면 실제로는
부딪히고, 크게 잡으면 좁은 통로를 못 지나간다.

코스트맵은 두 개를 따로 돌린다:

| | global_costmap | local_costmap |
|---|---|---|
| 범위 | 지도 전체 | 로봇 주변 3×3m (따라다님) |
| 기준 frame | `map` | `odom` (지도 보정 점프에 안 흔들리게) |
| 용도 | 전역 경로 계획 | 실시간 회피 |
| 갱신 | 1Hz | 5Hz |

### 2.2 전역 플래너(planner): 지도에서 길 찾기

"현재 위치 → 목표점"의 경로를 global costmap 위에서 계산한다. 우리는 NavFn
플러그인(다익스트라 알고리즘)을 쓴다 — 내비게이션 앱이 지도에서 길을 찾는 것과
같은 문제다. 비용이 낮은 픽셀을 따라 최단 경로를 찾으므로, inflation 덕에
자연스럽게 벽과 거리를 둔 경로가 나온다.
`allow_unknown: true` 로 설정했다 — SLAM 동시 주행에서는 아직 탐사 안 한
회색 영역으로도 경로를 계획할 수 있어야 하기 때문이다(로봇청소기가 미지의
방으로 들어가는 상황).

### 2.3 로컬 플래너/컨트롤러(controller): 경로를 바퀴 속도로

전역 경로는 "선"일 뿐이다. 이걸 따라가는 **실제 속도 명령**을 만드는 것이
controller(우리는 DWB 플러그인)다. DWB 의 동작 원리가 재미있다:

1. 지금 낼 수 있는 속도 후보를 수백 개 샘플링 (전진 20단계 × 회전 40단계)
2. 각 후보로 1.5초간 움직이면 어떻게 되는지 시뮬레이션
3. 여러 심사위원(critic)이 채점: 경로에 가까운가(PathAlign), 장애물과 먼가
   (BaseObstacle), 목표를 향하는가(GoalAlign) 등 — 파라미터의 `critics` 목록
4. 최고 점수 후보를 `/cmd_vel` 로 출력. 이걸 초당 10번 반복

즉 **매 순간 "가능한 미래 수백 개를 그려보고 최선을 고르는"** 방식이다.
지도에 없던 장애물이 나타나도 로컬 코스트맵에 반영되는 즉시 다음 계산부터
피해 간다.

### 2.4 행동 트리(BT Navigator)와 복구 동작(behaviors)

전체 흐름을 지휘하는 것은 **행동 트리(behavior tree)** 다. 기본 트리의 논리:

```
목표 수신 → [경로 계산 → 경로 추종] 반복 (1초마다 재계획)
              └─ 실패하면 → 복구 동작: 코스트맵 초기화 → 제자리 회전(spin)
                            → 후진(backup) → 대기(wait) → 재시도
```

복구 동작이 중요한 이유: 실제 환경에서는 반드시 막히는 순간이 온다(사람이
지나감, 라이다 노이즈로 유령 장애물이 낌 등). 그때 포기하지 않고 "시야를
새로고침하고(회전), 빠져나와서(후진), 다시 시도"하는 것이 자율주행의 강건함을
만든다.

### 2.5 lifecycle 노드와 cmd_vel 파이프라인

Nav2 노드들은 일반 노드와 달리 **lifecycle(생명주기) 노드**다 — 생성(Creating)
→ 구성(Configuring) → 활성(Activating) 단계를 거치며, `lifecycle_manager` 가
전체를 순서대로 깨운다. 로그에서 본 "Managed nodes are active" 가 "전원 켜짐"
신호다. 산업 표준다운 설계로, 시스템 전체를 안전하게 켜고 끌 수 있다.

최종 속도 명령이 로봇까지 가는 길 (원격 구조 주의):

```
controller(DWB) → /cmd_vel_nav → velocity_smoother → /cmd_vel → [Tailscale] → Pi → 바퀴
```

velocity_smoother 는 급가속/급정지를 하드웨어 한계(가감속 제한) 안으로
다듬는다. 모터 보호 + 오도메트리 미끄러짐 방지 효과.

## 3. 코드 구조와 사용 패키지

```
remote_pc/src/my_navigation/
├── package.xml / CMakeLists.txt        # 설정/런치만 담는 패키지 (소스 코드 없음)
├── config/
│   └── nav2_params.yaml                # ★ Nav2 전체 노드의 파라미터 (핵심 파일)
├── launch/
│   └── navigation.launch.py            # SLAM 동시 실행(기본) / 저장 지도 모드
└── rviz/
    └── nav.rviz                        # 코스트맵/경로/footprint 시각화 설정
```

my_slam 과 마찬가지로 소스 코드가 없다 — 검증된 표준 스택(Nav2)을 파라미터로
우리 로봇에 맞게 조립한다. 사용 패키지:

| 패키지 | 역할 |
|---|---|
| **nav2_bringup** | Nav2 표준 런치(navigation_launch.py, localization_launch.py) 재사용 |
| nav2_planner (NavFn) | 전역 경로 계획 (다익스트라) |
| nav2_controller (DWB) | 로컬 플래너 — 속도 샘플링+시뮬레이션+채점 |
| nav2_costmap_2d | global/local 코스트맵 |
| nav2_behaviors / bt_navigator | 복구 동작 / 행동 트리 지휘 |
| nav2_velocity_smoother | cmd_vel 을 가감속 한도로 다듬기 |
| nav2_amcl + nav2_map_server | 저장 지도 모드에서의 위치추정 (모드 2에서만) |
| my_slam | 기본 모드에서 지도+위치 제공 (slam_toolbox) |

## 4. 파라미터 작성 기준 — 어디서 온 값인가

`nav2_params.yaml` 은 두 출처를 합쳐 만들었다:

1. **구조/기본값**: Humble 공식 `nav2_bringup` 기본 파라미터.
   (컨테이너의 ROBOTIS turtlebot3_navigation2 파라미터는 구버전 잔재가 있어
   구조는 공식 쪽을 신뢰: `recoveries_server`(Galactic 명칭), `use_sim_time: True`
   오기 등을 확인함)
2. **로봇 고유 수치**: ROBOTIS burger 검증값 — 최대속도 0.22m/s,
   차동구동이라 y 방향 샘플 0, 회전 샘플 40, 제어주기 10Hz.

우리가 의도적으로 바꾼 것:

| 파라미터 | 값 | 이유 |
|---|---|---|
| `robot_base_frame` | `base_footprint` | slam_toolbox 와 프레임 규약 통일 (프로젝트 공통) |
| `controller_frequency` | 10.0 | ROBOTIS TB3 검증값. cmd_vel 이 VPN 을 건너는 원격 구조라 과한 주기보다 안정성 |
| `max_vel_x`, `max_velocity` | 0.22 | burger 하드웨어 한계와 일치 (velocity_smoother 까지 동일하게) |
| local costmap `plugins` | obstacle+inflation | 2D 라이다만 있으므로 3D용 voxel_layer 제외 |
| `allow_unknown` | true | 미탐사 영역 주행 허용 (SLAM 동시 주행에 필수) |
| `robot_radius` | **0.105 ★임시** | 표준 burger 값. 실측 후 다각형 footprint 로 교체 예정 |

## 5. 실행 방법

### 모드 1 — SLAM 하면서 주행 (기본)

```bash
docker compose exec remote-pc bash
export DISPLAY=:1        # VNC 세션에 RViz 표시
ros2 launch my_navigation navigation.launch.py
```

RViz 상단 툴바의 **"2D Goal Pose"** 를 클릭하고 지도 위 목표 지점을 찍으면
(누른 채 드래그 = 도착 방향 지정) 로봇이 출발한다. 내부적으로는
RViz → `/goal_pose` 토픽 → bt_navigator 가 받아 행동 트리 시작.

> ⚠️ 목표를 찍는 순간 **실제 로봇이 움직인다.** 처음에는 로봇 주변을
> 확보하고 짧은 거리부터 테스트할 것.

### 모드 2 — 저장된 지도로 주행 (지도 완성 후)

```bash
ros2 launch my_navigation navigation.launch.py \
    use_slam:=false map:=/overlay_ws/maps/my_map.yaml
```

slam_toolbox 대신 map_server(저장 지도) + AMCL(파티클 필터 위치추정)이 뜬다.
이때는 RViz "2D Pose Estimate" 로 초기 위치를 알려줘야 AMCL 이 수렴한다.

## 6. 검증 기록 (2026-07-07, 실물 로봇 연결 상태)

- colcon 빌드 성공 (my_slam → my_navigation 의존 순서 인식)
- 전체 lifecycle 활성화 확인: controller → smoother → planner → behaviors →
  bt_navigator → waypoint_follower → velocity_smoother → **"Managed nodes are active"**
- `/global_costmap/costmap` publish 확인 (해상도 0.05m)
- `/scan` 5Hz 수신 확인 (SLAM 과 동시 동작)
- 실제 목표점 주행은 미실시 — 로봇이 물리적으로 움직이는 테스트라 사용자 입회
  하에 진행하기로 함

## 7. 트러블슈팅

| 증상 | 원인/해결 |
|---|---|
| RViz 에서 코스트맵이 안 보임 | Map 디스플레이 Durability 를 Transient Local 로 (nav.rviz 반영됨) |
| 목표를 찍어도 안 움직임 | ① lifecycle 활성화 확인 (`Managed nodes are active` 로그) ② TF 확인: map→odom(slam), odom→base_footprint(로봇) 둘 다 있어야 함 |
| 좁은 통로에서 경로가 안 나옴 | inflation_radius(0.55) 가 통로 폭 대비 과함 → 낮추거나 footprint 실측 반영 |
| 주행이 뚝뚝 끊김 | 원격(VPN) 지연 가능성. controller_frequency 를 낮추거나 (이미 10Hz) 네트워크 상태 확인 |
| 벽에 스치거나 좁은 곳을 못 지나감 | footprint 가 실물과 다름 — ★ 실측 교체 (4장) |
| CLI 로 디버깅 시 토픽 안 보임 | `export ROS_SUPER_CLIENT=TRUE` (my_slam.md 트러블슈팅 참고) |

## 8. 현재 한계와 다음 단계

1. **footprint 실측 교체 (★ 최우선)** — 로봇 외형(전장×전폭, base_footprint
   원점 기준 꼭짓점)을 실측해 `robot_radius` 를 다각형 `footprint` 로 교체.
2. **URDF 보정과 연동** — LDS 위치 실측 보정(description/ 작업)이 끝나면
   지도 품질이 올라가고 코스트맵 정확도도 함께 개선됨.
3. **실주행 튜닝** — 실제 목표점 주행에서 DWB critic 가중치, inflation,
   goal tolerance 를 환경에 맞게 조정.
4. **robot_localization(EKF)** — LiDAR+IMU+엔코더 융합으로 odom 안정화 (로드맵).
5. **Vision AI 연동** — 카메라 인식 결과를 코스트맵/행동에 반영 (로드맵).
