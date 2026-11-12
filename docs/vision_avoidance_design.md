# 카메라 기반 장애물 회피 설계 — 깊이 층 + 의미(Vision AI) 층

> 상태: 설계 (2026-11-09) → **V1 깊이 층 구현 (2026-11-12, 책상 검증 · 바닥 실측 조정 남음)**. 구현하면서 바뀐 결정은 이 문서를 고친다.
> 관련: [my_navigation.md](./my_navigation.md) (현재 costmap 구성), [my_vision.md](./my_vision.md) (검출·3D 위치),
> [web_ui_design.md](./web_ui_design.md) (결과를 화면에 보여주는 쪽).

## 1. 문제

지금 Nav2 가 장애물을 피할 때 쓰는 입력은 **라이다(`/scan`) 하나**다 (local/global costmap 의 obstacle_layer).

- 라이다는 바닥 위 **125mm 높이의 평면 한 장**만 본다 (base_scan z=0.125).
  → 그보다 **낮은 것**(문턱·신발·전선·작은 상자)과 **위로 튀어나온 것**(책상 상판·의자 좌판·선반)을 못 본다.
- 카메라(D435i)는 이미 검출·거리 계산에 쓰이지만 **회피에는 관여하지 않는다.**

로봇청소기 같은 실내 주행이 목표이므로 이 빈틈을 카메라로 메운다.

## 2. 왜 두 층인가

검출(Vision AI)만으로 피하게 하면 생기는 빈틈:

| 한계 | 내용 |
|---|---|
| 모르는 물체 | 모델은 학습한 클래스만 검출(COCO 80종). 상자·문턱·전선·벽 모서리는 검출 안 됨 → 그대로 충돌 |
| 크기 정보 부족 | 현재 `/vision/objects` 는 박스 중앙 depth 의 **한 점**. 폭·깊이는 따로 추정해야 함 |
| 시야 이탈 | 카메라 화각은 컬러 기준 **약 69°**(depth 를 컬러에 정렬해 씀). 회전하면 물체가 화면 밖으로 나가 잊힘 |

그래서 역할을 나눈다.

| 층 | 대상 | 입력 | 역할 |
|---|---|---|---|
| **① 깊이 층** (geometric) | 카메라에 보이는 **모든 것** | 정렬 depth → PointCloud2 | **안전**: "뭔가 있다"면 무조건 피함 |
| **② 의미 층** (semantic) | **무엇인지 아는 물체** | `/vision/objects` (클래스·3D 위치·크기) | **행동**: 물체 종류에 따라 다르게 피함·감속 |

①이 바닥(안전)을 깔고, ②가 그 위에 "사람이니까 더 넓게, 천천히" 같은 판단을 얹는다.
업계 용어로 semantic costmap / social navigation 에 해당한다.

## 3. ① 깊이 층

```
/camera/depth/compressed (PNG16, mm, 컬러 정렬) + /camera/color/camera_info
      │ (서버)
      ▼
 depth → 점 구름 변환 노드 (my_vision 안 또는 depth_image_proc)
   - 픽셀 솎기 (예: 4px 간격 → 160x120 ≈ 1.9만 점)
   - 거리 범위 0.2~3.0 m, 높이 필터는 costmap 쪽에서
      │ /camera/points (sensor_msgs/PointCloud2, camera_color_optical_frame)
      ▼
 local_costmap: obstacle_layer 에 observation source 추가
   - min_obstacle_height ≈ 0.03 m  (바닥을 장애물로 찍지 않게 — 실측으로 조정)
   - max_obstacle_height ≈ 0.25 m  (로봇 키 + 여유)
   - marking / clearing 둘 다 true (사람이 지나간 자리 지우기)
```

- 변환은 서버에서 한다 (Pi 부하·브리지 대역폭 증가 없음 — 이미 depth 를 받고 있음).
- `depth_image_proc` 은 raw Image 입력이라 compressed 를 풀어 다시 넣는 과정이 필요하다. my_vision 이 이미
  depth 를 디코딩하고 deproject 하므로(`camera_io.deproject`) **my_vision 에 점 구름 발행을 추가**하는 쪽이 단순할 수
  있다 — 구현 시 비교.
- 검토: 6fps 갱신이 회피에 충분한가 (로봇 최대 0.22 m/s → 프레임당 3.7cm 이동, 충분할 것으로 예상),
  바닥 오검출(카메라 높이 60mm·수평 장착이라 바닥이 시야 아래쪽에 크게 잡힘), 반사·검은 물체의 depth 결측.

### 3.1 구현 결과 (V1, 2026-11-12)

설계에서 바뀐/정해진 것:
- 점 구름은 **my_vision 의 별도 노드 `depth_cloud_node`** (검출 노드에 붙이지 않음 — Vision 을 꺼도 회피는 동작해야 함).
  depth_image_proc 대신 직접 디코딩+역투영. stride 4, 0.2~3.0m. 6Hz, 약 1.1만 점, CPU 9%. (my_vision.md 4.7)
- costmap 에는 라이다 obstacle_layer 에 소스를 더하지 않고 **별도 `camera_layer`(ObstacleLayer)** 로 넣었다 — 카메라의
  지우기 광선이 라이다 장애물을 지우지 않게. 같은 토픽을 `camera_mark`(4~25cm, 찍기만) / `camera_clear`(바닥 포함, 지우기만)
  두 소스로 나눴다. **local·global 둘 다** 넣음 (경로 계획도 낮은 장애물을 알아야 같은 곳으로 계속 경로를 내지 않음).
- 높이: max 25cm(로봇 CAD 키 14.3cm + 여유), min **4cm(임시)** — 책상 위 측정에서 가까운 면이 −2~−3.5cm.
  ★ 바닥 시험에서 실측: 바닥 점 높이 분포(거리별)를 보고 min 을 낮출 수 있는지 판단.
- 끄기: `navigation.launch.py camera_obstacles:=false`, 실행 중엔 `ros2 param set /local_costmap/local_costmap camera_layer.enabled false`.
- 검증: 층을 하나씩 꺼 local costmap 비교 (my_navigation.md 2.1 그림). 남은 것: 바닥에서 라이다 아래 높이 물체(신발·상자)를 찍고 피해 가는지.

## 4. ② 의미 층 (Vision AI)

### 4.1 입력 보강 — 물체 크기 추정 (my_vision)

현재 `/vision/objects` 는 3D 점 하나. 회피용으로 **물체 반경**을 추가한다.

```
폭(m) ≈ bbox 폭(px) × 거리(m) / fx        (핀홀 모델, fx = camera_info 초점거리)
반경   ≈ 폭 / 2  (깊이 방향 크기는 모름 → 원으로 근사, 클래스별 최소 반경으로 보정)
```

출력은 지금처럼 Detection2DArray 를 쓰되 bbox 크기 필드를 미터로 채우거나, 전용 메시지(클래스·점수·위치·반경)를
둔다 — 구현 시 결정.

### 4.2 Nav2 costmap 플러그인 `SemanticLayer` (C++)

Nav2 는 costmap 층을 플러그인으로 추가할 수 있다 (`nav2_costmap_2d::Layer` 상속). 직접 만든다.

| 동작 | 내용 |
|---|---|
| 구독 | `/vision/objects` (map 또는 odom 프레임으로 변환된 위치) |
| 표시 | 물체마다 위치·반경으로 원을 장애물(LETHAL)로 찍고, **클래스별 추가 여유**를 둔다 |
| 기억 | 마지막으로 본 뒤 **N초 유지 후 소멸** (시야를 벗어나도 바로 잊지 않음). 다시 보이면 갱신 |
| 설정 | 클래스별 표 (YAML) |

```yaml
semantic_layer:
  plugin: "my_navigation::SemanticLayer"
  decay_sec: 5.0
  classes:
    person:       {extra_margin: 0.40, slow_radius: 1.5}   # 넓게 피하고 1.5m 안이면 감속
    dog:          {extra_margin: 0.40, slow_radius: 1.5}
    cat:          {extra_margin: 0.40, slow_radius: 1.5}
    chair:        {extra_margin: 0.10}
    potted plant: {extra_margin: 0.10}
    default:      {extra_margin: 0.05}
```

### 4.3 사람 근처 감속

Nav2 controller_server 는 속도 제한 토픽(`nav2_msgs/SpeedLimit`)을 받는다. 작은 노드(또는 SemanticLayer 내부)가
`slow_radius` 안에 사람이 있으면 최대 속도를 낮춘 값을 발행하고, 벗어나면 해제한다.

### 4.4 화면 (web UI)

장애물 지도 오버레이 위에 의미 층 물체를 **원 + 라벨("person 0.92 · 1.2m")**로 그린다.
깊이 층이 찍은 곳과 의미 층이 찍은 곳이 구분돼 보이게 색을 나눈다 → "AI 가 무엇을 보고 어떻게 판단했는지"가 보인다.

## 5. 단계

| 단계 | 내용 | 완료 기준 |
|---|---|---|
| **V1 깊이 층** ✅ 구현 (바닥 조정 남음) | 점 구름 발행 + local costmap 소스 추가 + 높이 필터 실측 조정 | 라이다 아래 높이의 물체(신발·상자)를 RViz/웹 장애물 지도에 찍고 피해 감. 바닥 오검출 없음 |
| **V2 크기 추정** | `/vision/objects` 에 반경 추가 | 의자·사람 반경이 실측과 대략 일치 (±30%) |
| **V3 SemanticLayer** | C++ costmap 플러그인, 클래스별 여유·소멸 | 사람 앞에서 의자보다 넓게 돌아감 |
| **V4 감속** | SpeedLimit 발행 | 사람 1.5m 안에서 속도 저하 확인 |
| V5 (선택) | 추적(물체 ID 유지·이동 방향 예측), 파인튜닝(실내 물체·문턱 클래스) | — |

V1 은 web UI 의 P2(지도·주행 화면)와 함께 보면 효과가 바로 보이므로 그 근처에 배치한다.

## 6. 미결

- 바닥 오검출 방지 높이(min_obstacle_height)는 실측 필요 — 바닥 재질·카메라 노이즈에 따라 다름.
- 의미 층의 프레임: map(SLAM 보정 시 점프) vs odom(연속). local costmap 이 odom 기준이므로 odom 이 유력.
- 클래스 목록·여유값은 실주행으로 조정.
- 깊이 층과 의미 층이 같은 물체를 둘 다 찍는 것은 문제 아님 (costmap 은 최댓값으로 합침).
- **반사·투명 재질(거울·유리)은 두 층 모두 못 막는다** — 라이다·depth 모두 적외선이라 거울 속 상을 거울 뒤에 찍거나 유리를
  통과한다. 대응은 사람이 그리는 **금지 구역·가상 벽**(Nav2 Keepout Filter, my_web_api.md 4.18, 2026-11-12 구현).
  센서 쪽 보강 후보: 초음파(소리라 거울·유리에서 반사) — 하드웨어 추가 필요.
