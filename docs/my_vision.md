# my_vision — Vision AI (물체 검출 + 거리) (Remote PC, GPU)

> 로봇 분야를 전혀 모르는 사람도 이 문서와 코드를 같이 보면
> "어떤 원리로 계산되고 왜 이렇게 작성했는지" 이해할 수 있도록 쓴 문서다.
> 입력이 되는 카메라 쪽 원리(정렬·캘리브레이션)는 [realsense_bringup.md](./realsense_bringup.md) 4장.

## 1. 한 줄 요약

로봇 카메라 영상에서 **물체를 찾고(무엇이, 어디에), 정렬된 depth 로 그 물체까지의
거리와 3D 위치를 계산**해 ROS 토픽으로 내보내는 패키지. 검출은 RF-DETR(GPU),
거리는 카메라 노드가 보내주는 컬러 정렬 depth 에서 읽는다.

```
Pi 카메라 노드 ──zenoh──▶ /camera/color/compressed ─┐
                          /camera/depth/compressed ─┼─▶ detector_node ─▶ /vision/detections (Detection2DArray)
                          /camera/color/camera_info ─┘        (RF-DETR)   └▶ /vision/annotated/compressed (박스 그린 JPEG)
```

## 2. 코드 구조

```
remote_pc/src/my_vision/
├── my_vision/
│   ├── camera_io.py       # 공용: 디코드, stamp 짝맞춤(FramePairer), 박스 depth 중앙값, deproject
│   ├── detector_node.py   # ★ RF-DETR 검출 + 거리·3D 위치 → Detection2DArray / 주석 영상
│   └── camera_viewer.py   # 카메라(또는 검출 결과) 영상 뷰어 — color | 정렬 depth | 오버레이
├── launch/vision.launch.py
├── package.xml / setup.py (ament_python)
```

```
├── my_vision/backends.py  # 추론 백엔드: torch(rfdetr predict) / tensorrt(엔진 캐시·실행)
├── my_vision/trt_build.py # ONNX→TensorRT 빌더(TensorRT Python API 직접, 옵션 풍부) + 엔진 러너 + 검증 + CLI build_trt
```

실행 환경은 컨테이너 이미지의 `vision` 스테이지(docker/Dockerfile): ROS Humble 위에
PyTorch(CUDA 12.8 휠) + `rfdetr` + `vision_msgs` + TensorRT(`tensorrt-cu12`) + ONNX 도구.
가중치는 `RF_HOME=/overlay_ws/models`(호스트 `remote_pc/models`, gitignore)에 첫 실행 때
자동 다운로드·캐시된다.

## 3. 출력 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/vision/detections` | `vision_msgs/Detection2DArray` | 검출마다 `bbox`(픽셀 중심·크기), `results[0].hypothesis.class_id`(이름)·`score`, **`results[0].pose.pose.position`(x,y,z, m)** — 카메라 광학 좌표계 3D 위치. z=0 이면 그 박스에서 유효 depth 를 못 구한 것 |
| `/vision/objects` | `vision_msgs/Detection2DArray` | 위와 같은 검출을 **`target_frame`(기본 `base_link`, SLAM 중이면 `map`)으로 TF2 변환**한 것. `pose.position` = 로봇/지도 좌표 3D 위치. 3D 를 못 구한 검출은 제외 |
| `/vision/markers` | `visualization_msgs/MarkerArray` | RViz 용 구(위치) + 텍스트(이름·점수·거리), `target_frame` 기준 |
| `/vision/annotated/compressed` | `sensor_msgs/CompressedImage` | 박스·클래스·점수·거리를 그린 JPEG (뷰어/RViz 용) |

`/vision/detections` 의 `header.frame_id` 는 입력 영상의 `camera_color_optical_frame` (카메라 계약 그대로).
`/vision/objects` 는 URDF 의 카메라 TF 체인(description.md 3.3)을 통해 `base_link` — 또는 `map` — 좌표로
옮긴 결과라 "로봇 앞 1.5m 왼쪽 0.3m" / "지도 위 어디" 로 바로 쓸 수 있다.

## 4. 원리 — 처음 보는 사람을 위한 설명

### 4.1 검출기: RF-DETR 가 하는 일
이미지를 넣으면 "어디에(박스) 무엇이(클래스) 얼마나 확실하게(점수)" 있는지 목록을 돌려주는
신경망이다. COCO 데이터셋으로 사전학습돼 사람·의자·컵·냉장고 등 80종을 안다. DETR 계열이라
후처리(NMS)가 필요 없고, 실시간급 속도(A6000 에서 medium 576px 기준 **~12ms**)다.
`threshold`(기본 0.5) 아래 점수는 버린다. 크기는 nano/small/medium/large 중 선택(기본 medium).

### 4.2 거리: 정렬 depth 에서 박스 안을 읽는다
카메라 노드가 depth 를 **컬러에 정렬**해 보내므로, 검출 박스의 픽셀을 그대로 depth 이미지에
대면 그 물체의 거리가 나온다 (정렬이 없으면 두 이미지의 같은 픽셀이 다른 곳을 가리켜 이게
불가능하다). 다만:
- **박스 전체를 쓰지 않고 중앙 50% 영역만** 쓴다 — 박스 가장자리는 배경이 섞인다.
- **평균이 아니라 중앙값** — 구멍(0)이나 배경 픽셀 몇 개에 값이 끌려가지 않게.
- 0(측정 없음)·비현실 범위(0.2m 미만, 8m 초과)는 제외.
→ `camera_io.depth_in_box()`. 유효 픽셀이 하나도 없으면 거리 없음(z=0)으로 보고한다.

### 4.3 3D 위치: 픽셀 + 거리 → 공간 좌표 (deproject)
카메라는 3D 점을 픽셀로 "투영"한다: `u = fx·X/Z + ppx`, `v = fy·Y/Z + ppy` (fx,fy,ppx,ppy 는
camera_info 의 인트린식 K). 거리 Z 를 알면 이걸 거꾸로 풀 수 있다:
`X = (u−ppx)/fx·Z`, `Y = (v−ppy)/fy·Z`. 박스 중심 픽셀과 4.2 의 거리를 넣으면 물체의 3D
위치가 나온다 (`camera_io.deproject()`). 좌표계는 광학 좌표(z 앞, x 오른쪽, y 아래).

### 4.4 TensorRT 백엔드 — "엔진은 실행할 환경에서 만든다"
TensorRT 는 신경망을 특정 GPU 에 맞춰 최적화한 **엔진(.trt)** 으로 컴파일해 돌리는 NVIDIA 런타임이다.
빠르지만 엔진 파일은 **빌드한 TensorRT 버전·CUDA·GPU 아키텍처에 묶여 이식되지 않는다.**
호스트에서 만든 엔진을 컨테이너에서 쓰면 버전이 다를 때 로드가 실패한다. 그래서:
- 이식 가능한 산출물은 **ONNX** 뿐이고, 엔진은 "실행할 환경에서 생성하는 캐시"로 취급한다.
- `backend:=tensorrt` 첫 실행 때 rfdetr 의 `export(format='tensorrt')` 가 **이 컨테이너 안에서**
  ONNX → 엔진(fp16)을 빌드한다 (TensorRT Python API/polygraphy, `trtexec` 불필요, A6000 실측 44초).
- 캐시 디렉터리 이름에 환경을 새긴다: `models/trt/rf-detr-{크기}-{해상도}-trt{TRT버전}-{GPU}/`
  → 이미지나 GPU 가 바뀌면 자동으로 다시 빌드. 이후 실행은 엔진만 로드(초 단위).
- 실행(`trt_build.EngineRunner`): 엔진의 I/O 텐서마다 torch 로 GPU 버퍼를 잡아 주소를 고정
  (`set_tensor_address`)하고 전용 CUDA 스트림에서 `execute_async_v3`. 전처리(리사이즈 규약·ImageNet
  정규화)와 후처리(sigmoid·배경 슬롯 제외·top-k)는 **rfdetr 의 함수를 그대로 import** 해 torch
  경로와 수치가 일치하게 한다 — 같은 프레임에서 torch/TensorRT/ONNX 세 경로의 점수가 동일함을 확인했다.
- 엔진 로드 후 torch 모델은 GPU 에서 내려 메모리를 아낀다.

#### 빌드 옵션 — `trt_build.build_engine` (TensorRT Python API 직접)
polygraphy 한 줄로는 정밀도 플래그 정도만 되므로, 빌더를 직접 다뤄 trtexec 수준의 옵션을 연다
(각 옵션의 주석에 대응 `trtexec` 플래그를 적어 두었다):

| 옵션 (노드 파라미터) | 의미 | trtexec 대응 |
|---|---|---|
| `trt_precision` fp32/fp16/**int8** | int8 은 PTQ: 캘리브레이션 이미지로 활성값 통계를 모아 스케일 결정, 캐시(.calib) 재사용. INT8 커널이 없거나 정확도가 나쁜 레이어는 FP16 fallback | `--fp16` / `--int8 --calib=` |
| `trt_opt_level` 0~5 | 빌더가 tactic 을 얼마나 오래 탐색하나 (0 빠른 빌드 … 5 최선 엔진) | `--builderOptimizationLevel` |
| `trt_workspace_gib` | tactic 시험용 GPU 스크래치 상한 | `--memPoolSize=workspace:` |
| `trt_timing_cache` | tactic 벤치 결과 캐시 → 재빌드 시간 단축 | `--timingCacheFile` |
| `trt_tf32` | Ampere+ 의 TF32 matmul (엄격 FP32 재현 시 끔) | `--noTF32` |
| (CLI) 동적 shape, 희소성, 프로파일링 메타, DLA | 배치/해상도 범위 프로파일, 2:4 프루닝 커널, 레이어별 프로파일 정보, Jetson DLA | `--minShapes/--optShapes/--maxShapes`, `--sparsity`, `--profilingVerbosity`, `--useDLACore` |

빌드 후 `verify_engine` 이 같은 입력을 ONNX Runtime 과 엔진에 넣어 비교한다. DETR 은 쿼리 300개
대부분이 점수 낮은 '버릴' 쿼리라 그 박스는 정밀도에 따라 크게 요동하므로, **임계값 이상인 실제
검출 쿼리끼리만** 점수·박스 차를 잰다 (전체 최대 차는 의미 없음 — 실측으로 배움).

INT8 절차: `ros2 run my_vision camera_viewer --save-dir /overlay_ws/models/calib --save-count 50` 으로
실제 로봇 카메라 프레임을 모은 뒤 `trt_precision:=int8 trt_calib_dir:=/overlay_ws/models/calib`.
캘리브레이션 전처리는 rfdetr 함수를 재사용해 추론 전처리와 bit-exact 하다 (cv2.resize 로 흉내 내면
리사이즈 규약이 달라 통계가 어긋난다). 독립 빌드/벤치는 `ros2 run my_vision build_trt --help`.

### 4.5 좌표 변환 — 카메라 좌표를 로봇/지도 좌표로 (TF2)
4.3 의 3D 위치는 **카메라 광학 프레임** 값이다 (z 앞, x 오른쪽, y 아래). 로봇 입장에선 "내 앞 몇 m, 왼쪽
몇 m" 가 필요하고, 지도에 찍으려면 `map` 좌표가 필요하다. ROS 의 TF2 가 이 변환을 담당한다:
- URDF 가 `base_link → camera_bottom_screw_frame → camera_link → camera_color_frame →
  camera_color_optical_frame` 의 **정적 변환**을 publish 하고(bringup 의 robot_state_publisher,
  실측 기반 — description.md 3.3), SLAM 이 `map → odom`, 로봇이 `odom → base_footprint` 를 publish 한다.
- 노드는 `tf2_ros.Buffer` 로 이 체인을 모아 두었다가 `lookup_transform(target, camera_optical)` 로
  합성 변환을 얻고 `do_transform_point` 로 점을 옮긴다. 회전(광학 규약 → 몸체 규약)과 평행이동
  (카메라가 로봇 중심에서 앞 58mm·왼쪽 33mm·위 60mm)이 한 번에 적용된다.
- **시각은 "최신 TF"** 를 쓴다. 정적 체인은 시각과 무관하고, `map→odom` 은 동적이지만 Pi 와 서버의
  시계가 완벽히 같지 않아 영상 stamp 로 조회하면 실패할 수 있다. 느린 로봇이라 수십 ms 차이는 무시 가능.
- `target_frame` 의 TF 가 없으면(예: SLAM 이 안 떠서 `map` 없음) `base_link` 로 fallback 하고 한 번만 경고.

검산 (2026-09-08 실기): base_link 결과 냉장고 (1.54, −0.57, −0.07) 를 역산하면 카메라 좌표 (오른쪽 0.60,
아래 0.13, 앞 1.48) — 영상 오른쪽에 있던 그 물체와 일치. `/vision/objects` 6.7Hz.

### 4.6 짝 맞춤과 QoS
color/depth 는 best effort 로 오므로 한쪽이 유실될 수 있다. 세 토픽의 `stamp` 가 같다는
계약을 이용해 `FramePairer` 가 같은 stamp 끼리만 짝을 지어 처리한다(짝이 안 맞은 옛 프레임은 버림).
구독 QoS 는 퍼블리셔와 같은 `sensor_data`(best effort) — 다르면 매칭이 안 돼 아무것도 안 온다.

## 5. 실측 (A6000, medium 576px, 카메라 640x480@6fps)

| 백엔드 | 추론 지연 | GPU 상주 메모리 | 빌드 | ONNX 대비 검출 쿼리 최대 차 (점수 / 박스) |
|---|---|---|---|---|
| torch | 12ms | (torch 모델 전체) | — | 기준 (설치만으로 동작) |
| tensorrt **fp16** (기본 backend) | **6ms** | **450MB** | 39초 | 0.15 / 0.25 (경계선 검출의 요동, 검출 수·최고 점수는 일치) |
| tensorrt int8 (PTQ, 50장) | 6ms | 450MB | 139초 | 0.25 / 0.51 |

- **INT8 은 이 조합(A6000 + RF-DETR)에선 이득이 없다**: 트랜스포머 디코더의 상당 레이어가 "scale 없음 →
  FP16 fallback" 되어 지연·엔진 크기가 fp16 과 같고, 정확도만 더 흔들린다. 그래서 기본은 fp16.
  INT8 이 의미 있는 곳은 Jetson 급이나 CNN 계열이며, 그때도 Q/DQ 명시적 양자화(ModelOpt)가 더 낫다.

- 처리율은 둘 다 6fps(입력 fps 에 묶임 — 카메라를 15fps 로 올려도 여유 충분)
- 검출 예: refrigerator 1.47m, chair 1.78m, book 1.97m — 뷰어의 중앙 십자선 거리(1.96m)와 일치
- 주석 영상에서 박스가 물체 윤곽과 맞고 거리가 물체별로 구분됨 (camera_viewer 로 확인)

## 6. 사용법 (서버 컨테이너)

### 6.1 설정은 `config/vision_params.yaml` 하나로
노드의 모든 설정(모델·가중치·임계값·백엔드·TensorRT 빌드 옵션·거리 계산·토픽)은
`remote_pc/src/my_vision/config/vision_params.yaml` 에 주석과 함께 있다. **이 파일을 고친 뒤 실행하면
그대로 적용된다** (symlink-install 이라 재빌드 불필요). 값 하나만 급히 바꿀 땐 런치 인자로:

```bash
cd /overlay_ws && colcon build --symlink-install && source install/setup.bash   # 최초 1회
ros2 launch my_vision vision.launch.py                                   # config/vision_params.yaml
ros2 launch my_vision vision.launch.py params_file:=/overlay_ws/my.yaml  # 다른 설정 파일
ros2 launch my_vision vision.launch.py threshold:=0.3 backend:=tensorrt  # 준 인자만 YAML 을 덮어씀
ros2 topic echo /vision/detections                                       # 검출 목록
export DISPLAY=:0; ros2 run my_vision camera_viewer --color-topic /vision/annotated/compressed   # 눈으로
```
런치 인자로 덮어쓸 수 있는 것: `model`, `weights`, `threshold`, `backend`, `trt_precision`,
`trt_opt_level`, `trt_calib_dir`. 나머지는 YAML 에서. 지도 좌표로 받으려면 YAML 의 `target_frame: map`
(SLAM 실행 중일 때). RViz 에서 `/vision/markers` (MarkerArray) 를 추가하면 검출 물체가 구·라벨로 보인다.

### 6.2 다른 모델 / 전이학습 모델 쓰기
```yaml
model: medium                                   # 아키텍처 (파인튜닝 때 쓴 크기와 같아야 함)
weights: /overlay_ws/models/my_robot.pth        # '' 이면 공식 COCO 사전학습, 경로면 그 체크포인트
class_names: ['']                               # 체크포인트에 이름이 없을 때만 id 순서대로
```
- `weights` 를 주면 rfdetr 이 그 `.pth` 를 로드하고 **클래스 수를 체크포인트에서 자동 추론**한다.
  클래스명은 체크포인트 저장값 → `class_names` → COCO 표 순으로 쓴다 (rfdetr `train()` 은 보통 저장).
- `backend: tensorrt` 면 엔진 캐시 디렉터리 이름에 **체크포인트 태그(파일명+해시)** 가 들어가
  `models/trt/rf-detr-medium-576-my_robot-1a2b3c4d-fp16-trt…/` 처럼 분리된다 — COCO 모델의 엔진과
  절대 섞이지 않고, 체크포인트를 바꾸면 자동으로 새 엔진을 빌드한다.
- 공식 가중치를 오프라인으로 쓰려면 `weights_dir`(기본 `/overlay_ws/models`)에 `rf-detr-<크기>.pth` 를
  두면 된다 (MD5 가 맞으면 다운로드 생략).

실제 검증: 공식 `.pth` 를 다른 이름으로 복사해 `weights` 로 지정 + tensorrt → 새 캐시
`…-my_finetuned-eb849b85-fp16-…` 에 엔진 빌드(39초) 후 정상 검출.

### 6.3 로그
5초마다 `fps | 추론 ms | 검출 요약`, 시작 시 가중치 출처·클래스명 출처·백엔드·엔진 경로가 찍힌다.
독립 엔진 빌드/벤치: `ros2 run my_vision build_trt --help`.

## 7. 개발 중 부딪힌 것 (기록)

- pip 로 torch/rfdetr 를 깔면 numpy 2.x 가 따라와 apt 의 cv2(numpy 1.x 빌드)가 `import cv2` 에서
  깨진다 → Dockerfile 에서 `numpy<2` 고정 (별도 레이어).
- rfdetr 1.9 는 API 가 바뀌었다: `RFDETRBase`→deprecated(medium 이 대응), `rfdetr.util` 제거,
  COCO 클래스는 `rfdetr.assets.coco_classes`, 가중치 캐시는 `RF_HOME`. 크기별 입력 해상도가
  달라(medium 576, 32 의 배수) 해상도를 강제하지 않고 모델 기본값을 쓴다.
- `img[:, :, ::-1]`(BGR→RGB) 은 음수 stride 뷰라 torch 가 거부 → `cv2.cvtColor`.
- 원격 셸에서 `pkill -f 패턴` 은 같은 명령줄 뒤쪽에 그 문자열이 또 있으면 **자기 셸을 죽인다**
  (정지와 실행을 한 명령에 넣었을 때). 정지는 별도 호출로, 패턴은 실행 줄에 없는 단어로.
- `pip install rfdetr[tensorrt]` 는 메타 패키지 `tensorrt` 를 통해 **cu13 변종**을 골랐다(실측) — torch 는
  CUDA 12.8 이라 한 프로세스에 CUDA 런타임 두 세대가 섞인다. `tensorrt-cu12` 를 명시해 해결.
- rfdetr export 의 엔진 파일명은 버전마다 다르다(`rfdetr-medium.trt`) → 이름을 가정하지 않고
  export 반환 경로 / 디렉터리의 `*.trt` 를 쓴다.
- "검출 0" 이 TensorRT 버그처럼 보였지만, 같은 프레임 비교로 세 경로가 동일함을 확인 — 장면의
  최고 점수(0.47)가 임계값(0.5) 아래였을 뿐. 백엔드를 의심하기 전에 **같은 입력으로 나란히 비교**할 것.

## 8. 다음 단계

1. ~~base_link 좌표 변환~~ ✅ (2026-09-08, `/vision/objects` + `/vision/markers`)
2. 통합 런치·RViz 구성에서 마커를 지도 위에 표시 (target_frame map).
3. 필요 시 커스텀 데이터로 파인튜닝 (rfdetr `train()`), 추적(ID 유지).
3. 양자화가 필요해지면 PTQ 대신 Q/DQ 명시적 양자화(ModelOpt) 로 — PTQ INT8 은 이 모델에서 이득 없음 확인.
