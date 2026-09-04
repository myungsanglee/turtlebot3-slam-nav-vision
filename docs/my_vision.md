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

실행 환경은 컨테이너 이미지의 `vision` 스테이지(docker/Dockerfile): ROS Humble 위에
PyTorch(CUDA 12.8 휠) + `rfdetr` + `vision_msgs`. 가중치는 `RF_HOME=/overlay_ws/models`
(호스트 `remote_pc/models`, gitignore)에 첫 실행 때 자동 다운로드·캐시된다.

## 3. 출력 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/vision/detections` | `vision_msgs/Detection2DArray` | 검출마다 `bbox`(픽셀 중심·크기), `results[0].hypothesis.class_id`(이름)·`score`, **`results[0].pose.pose.position`(x,y,z, m)** — 카메라 광학 좌표계 3D 위치. z=0 이면 그 박스에서 유효 depth 를 못 구한 것 |
| `/vision/annotated/compressed` | `sensor_msgs/CompressedImage` | 박스·클래스·점수·거리를 그린 JPEG (뷰어/RViz 용) |

`header.frame_id` 는 입력 영상의 `camera_color_optical_frame`. 이 좌표를 로봇 좌표(`base_link`)로
옮기려면 `base_link → camera_link` TF 가 필요하다 (CLAUDE.md 5번 — 카메라 위치 실측 후 URDF 반영).

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

### 4.4 짝 맞춤과 QoS
color/depth 는 best effort 로 오므로 한쪽이 유실될 수 있다. 세 토픽의 `stamp` 가 같다는
계약을 이용해 `FramePairer` 가 같은 stamp 끼리만 짝을 지어 처리한다(짝이 안 맞은 옛 프레임은 버림).
구독 QoS 는 퍼블리셔와 같은 `sensor_data`(best effort) — 다르면 매칭이 안 돼 아무것도 안 온다.

## 5. 실측 (2026-09-04, A6000, medium, 카메라 640x480@6fps)

- 추론 12ms/프레임, 처리율 6fps(입력 fps 에 묶임 — 카메라를 15fps 로 올려도 여유 충분)
- 검출 예: refrigerator 1.47m, chair 1.78m, book 1.97m — 뷰어의 중앙 십자선 거리(1.96m)와 일치
- 주석 영상에서 박스가 물체 윤곽과 맞고 거리가 물체별로 구분됨 (camera_viewer 로 확인)

## 6. 사용법 (서버 컨테이너)

```bash
cd /overlay_ws && colcon build --symlink-install && source install/setup.bash   # 최초 1회
ros2 launch my_vision vision.launch.py                     # 기본 medium, threshold 0.5
ros2 launch my_vision vision.launch.py model:=small threshold:=0.4
ros2 topic echo /vision/detections                         # 검출 목록
export DISPLAY=:0; ros2 run my_vision camera_viewer --color-topic /vision/annotated/compressed   # 눈으로
```
파라미터: `model`, `threshold`, `resolution`(0=모델 기본), `depth_roi_frac`(0.5), `class_filter`
(예: `['person','chair']`), `weights_dir`. 로그에 5초마다 fps·추론 시간·검출 요약이 찍힌다.

## 7. 개발 중 부딪힌 것 (기록)

- pip 로 torch/rfdetr 를 깔면 numpy 2.x 가 따라와 apt 의 cv2(numpy 1.x 빌드)가 `import cv2` 에서
  깨진다 → Dockerfile 에서 `numpy<2` 고정 (별도 레이어).
- rfdetr 1.9 는 API 가 바뀌었다: `RFDETRBase`→deprecated(medium 이 대응), `rfdetr.util` 제거,
  COCO 클래스는 `rfdetr.assets.coco_classes`, 가중치 캐시는 `RF_HOME`. 크기별 입력 해상도가
  달라(medium 576, 32 의 배수) 해상도를 강제하지 않고 모델 기본값을 쓴다.
- `img[:, :, ::-1]`(BGR→RGB) 은 음수 stride 뷰라 torch 가 거부 → `cv2.cvtColor`.

## 8. 다음 단계

1. **TensorRT 변환** — rfdetr 의 ONNX export → `trtexec`/TensorRT 런타임으로 추론 (지연·GPU 점유 ↓).
2. **base_link 좌표 변환** — camera_link TF 실측 반영 후 3D 위치를 로봇 좌표로 (TF2).
3. 필요 시 커스텀 데이터로 파인튜닝 (rfdetr `train()`), 추적(ID 유지), RViz 마커 표시.
