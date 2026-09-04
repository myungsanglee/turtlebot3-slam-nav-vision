# realsense_bringup — RealSense D435i 카메라 브링업 (Raspberry Pi)

> 로봇 분야를 전혀 모르는 사람도 이 문서와 코드를 같이 보면
> "어떤 원리로 계산되고 왜 이렇게 작성했는지" 이해할 수 있도록 쓴 문서다.
> 문제 해결 "여정"은 [troubleshooting.md](./troubleshooting.md) 2026-08-27, 09-02, 09-03 항목.

## 1. 한 줄 요약

로봇에 달린 RealSense D435i 카메라의 **컬러 영상과, 컬러에 정렬된 거리(depth) 영상**을
압축해서 ROS 토픽으로 내보내는 패키지. 공식 드라이버(realsense2_camera)가 이 Pi 에서
간헐적으로 막히는 문제를 겪고, **pyrealsense2 로 직접 만든 경량 노드**로 해결했다.

## 2. 코드 구조

```
robot/src/realsense_bringup/
├── scripts/rs_camera_node.py      # ★ 자체 카메라 노드 (기본 드라이버)
├── launch/rs_camera.launch.py     # 자체 노드 런치
├── launch/realsense.launch.py     # 공식 realsense2_camera 런치 (대안으로 유지)
├── launch/full_bringup.launch.py  # turtlebot3_bringup + 카메라 통합 (camera_driver 선택)
├── CMakeLists.txt / package.xml
```

Pi 의 `~/realsense_ros_ws/src/realsense_bringup` 이 이 디렉터리로 심볼릭 링크되어
빌드된다 (pi_setup.md 5단계). 원격 전송은 zenoh 브리지가 담당하며, allow 리스트의
정규식(`/camera/.*/compressed`, `/camera/.*/camera_info`)이 이 토픽들을 그대로 통과시킨다.

## 3. 출력 토픽 — Vision 파이프라인과의 계약

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/camera/color/compressed` | `sensor_msgs/CompressedImage` | 컬러 JPEG (`format: "jpeg"`) |
| `/camera/depth/compressed` | `sensor_msgs/CompressedImage` | **컬러에 정렬된** depth, PNG 16bit 무손실, **mm 단위 uint16** (`format: "16UC1; png"`) |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 컬러 카메라 인트린식 (아래 4.2) |

- 세 토픽은 **같은 frameset** 에서 나오므로 `header.stamp` 가 동일 → 소비자가 시간으로 짝지을 수 있다.
- QoS 는 **sensor_data(best effort)** — `/scan` 과 같은 정책. 구독자도 best effort 로 맞출 것.
- **depth 를 JPEG 로 압축하면 안 되는 이유**: JPEG 는 8bit 손실 압축이라 "1234mm" 같은
  거리값이 뭉개진다. PNG 는 16bit 를 무손실로 보존한다. 소비 측 복원:
  ```python
  depth_mm = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)  # uint16, mm
  ```
- 실측 (640x480): color ≈ 36KB/프레임, depth ≈ 93KB/프레임. 15fps 면 합계 ≈ 1.9MB/s 로
  Tailscale 전송 확인. **기본값은 6fps**(합계 ≈ 0.8MB/s) — Pi 부하(전원 처짐)와 대역폭을
  고려한 선택이며 Vision 추론 주기에 충분. 카메라 지원값은 6/15/30/60 (5 는 없음).

## 4. 원리 — 처음 보는 사람을 위한 설명

### 4.1 "정렬(align)"이란 무엇이고 왜 필요한가

D435i 는 카메라가 두 개다: **컬러 카메라**와 **depth 센서(적외선 스테레오)**. 둘은
몸체 안에서 **약 15mm 떨어져** 있고 시야각도 다르다. 그래서 컬러 이미지의 (u,v) 픽셀과
depth 이미지의 (u,v) 픽셀은 **같은 물체를 가리키지 않는다.**

Vision AI 는 "컬러에서 컵을 찾았는데, 그 컵까지 거리는 얼마인가"를 알아야 한다. 그러려면
depth 를 **컬러 카메라 시점으로 다시 그려서** 두 이미지의 픽셀이 1:1 로 대응하게 만들어야
한다. 이것이 정렬(align)이다. 정렬된 depth 에선 `depth[v, u]` 가 곧 `color[v, u]` 지점의
거리다. 그리고 두 이미지가 같은 시점이므로 **camera_info 도 컬러 것 하나면 충분**하다.

### 4.2 정렬에 필요한 세 가지 숫자 — 캘리브레이션

- **인트린식(intrinsics)**: 카메라 하나의 "렌즈 성격". 초점거리(fx, fy)와 광학 중심(ppx, ppy).
  3D 점 (X,Y,Z) 가 픽셀 어디에 맺히는지 알려준다: `u = fx·X/Z + ppx`, `v = fy·Y/Z + ppy`.
  거꾸로 픽셀+거리로 3D 점을 복원할 수도 있다(deproject).
- **익스트린식(extrinsics)**: 두 카메라 사이의 **회전 + 평행이동**. depth 카메라 좌표의 점을
  컬러 카메라 좌표로 옮긴다: `P_color = R · P_depth + t`. 이 로봇의 실측 캐시값은
  `t ≈ (0.0149, 0.0001, 0.0003) m` — RGB 모듈이 depth 기준 x 로 14.9mm 떨어져 있다는 뜻으로,
  D435i 의 실제 물리 배치와 일치한다(값이 맞다는 방증).
- 이 값들은 **공장 보정값이라 카메라마다 고정**이고 바뀌지 않는다. → 한 번 읽으면 캐시할 수 있다 (핵심).

### 4.3 정렬 계산 순서 (코드 `make_manual_aligner`)

depth 이미지의 모든 픽셀에 대해 벡터 연산으로:
1. **deproject**: depth 픽셀 (u,v) + 거리 z → depth 카메라 3D 점 `(x_n·z, y_n·z, z)`
   (`x_n = (u-ppx)/fx` 는 미리 계산해 둔 "정규화 광선")
2. **변환**: 익스트린식으로 컬러 카메라 좌표 `P_c = R·P + t`
3. **project**: 컬러 인트린식으로 컬러 픽셀 `(u_c, v_c)` 계산
4. **기록**: 결과 이미지 `out[v_c, u_c] = Z_c`. 여러 depth 점이 같은 컬러 픽셀에 떨어지면
   (가까운 물체가 먼 물체를 가림) **가까운 점이 이겨야** 한다 → 먼 점부터 쓰고 가까운 점이
   덮어쓰는 z-buffer (`argsort(-z)`).

5. **구멍 메우기**: depth 는 컬러보다 시야가 넓어 각해상도가 낮다(≈7.4 vs 9.3 px/°).
   depth 픽셀 하나가 컬러 픽셀 ~1.6개를 덮어야 하는데 점 하나만 찍으므로 빈 픽셀이
   생긴다(실측 채움 40%). 3x3 이웃 중 **가장 가까운 값**으로 빈 픽셀만 채워 100% 로
   만든다 (rs.align 은 사각형 래스터화로 같은 문제를 푼다).

librealsense 의 `rs.align` 이 하는 일과 같은 기하다. 렌즈 왜곡은 무시한다 (depth 는
무왜곡, 컬러 계수도 매우 작음). 실제 캐시값으로 검증: 1m 평면 → 995~1005mm 100% 채움.
Pi4 실측 **≈130ms/프레임(~8fps)** — 이 경로는 rs.align 이 안 되는 나쁜 세션의
**대체 수단**이라 이 속도로 충분하고(좋은 세션은 rs.align 15fps), Vision 이 depth 를
15fps 로 쓸 일도 없다.

## 5. 왜 자체 노드인가 — 설계와 실측 근거

### 5.1 문제: 공식 노드가 Pi 에서 간헐 실패

Pi4 의 USB3 컨트롤러 + RSUSB(유저스페이스) 백엔드 조합에서 **RGB 쪽 USB 컨트롤 경로가
세션(장치 open) 단위로 간헐 불통**이 된다. 하필 이 경로로 읽는 것이 캘리브레이션(XU 컨트롤)
이라, 공식 노드는 camera_info·align_depth 에서 막히고, 재시도하며 `control_transfer`
에러를 홍수내고, 심하면 `start()` 가 멈춰 손으로 죽여야 했다. 반면 **"프레임만 뽑는"
순수 스트리밍은 경로 상태와 무관하게 항상 됐다** (troubleshooting 09-02).

### 5.2 해법 세 가지

**① 캘리브레이션 캐시로 XU 독립** — 경로가 멀쩡한 세션에서 한 번 읽히면
`~/.rs_camera_calib.json` 에 저장. 이후 세션은:
- 읽기 성공 → `rs.align`(C++, 빠름) 사용
- 읽기 실패 → 캐시로 **수동 정렬**(4.3) → 정렬 품질 동일, XU 무관
- 캐시도 없고 읽기도 실패 → color 만 내보내며 재시도, 몇 번 안 되면 자식이 **스스로 종료해
  새 세션**을 연다(경로 상태가 세션 단위이므로). 한 번만 성공하면 영구 해결.

**② 캡처를 별도 프로세스로 분리** — `pipeline.start()` 가 멈출 때 **GIL 을 쥔 채** 멈춰서
같은 프로세스의 감시 타이머·스레드까지 얼어붙는 것을 실측했다. 스레드 감시로는 자기 복구가
불가능하다. 그래서 부모(ROS publish + 감시) / 자식(캡처·정렬·인코딩) 으로 나눴다.
자식이 어디에 갇히든 부모는 살아 있고 SIGKILL 은 GIL 과 무관하다. 덤으로 인코딩과
publish 가 4코어를 나눠 쓴다.

**③ 확실하되 온화한 복구** — 실패 유형(start 멈춤/RGB 미시작/정체)은 전부 "자식에게서
데이터가 안 온다"로 드러나므로 **타임아웃 하나**로 잡는다. 단, 개발 중 실측한 하드웨어
특성 두 가지 때문에 공격적이면 안 된다:
- **장치 열거/start 도중인 프로세스를 kill 하면 카메라가 USB 버스에서 떨어져 나간다**
  (물리 재연결로만 복구). 콜드 스타트는 열거에만 ~11초 걸리므로 시작 타임아웃은 45초로
  넉넉히 — 정상 시작을 조기에 죽이는 오탐이 최악이다.
- **USB 소프트 리셋을 10초 간격으로 연타해도 버스에서 떨어진다** → 연속 실패 2회 후에만,
  최소 60초 간격, 장치가 버스에 있을 때만. 재시작은 지수 백오프. 장치가 없으면 재연결만 대기.

### 5.3 실측 결과

- 콜드 스타트: 장치 열거 ~11s(RSUSB 프로빙) + start 0.7s. 따뜻한 장치는 열거 0.1s.
- 좋은 세션: 캘리브레이션 즉시 읽힘 → 캐시 생성 → rs.align 15fps.
- 나쁜 세션에서 시작해도 재시도(조기 kill 없이)가 두 번째 세션에서 수렴하는 것 확인.
- 스트리밍: 15.0fps 안정, 서버 수신 확인, depth `16UC1; png` 82~93KB.
- 수동 정렬(대체 경로): 값 정확(1m 평면 995~1005mm), 채움 100%, Pi4 ≈130ms/프레임(~8fps).

## 6. 사용법

```bash
# Pi — 통합 (로봇 + 카메라, 기본 custom 드라이버)
ros2 launch realsense_bringup full_bringup.launch.py
# Pi — 카메라만
ros2 launch realsense_bringup rs_camera.launch.py
ros2 launch realsense_bringup rs_camera.launch.py fps:=15                  # 고속 (지원값 6/15/30/60)
# 공식 노드로 비교하고 싶을 때
ros2 launch realsense_bringup full_bringup.launch.py camera_driver:=realsense2
```

주요 파라미터 (`rs_camera_node.py`): `width/height/fps`, `jpeg_quality`, `png_compression`,
`start_timeout_sec`(45), `stall_timeout_ms`(5000), `usb_reset_after_failures`(2),
`usb_reset_min_interval_sec`(60), `calib_cache_file`(`~/.rs_camera_calib.json`).

노드 로그의 `[자식] ...` 단계 메시지로 어디서 시간이 가는지, 10초마다 stats 로 fps·대역폭을
볼 수 있다. **카메라를 바꾸면 캐시 파일을 지울 것** (캘리브레이션은 개체별 값).

## 7. 운영 주의 — 이 노드에도 적용되는 하드웨어 특성

- **멈춘 카메라 프로세스를 함부로 `kill -9` 하지 말 것.** 열거/start 도중이면 카메라가
  버스에서 떨어진다. 노드는 이를 알고 넉넉한 타임아웃으로 정상 시작을 기다린다.
  손으로 정리할 땐 스트리밍 중(로그에 fps 가 찍히는 상태)에 하는 것이 안전하다.
- 카메라가 버스에서 사라졌으면(`lsusb` 에 없음) 소프트웨어로는 복구 불가 — 물리 재연결.
- **fps 가 절반(≈8)으로 떨어지면 전원부터**: `vcgencmd get_throttled` 을 **카메라 스트리밍 중에**
  확인. `0x50005`(bit0=현재 전압 부족, bit2=스로틀링 중)이면 ARM 클럭이 600MHz 로 떨어져
  노드가 2.5배 느려진다. 유휴 땐 `0x50000`(이력만)으로 보여 놓치기 쉽다 — 부하 의존 처짐
  (troubleshooting 09-03(2)). 전원 경로(어댑터·USB-C 케이블·연결)를 점검.

## 8. 다음 단계

1. `base_link → camera_link` TF 를 URDF 에 실측 반영 (description.md 6장) — 정렬 depth 의
   3D 점을 로봇 좌표로 옮기려면 필요.
2. Vision AI 노드(서버): 이 토픽 구독 → 디코드 → 추론. depth 로 검출 물체까지 거리 계산.
3. 정렬 depth 의 구멍(color 해상도에 대응 안 되는 픽셀) 보간은 소비 측에서 필요 시.
