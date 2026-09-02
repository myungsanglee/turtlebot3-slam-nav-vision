#!/usr/bin/env python3
# =============================================================================
# rs_camera_node.py — pyrealsense2 기반 경량 RealSense 카메라 노드 (Raspberry Pi)
#
# [왜 직접 만들었나]
#   Pi4 USB 컨트롤러 + RSUSB 백엔드 조합에서 RGB 쪽 USB 컨트롤 경로가 세션 단위로
#   간헐 불통이 된다. 이 경로로 읽는 것이 바로 캘리브레이션(인트린식·익스트린식,
#   XU 컨트롤)이라서, 공식 realsense2_camera 노드는 camera_info 와 align_depth 에서
#   막히고 재시도로 control_transfer 를 홍수내며, 멈추면 손으로 죽여야 했다
#   (docs/troubleshooting.md 2026-09-02). 반면 "프레임만 뽑는" 순수 스트리밍은
#   항상 안정적이었다. → 스트리밍은 최소 경로로, 캘리브레이션은 캐시로 독립시킨다.
#
# [출력 — Vision 파이프라인이 소비하는 계약]
#   /camera/color/compressed    sensor_msgs/CompressedImage  format "jpeg"
#   /camera/depth/compressed    sensor_msgs/CompressedImage  format "16UC1; png"
#                               → PNG 16bit 무손실, 값은 mm 단위 (uint16), color 에 정렬됨.
#                                 cv2.imdecode(buf, cv2.IMREAD_UNCHANGED) 로 복원.
#                                 (JPEG 는 8bit 손실이라 거리값이 깨져 쓸 수 없음)
#   /camera/color/camera_info   sensor_msgs/CameraInfo (color 인트린식)
#   depth 는 color 프레임에 정렬돼 있으므로 두 이미지의 픽셀 (u,v) 가 같은 지점을
#   가리키고, camera_info 도 color 것 하나면 둘 다에 쓸 수 있다.
#   두 이미지는 같은 frameset 이라 header.stamp 가 동일 (동기화 소비 가능).
#
# [정렬 — 캘리브레이션 캐시로 XU 독립] ★ 이 노드의 핵심
#   캘리브레이션(color/depth 인트린식 + depth→color 익스트린식)은 공장 보정값이라
#   불변이다. 경로가 멀쩡한 세션에서 한 번 읽히면 파일에 캐시하고, 그 뒤로는
#     - 읽기 성공 세션: rs.align (C++, 빠름) 사용
#     - 읽기 실패 세션: 캐시로 numpy 수동 정렬 (deproject → 변환 → project, z-buffer)
#   캐시도 없고 읽기도 실패하면 color 만 publish 하며 주기적으로 다시 읽는다.
#   한 번이라도 성공하면 이후 실행은 XU 에 전혀 의존하지 않는다.
#
# [구조 — 캡처는 별도 "프로세스"] ★ 스레드가 아닌 이유
#   pipeline.start() 가 멈출 때 GIL 을 쥔 채 멈추므로 같은 프로세스의 감시 스레드까지
#   얼어붙어 자기 복구가 불가능하다 (실측). 부모(ROS publish + 감시) / 자식(캡처 +
#   정렬 + 인코딩) 으로 나누면 부모는 항상 살아 있고, 인코딩과 publish 가 코어를 나눠 쓴다.
#
# [복구 — 확실하되 온화하게] ★ 공격적 복구의 실측 부작용 두 가지
#   1) 장치 열거/start() 도중인 자식을 SIGKILL 하면 USB 트랜잭션이 끊겨 카메라가
#      버스에서 떨어져 나간다 (물리 재연결로만 복구). 콜드 스타트는 열거에만 ~11s
#      걸리므로 start 타임아웃은 넉넉히(45s) — 진짜 hang 만 잡고 정상 시작은 절대 안 죽인다.
#   2) USB 소프트 리셋(USBDEVFS_RESET)을 10초 간격으로 연타해도 버스에서 떨어진다.
#      → 연속 실패 N회 후에만, 최소 간격을 두고, 장치가 버스에 있을 때만 리셋.
#   재시작 간격은 지수 백오프, 장치가 버스에 없으면 재연결만 기다린다.
#
# [대역폭] 원격(Tailscale)은 compressed 만 브리징. depth PNG 가 color JPEG 보다
#   크므로, 주기적으로 찍히는 stats 로그로 실측하고 해상도/fps 를 조절한다.
# =============================================================================
import fcntl
import json
import multiprocessing as mp
import queue
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

USB_VENDOR_ID, USB_PRODUCT_ID = '8086', '0b3a'   # Intel RealSense D435i
USBDEVFS_RESET = 21780                           # _IO('U', 20)
DEFAULT_DEPTH_SCALE = 0.001                      # D400 계열 depth 단위 1mm


# ----------------------------------------------------------------------------- 수동 정렬
def make_manual_aligner(calib, depth_scale):
    """캐시 캘리브레이션으로 depth→color 정렬 (rs.align 대체). 출력은 mm uint16.

    원리 (rs.align 과 동일한 기하): depth 픽셀 → 3D 점(depth 카메라 좌표) → 익스트린식으로
    color 카메라 좌표로 변환 → color 이미지 평면에 투영 → 그 픽셀에 거리 기록.
    여러 depth 점이 같은 color 픽셀에 떨어지면 가까운 점이 이긴다(z-buffer, 가림 처리).
    렌즈 왜곡은 무시한다 (D435i depth 는 무왜곡, color 계수도 매우 작음).
    """
    ci, di, ex = calib['color'], calib['depth'], calib['extr']
    w, h = ci['width'], ci['height']
    # librealsense 익스트린식 rotation 은 column-major → row-major 행렬로 전치
    R = np.array(ex['rotation'], dtype=np.float32).reshape(3, 3).T
    t = np.array(ex['translation'], dtype=np.float32)
    us, vs = np.meshgrid(np.arange(di['width'], dtype=np.float32),
                         np.arange(di['height'], dtype=np.float32))
    xn = (us - di['ppx']) / di['fx']          # depth 픽셀의 정규화 광선 (미리 계산)
    yn = (vs - di['ppy']) / di['fy']

    def align(depth_raw):
        z = depth_raw.astype(np.float32) * depth_scale          # m
        X, Y = xn * z, yn * z                                   # depth 카메라 3D
        Xc = R[0, 0] * X + R[0, 1] * Y + R[0, 2] * z + t[0]     # color 카메라 3D
        Yc = R[1, 0] * X + R[1, 1] * Y + R[1, 2] * z + t[1]
        Zc = R[2, 0] * X + R[2, 1] * Y + R[2, 2] * z + t[2]
        valid = (z > 0) & (Zc > 0)
        Zs = np.where(valid, Zc, 1.0)
        uc = (ci['fx'] * Xc / Zs + ci['ppx'] + 0.5).astype(np.int32)   # color 픽셀로 투영
        vc = (ci['fy'] * Yc / Zs + ci['ppy'] + 0.5).astype(np.int32)
        valid &= (uc >= 0) & (uc < w) & (vc >= 0) & (vc < h)
        idx = vc[valid] * w + uc[valid]
        zmm = (Zc[valid] * 1000.0).astype(np.uint16)
        # z-buffer: 먼 점부터 쓰고 가까운 점이 덮어쓴다. uint16 안정 정렬은 기수 정렬(O(n))
        # 이라 float argsort 보다 훨씬 빠르다 (Pi4 실측으로 선택).
        order = np.argsort(zmm, kind='stable')[::-1]
        out = np.zeros(h * w, dtype=np.uint16)
        out[idx[order]] = zmm[order]
        out = out.reshape(h, w)
        # 구멍 메우기: depth 는 color 보다 각해상도가 낮아(시야가 더 넓음) depth 픽셀 하나가
        # color 픽셀 ~1.6개를 덮어야 하는데 점 하나만 찍으므로 빈 픽셀이 생긴다. 3x3 이웃 중
        # 가장 가까운 값으로 채운다 (rs.align 은 사각형 래스터화로 같은 문제를 푼다).
        # 기존 유효 픽셀은 건드리지 않는다.
        big = np.where(out == 0, 65535, out).astype(np.uint16)
        p = np.pad(big, 1, constant_values=65535)
        m = big.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy or dx:
                    np.minimum(m, p[1 + dy:1 + dy + h, 1 + dx:1 + dx + w], out=m)
        filled = np.where(out == 0, m, out)
        return np.where(filled == 65535, 0, filled).astype(np.uint16)
    return align


# ----------------------------------------------------------------------------- 자식 프로세스
def _capture_process(q, cfg):
    """pyrealsense2 캡처 + 정렬 + 인코딩. rclpy 를 모른다. 어디서 멈추든 부모가 회수한다."""
    import cv2
    import numpy as np
    import pyrealsense2 as rs

    def stage(msg):
        q.put(('stage', msg))

    def intr_dict(i):
        return {'width': i.width, 'height': i.height, 'fx': i.fx, 'fy': i.fy,
                'ppx': i.ppx, 'ppy': i.ppy, 'coeffs': list(i.coeffs)}

    def read_calibration(profile):
        """XU 읽기 3종. 하나라도 실패하면 None (이 세션은 컨트롤 경로 불통으로 간주)."""
        try:
            cs = profile.get_stream(rs.stream.color).as_video_stream_profile()
            ds = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            ex = ds.get_extrinsics_to(cs)
            return {'color': intr_dict(cs.get_intrinsics()), 'depth': intr_dict(ds.get_intrinsics()),
                    'extr': {'rotation': list(ex.rotation), 'translation': list(ex.translation)}}
        except RuntimeError as e:
            q.put(('log', f'캘리브레이션 XU 읽기 실패: {e}'))
            return None

    stage('import 완료 → 장치 열거(rs.pipeline)')
    t0 = time.monotonic()
    pipeline = rs.pipeline()
    stage(f'장치 열거 완료 ({time.monotonic() - t0:.1f}s) → pipeline.start()')
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.depth, cfg['width'], cfg['height'], rs.format.z16, cfg['fps'])
    rs_cfg.enable_stream(rs.stream.color, cfg['width'], cfg['height'], rs.format.bgr8, cfg['fps'])
    t0 = time.monotonic()
    profile = pipeline.start(rs_cfg)          # 멈추거나 던지면 부모가 감지해 재시작
    stage(f'pipeline.start() 성공 ({time.monotonic() - t0:.1f}s)')

    try:
        try:
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        except Exception:
            depth_scale = DEFAULT_DEPTH_SCALE
        depth_to_mm = depth_scale * 1000.0

        # 캘리브레이션: 장치 읽기 1회 → 성공이면 부모에 전달(캐시), 실패면 캐시 사용
        calib = read_calibration(profile)
        if calib is not None:
            q.put(('calib', calib))
            rs_align = rs.align(rs.stream.color)        # 읽기가 되는 세션이면 rs.align 도 된다
            manual = None
            stage('캘리브레이션 읽기 성공 → rs.align 사용')
        else:
            rs_align = None
            calib = cfg.get('calib')
            manual = make_manual_aligner(calib, depth_scale) if calib else None
            stage('캐시 캘리브레이션으로 수동 정렬' if manual else
                  '캘리브레이션 없음 → color 만 publish, 주기적으로 재시도')
        t_calib = time.monotonic()
        calib_fails = 0

        jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, cfg['jpeg_quality']]
        png_params = [cv2.IMWRITE_PNG_COMPRESSION, cfg['png_compression']]
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=cfg['stall_timeout_ms'])  # 타임아웃 → 예외 → 종료
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth:
                continue

            depth_mm = None
            if rs_align is not None:
                try:
                    d = rs_align.process(frames).get_depth_frame()
                    if d:
                        depth_mm = np.asanyarray(d.get_data())
                        if depth_to_mm != 1.0:
                            depth_mm = (depth_mm * depth_to_mm).astype(np.uint16)
                except RuntimeError as e:
                    rs_align = None                     # 이 세션은 포기 → 수동/보류
                    manual = make_manual_aligner(calib, depth_scale) if calib else None
                    q.put(('log', f'rs.align 실패({e}) → ' + ('수동 정렬로 전환' if manual else 'depth 보류')))
            if depth_mm is None and manual is not None:
                depth_mm = manual(np.asanyarray(depth.get_data()))

            ok_c, jpg = cv2.imencode('.jpg', np.asanyarray(color.get_data()), jpeg_params)
            png_bytes = None
            if depth_mm is not None:
                ok_d, png = cv2.imencode('.png', depth_mm, png_params)
                png_bytes = png.tobytes() if ok_d else None
            if ok_c:
                try:
                    q.put(('frame', jpg.tobytes(), png_bytes), block=False)
                except queue.Full:
                    pass                                # 부모가 밀리면 최신 유지를 위해 버림

            if manual is None and rs_align is None and \
                    time.monotonic() - t_calib >= cfg['calib_retry_period_sec']:
                t_calib = time.monotonic()
                calib = read_calibration(profile)
                if calib is not None:
                    q.put(('calib', calib))
                    manual = make_manual_aligner(calib, depth_scale)
                    stage('캘리브레이션 확보 → 정렬 depth publish 시작')
                else:
                    calib_fails += 1
                    if calib_fails >= cfg['calib_max_retries']:
                        # 컨트롤 경로 상태는 세션(장치 open) 단위라, 이 세션에선 안 되는 것.
                        # 스스로 깨끗이 종료(pipeline.stop)해 부모가 새 세션을 열게 한다 (kill 아님).
                        raise RuntimeError('이 세션에선 캘리브레이션을 못 읽음 → 새 세션 시도')
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass


# ----------------------------------------------------------------------------- 부모 노드
class RsCameraNode(Node):

    def __init__(self):
        super().__init__('rs_camera')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('jpeg_quality', 80)      # color JPEG 품질 (0~100)
        self.declare_parameter('png_compression', 1)    # depth PNG 압축 (0~9, 낮을수록 빠름)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        # 자식의 "한 단계" 상한 (단계 메시지마다 타이머 리셋). ★ 넉넉해야 한다:
        #   콜드 스타트는 장치 열거만 ~11s 이고, 열거/start 도중인 자식을 죽이면 카메라가
        #   USB 버스에서 떨어져 나간다(실측). 진짜 hang 만 잡도록.
        self.declare_parameter('start_timeout_sec', 45.0)
        self.declare_parameter('stall_timeout_ms', 5000)    # 스트리밍 중 프레임 정체
        self.declare_parameter('retry_delay_sec', 3.0)      # 재시작 간격 (지수 백오프 시작값)
        self.declare_parameter('max_retry_delay_sec', 30.0)
        self.declare_parameter('usb_reset_after_failures', 2)      # 연속 실패 N회 후에만 리셋
        self.declare_parameter('usb_reset_min_interval_sec', 60.0)  # 리셋 연타 금지
        self.declare_parameter('calib_retry_period_sec', 10.0)
        self.declare_parameter('calib_max_retries', 3)      # 캐시 없는 나쁜 세션은 이만큼 후 새 세션
        self.declare_parameter('calib_cache_file', '~/.rs_camera_calib.json')
        self.declare_parameter('stats_period_sec', 10.0)    # 0 이면 stats 로그 끔

        p = self.get_parameter
        self.cfg = {k: p(k).value for k in (
            'width', 'height', 'fps', 'jpeg_quality', 'png_compression',
            'stall_timeout_ms', 'calib_retry_period_sec', 'calib_max_retries')}
        self.frame_id = p('frame_id').value
        self.start_timeout = p('start_timeout_sec').value
        self.stall_timeout = p('stall_timeout_ms').value / 1000.0
        self.retry_delay = p('retry_delay_sec').value
        self.max_retry_delay = p('max_retry_delay_sec').value
        self.reset_after_failures = p('usb_reset_after_failures').value
        self.reset_min_interval = p('usb_reset_min_interval_sec').value
        self.cache_file = Path(p('calib_cache_file').value).expanduser()
        self.stats_period = p('stats_period_sec').value

        # 센서 스트림 표준 QoS (best effort) — /scan 과 같은 정책, 소비자도 이걸로 구독
        qos = qos_profile_sensor_data
        self.pub_color = self.create_publisher(CompressedImage, '/camera/color/compressed', qos)
        self.pub_depth = self.create_publisher(CompressedImage, '/camera/depth/compressed', qos)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)

        # 캐시가 있으면 자식이 XU 를 못 읽어도 정렬·camera_info 가 바로 가능하다
        self.calib = self._load_cache()
        self.cam_info = self._make_camera_info(self.calib['color']) if self.calib else None
        if self.calib:
            self.get_logger().info('캘리브레이션 캐시 로드 (장치 읽기 성공 시 갱신)')

        # 'spawn': fork 는 rclpy/DDS 스레드를 복제해 불안정하므로 깨끗한 새 인터프리터로
        self._ctx = mp.get_context('spawn')
        self._child = None
        self._stop = False
        self._sup = threading.Thread(target=self._supervise, daemon=True)
        self._sup.start()

    # ------------------------------------------------------------------ 감시 루프
    def _supervise(self):
        failures = 0
        delay = self.retry_delay
        last_reset = float('-inf')
        waiting_device = False
        while not self._stop:
            if self._find_device() is None:
                if not waiting_device:
                    self.get_logger().warn('RealSense 가 USB 버스에 없음 — 재연결 대기 (물리 재연결 필요할 수 있음)')
                    waiting_device = True
                time.sleep(5.0)
                continue
            if waiting_device:
                self.get_logger().info('RealSense 재감지 → 5s 안정 대기 후 시작')
                waiting_device = False
                time.sleep(5.0)

            streamed = self._run_child()
            if self._stop:
                break
            if streamed:                       # 한 번이라도 흘렀으면 백오프 초기화
                failures, delay = 0, self.retry_delay
            else:
                failures += 1

            now = time.monotonic()
            if (failures >= self.reset_after_failures
                    and now - last_reset >= self.reset_min_interval
                    and self._find_device() is not None):
                self._usb_soft_reset()
                last_reset = time.monotonic()
            self.get_logger().info(f'{delay:.0f}s 후 재시작 (연속 실패 {failures}회)')
            time.sleep(delay)
            delay = min(delay * 2, self.max_retry_delay)

    def _run_child(self):
        """자식 하나의 생애: 시작 → 데이터 수신·publish → 타임아웃/종료 시 회수.
        프레임을 하나라도 받았으면 True."""
        q = self._ctx.Queue(maxsize=2)
        cfg = dict(self.cfg, calib=self.calib)
        child = self._ctx.Process(target=_capture_process, args=(q, cfg), daemon=True)
        child.start()
        self._child = child
        self.get_logger().info(
            f'캡처 프로세스 시작 (pid {child.pid}): '
            f'color+depth(aligned) {self.cfg["width"]}x{self.cfg["height"]}@{self.cfg["fps"]}')

        last_data = t_stats = time.monotonic()
        n_frames = n_depth = jpg_bytes = png_bytes = 0
        streaming = False
        while not self._stop:
            try:
                item = q.get(timeout=1.0)
            except queue.Empty:
                item = None
            now = time.monotonic()
            if item is not None:
                last_data = now
                kind = item[0]
                if kind == 'frame':
                    if not streaming:
                        streaming = True
                        self.get_logger().info('스트리밍 시작')
                    self._publish(item[1], item[2])
                    n_frames += 1
                    jpg_bytes += len(item[1])
                    if item[2] is not None:
                        n_depth += 1
                        png_bytes += len(item[2])
                elif kind == 'calib':
                    self._on_calibration(item[1])
                elif kind == 'stage':
                    self.get_logger().info(f'[자식] {item[1]}')
                elif kind == 'log':
                    self.get_logger().warn(item[1])

            if self.stats_period > 0 and now - t_stats >= self.stats_period and n_frames:
                dt = now - t_stats
                depth_txt = (f'depth {png_bytes / dt / 1e6:.2f} MB/s (평균 {png_bytes / n_depth / 1e3:.0f} KB)'
                             if n_depth else 'depth 없음')
                self.get_logger().info(
                    f'{n_frames / dt:.1f} fps | color {jpg_bytes / dt / 1e6:.2f} MB/s '
                    f'(평균 {jpg_bytes / n_frames / 1e3:.0f} KB) | {depth_txt}')
                n_frames = n_depth = jpg_bytes = png_bytes = 0
                t_stats = now

            if not child.is_alive():
                self.get_logger().warn(f'캡처 프로세스 종료됨 (exit {child.exitcode})')
                break
            limit = self.stall_timeout if streaming else self.start_timeout
            if now - last_data > limit:
                self.get_logger().warn(
                    f'{limit:.0f}s 동안 데이터 없음 ({"프레임 정체" if streaming else "start 멈춤/RGB 미시작"})')
                break

        if child.is_alive():
            child.kill()                       # GIL 에 갇혀 있어도 확실히 죽는다
        child.join(timeout=3.0)
        q.close()
        return streaming

    # ------------------------------------------------------------------ USB 장치
    def _find_device(self):
        """sysfs 에서 D435i 의 (bus, devnum) 을 찾는다. 버스에 없으면 None."""
        try:
            for dev in Path('/sys/bus/usb/devices').iterdir():
                vid, pid = dev / 'idVendor', dev / 'idProduct'
                if not (vid.exists() and pid.exists()):
                    continue
                if vid.read_text().strip() == USB_VENDOR_ID and pid.read_text().strip() == USB_PRODUCT_ID:
                    return int((dev / 'busnum').read_text()), int((dev / 'devnum').read_text())
        except Exception:
            pass
        return None

    def _usb_soft_reset(self):
        """USBDEVFS_RESET ioctl 로 카메라를 재열거 (udev 규칙 덕에 sudo 불필요)."""
        found = self._find_device()
        if found is None:
            return
        bus, num = found
        try:
            with open(f'/dev/bus/usb/{bus:03d}/{num:03d}', 'wb') as f:
                fcntl.ioctl(f, USBDEVFS_RESET, 0)
            self.get_logger().info(f'USB 소프트 리셋 (bus {bus} dev {num}) → 재열거 대기')
            time.sleep(3.0)
        except Exception as e:
            self.get_logger().warn(f'USB 리셋 실패: {e}')

    # ------------------------------------------------------------------ publish
    def _publish(self, jpg, png):
        stamp = self.get_clock().now().to_msg()     # 두 이미지·info 동일 stamp

        msg_c = CompressedImage()
        msg_c.header.stamp = stamp
        msg_c.header.frame_id = self.frame_id
        msg_c.format = 'jpeg'
        msg_c.data = jpg
        self.pub_color.publish(msg_c)

        if png is not None:
            msg_d = CompressedImage()
            msg_d.header.stamp = stamp
            msg_d.header.frame_id = self.frame_id
            msg_d.format = '16UC1; png'
            msg_d.data = png
            self.pub_depth.publish(msg_d)

        if self.cam_info is not None:
            self.cam_info.header.stamp = stamp
            self.pub_info.publish(self.cam_info)

    # ------------------------------------------------------------------ 캘리브레이션
    def _on_calibration(self, calib):
        self.calib = calib
        self.cam_info = self._make_camera_info(calib['color'])
        self._save_cache(calib)
        self.get_logger().info('캘리브레이션 확보 → camera_info publish, 캐시 저장')

    def _cache_key(self):
        return f'{self.cfg["width"]}x{self.cfg["height"]}'

    def _save_cache(self, calib):
        try:
            data = json.loads(self.cache_file.read_text()) if self.cache_file.exists() else {}
            data[self._cache_key()] = calib
            self.cache_file.write_text(json.dumps(data, indent=2))
        except Exception as e:
            self.get_logger().warn(f'캘리브레이션 캐시 저장 실패: {e}')

    def _load_cache(self):
        try:
            if not self.cache_file.exists():
                return None
            return json.loads(self.cache_file.read_text()).get(self._cache_key())
        except Exception as e:
            self.get_logger().warn(f'캘리브레이션 캐시 읽기 실패: {e}')
            return None

    def _make_camera_info(self, i):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width, info.height = i['width'], i['height']
        info.distortion_model = 'plumb_bob'
        info.d = list(i['coeffs'])
        info.k = [i['fx'], 0.0, i['ppx'],
                  0.0, i['fy'], i['ppy'],
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [i['fx'], 0.0, i['ppx'], 0.0,
                  0.0, i['fy'], i['ppy'], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    # ------------------------------------------------------------------ 종료
    def destroy_node(self):
        self._stop = True
        if self._child is not None and self._child.is_alive():
            self._child.kill()
            self._child.join(timeout=3.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = RsCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
