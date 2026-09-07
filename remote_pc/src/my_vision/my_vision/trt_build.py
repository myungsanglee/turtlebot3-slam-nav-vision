# =============================================================================
# trt_build.py — ONNX → TensorRT 엔진 빌더 (TensorRT Python API 직접 사용) + 엔진 러너 + 검증
#
# polygraphy 한 줄(CreateConfig(fp16=True))로는 못 하는 빌드 옵션을 전부 열어 둔다:
#   정밀도 fp32/fp16/int8(PTQ 캘리브레이션 + 캐시), 정적/동적 shape(최적화 프로파일),
#   workspace·최적화 레벨·타이밍 캐시·TF32·2:4 희소성·프로파일링 메타데이터·DLA.
#   각 옵션 주석에 대응하는 trtexec 플래그를 적어 두었다.
#
# [이 환경에 맞춘 점]
#   - GPU 버퍼는 pycuda 대신 torch 텐서 (컨테이너에 torch 만 있음)
#   - INT8 캘리브레이션 전처리는 rfdetr 의 함수를 재사용 → 추론 전처리와 bit-exact 일치
#     (cv2.resize 로 흉내 내면 리사이즈 규약이 달라 캘리브레이션 통계가 미세하게 어긋난다)
#   - TensorRT 10: EXPLICIT_BATCH 는 기본(플래그 deprecated), IInt8EntropyCalibrator2 는
#     deprecated 지만 동작 (NVIDIA 는 Q/DQ 명시적 양자화 권장 — 추후 과제)
#   - 빌드 후 ONNX Runtime 과 같은 입력으로 비교해 수치 차이를 로그로 남긴다
#
# [사용] 노드가 자동으로 부르지만(backends.TensorRTBackend), 독립 실행도 된다:
#   ros2 run my_vision build_trt --model medium --precision fp16 --out /overlay_ws/models/trt/manual
#   ros2 run my_vision build_trt --onnx m.onnx --engine m_int8.trt --precision int8 --calib-dir /overlay_ws/models/calib
# =============================================================================
import argparse
import glob
import os
import time
from pathlib import Path

import numpy as np

IMAGE_EXTS = ('*.jpg', '*.jpeg', '*.png', '*.bmp')


def _preprocess_file(path, h, w):
    """캘리브레이션/검증용 이미지 파일 → (1,3,h,w) float32 — rfdetr 추론 전처리와 동일 규약."""
    from PIL import Image
    from rfdetr.export._onnx.inference import _preprocess_pil_to_nchw
    with Image.open(path) as im:
        return _preprocess_pil_to_nchw(im, h, w, 3)


def _list_images(d):
    files = []
    for pat in IMAGE_EXTS:
        files += glob.glob(os.path.join(d, pat)) + glob.glob(os.path.join(d, pat.upper()))
    return sorted(set(files))


# ----------------------------------------------------------------------------- INT8 캘리브레이터
def make_calibrator(trt, image_dir, cache_file, shape, max_images, log):
    """IInt8EntropyCalibrator2 구현 (trt 모듈을 받아 lazy 하게 클래스를 만든다)."""
    import torch

    class ImageCalibrator(trt.IInt8EntropyCalibrator2):
        """TensorRT 가 get_batch 를 반복 호출하며 활성값 통계를 모아 텐서별 양자화 스케일을 정한다.
        캐시 파일이 있으면 이미지 패스를 건너뛴다 (같은 모델·같은 데이터면 재사용 가능)."""

        def __init__(self):
            super().__init__()
            self.cache_file = cache_file
            self.n, _, self.h, self.w = shape
            self.files = _list_images(image_dir)[:max_images]
            if not self.files:
                raise ValueError(f'INT8 캘리브레이션 이미지가 없음: {image_dir}')
            self.i = 0
            self.buf = torch.empty(shape, dtype=torch.float32, device='cuda')   # 배치 하나 크기의 GPU 버퍼
            log(f'[calib] 이미지 {len(self.files)}장, 배치 {self.n}, 캐시 {cache_file}')

        def get_batch_size(self):
            return self.n

        def get_batch(self, names):
            if self.i + self.n > len(self.files):
                return None                                    # 더 없음
            batch = np.concatenate([_preprocess_file(f, self.h, self.w)
                                    for f in self.files[self.i:self.i + self.n]], axis=0)
            self.buf.copy_(torch.from_numpy(np.ascontiguousarray(batch)))
            self.i += self.n
            return [int(self.buf.data_ptr())]

        def read_calibration_cache(self):
            if os.path.exists(self.cache_file):
                log(f'[calib] 캐시 사용: {self.cache_file}')
                return open(self.cache_file, 'rb').read()
            return None

        def write_calibration_cache(self, cache):
            with open(self.cache_file, 'wb') as f:
                f.write(cache)
            log(f'[calib] 캐시 저장: {self.cache_file}')

    return ImageCalibrator()


# ----------------------------------------------------------------------------- 빌드
def build_engine(onnx_path, engine_path, *, precision='fp16', shape=None, dynamic=None,
                 workspace_gib=4.0, opt_level=3, timing_cache=None, tf32=True, sparse=False,
                 detailed_profiling=False, calib_dir=None, calib_cache=None, calib_batch=1,
                 calib_max_images=200, dla_core=None, log=print):
    """ONNX → 직렬화된 TensorRT 엔진 파일. 반환: engine_path.

    shape   : 정적 입력 (N,C,H,W). None 이면 ONNX 에 적힌 shape 그대로.
    dynamic : {'min':(..), 'opt':(..), 'max':(..)} 최적화 프로파일 (배치/해상도 범위).
    """
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    # ONNX 네트워크는 명시적 배치. TRT 10 부터는 기본이라 플래그가 deprecated (경고만).
    major = int(trt.__version__.split('.')[0])
    flags = 0 if major >= 10 else (1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    network = builder.create_network(flags)
    parser = trt.OnnxParser(network, logger)
    with open(onnx_path, 'rb') as f:
        if not parser.parse(f.read()):
            errs = '\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors))
            raise RuntimeError(f'ONNX 파싱 실패: {onnx_path}\n{errs}')

    config = builder.create_builder_config()
    # WORKSPACE (trtexec --memPoolSize=workspace): 빌더가 tactic 을 시험할 때 쓰는 GPU 스크래치.
    # 클수록 좋은 커널을 찾을 여지가 생기지만 VRAM 을 넘으면 안 된다.
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, int(workspace_gib * (1 << 30)))
    # 최적화 레벨 (trtexec --builderOptimizationLevel): 0=빠른 빌드/약한 엔진 … 5=느린 빌드/최선 엔진
    config.builder_optimization_level = int(opt_level)
    # TF32 (기본 on): Ampere+ 에서 FP32 matmul/conv 가속 (가수 정밀도 약간 감소). 엄격 FP32 재현엔 끔.
    if not tf32:
        config.clear_flag(trt.BuilderFlag.TF32)
    # 2:4 구조적 희소성: 가중치가 그 패턴으로 프루닝돼 있을 때만 이득 (아니면 무해한 no-op)
    if sparse:
        config.set_flag(trt.BuilderFlag.SPARSE_WEIGHTS)
    # 레이어별 이름/타입을 엔진에 남겨 프로파일러가 시간을 레이어에 귀속시킬 수 있게 (크기 약간 증가)
    if detailed_profiling:
        config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED
    # 타이밍 캐시 (trtexec --timingCacheFile): tactic 벤치 결과를 재사용해 재빌드 시간을 크게 줄인다
    tcache = None
    if timing_cache:
        blob = open(timing_cache, 'rb').read() if os.path.exists(timing_cache) else b''
        tcache = config.create_timing_cache(blob)
        config.set_timing_cache(tcache, ignore_mismatch=False)
        if blob:
            log(f'[timing] 캐시 로드: {timing_cache}')

    inp = network.get_input(0)
    if dynamic:
        # 동적 shape: min/opt/max 범위를 선언. TRT 는 opt 에 맞춰 커널을 고르고 범위 안은 다 받는다.
        profile = builder.create_optimization_profile()
        profile.set_shape(inp.name, tuple(dynamic['min']), tuple(dynamic['opt']), tuple(dynamic['max']))
        config.add_optimization_profile(profile)
        calib_shape = tuple(dynamic['opt'])
        log(f"[shape] dynamic min={dynamic['min']} opt={dynamic['opt']} max={dynamic['max']}")
    else:
        if shape is not None:
            inp.shape = tuple(shape)
        calib_shape = tuple(inp.shape)
        if any(d < 0 for d in calib_shape):
            raise ValueError(f'ONNX 입력이 동적({calib_shape})인데 shape/dynamic 이 주어지지 않음')
        log(f'[shape] static {calib_shape}')

    # 정밀도
    if precision == 'fp16':
        if not builder.platform_has_fast_fp16:
            log('[warn] 이 GPU 는 빠른 FP16 을 지원하지 않는다고 보고함')
        config.set_flag(trt.BuilderFlag.FP16)        # 레이어별로 FP16 허용 (정확도상 필요한 곳은 FP32 유지)
    elif precision == 'int8':
        if not builder.platform_has_fast_int8:
            log('[warn] 이 GPU 는 빠른 INT8 을 지원하지 않는다고 보고함')
        if not calib_dir:
            raise ValueError('INT8 은 캘리브레이션 이미지 디렉터리(calib_dir)가 필요')
        config.set_flag(trt.BuilderFlag.INT8)
        config.set_flag(trt.BuilderFlag.FP16)        # INT8 커널이 없거나 정확도가 나쁜 레이어는 FP16 으로
        calib_cache = calib_cache or str(Path(engine_path).with_suffix('.calib'))
        cshape = (calib_batch,) + tuple(calib_shape[1:])
        config.int8_calibrator = make_calibrator(trt, calib_dir, calib_cache, cshape, calib_max_images, log)
        if dynamic:
            config.set_calibration_profile(profile)  # 캘리브레이터는 고정 shape 로 돌므로 프로파일을 알려준다
    elif precision != 'fp32':
        raise ValueError(f'precision 은 fp32|fp16|int8: {precision}')
    log(f'[precision] {precision}' + (' (+FP16 fallback)' if precision == 'int8' else ''))

    # DLA (Jetson 의 고정 기능 가속기): FP16/INT8 만, 미지원 레이어는 GPU 로 fallback. 데스크톱 GPU 엔 무의미.
    if dla_core is not None:
        if precision == 'fp32':
            raise ValueError('DLA 는 fp16 또는 int8 필요')
        config.default_device_type = trt.DeviceType.DLA
        config.DLA_core = int(dla_core)
        config.set_flag(trt.BuilderFlag.GPU_FALLBACK)
        log(f'[dla] core {dla_core} (+GPU fallback)')

    log(f'[build] 엔진 빌드 시작 (opt_level={opt_level}, workspace={workspace_gib}GiB) — 수 분 걸릴 수 있음')
    t0 = time.monotonic()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError('엔진 빌드 실패 (build_serialized_network → None)')
    if tcache is not None:
        with open(timing_cache, 'wb') as f:
            f.write(memoryview(tcache.serialize()))
    Path(engine_path).parent.mkdir(parents=True, exist_ok=True)
    with open(engine_path, 'wb') as f:
        f.write(bytes(serialized))
    log(f'[done] {engine_path} ({serialized.nbytes / (1 << 20):.1f} MB, {time.monotonic() - t0:.0f}s)')
    return str(engine_path)


# ----------------------------------------------------------------------------- 러너
class EngineRunner:
    """엔진 로드 + I/O 버퍼(torch) + 실행. 입력 numpy → 출력 {이름: numpy}."""

    def __init__(self, engine_path, input_shape=None):
        import tensorrt as trt
        import torch
        self.torch = torch
        self.logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, '')
        with open(engine_path, 'rb') as f, trt.Runtime(self.logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.in_name = next(n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        if input_shape is not None or -1 in tuple(self.engine.get_tensor_shape(self.in_name)):
            if input_shape is None:
                raise ValueError('동적 엔진은 input_shape 가 필요')
            self.ctx.set_input_shape(self.in_name, tuple(input_shape))   # 동적 엔진: 실제 shape 확정
        to_torch = {np.float32: torch.float32, np.float16: torch.float16,
                    np.int32: torch.int32, np.int64: torch.int64, np.bool_: torch.bool}
        self.inputs, self.outputs = {}, {}
        for n in names:
            shape = tuple(self.ctx.get_tensor_shape(n))
            dtype = to_torch[np.dtype(trt.nptype(self.engine.get_tensor_dtype(n))).type]
            buf = torch.empty(shape, dtype=dtype, device='cuda')
            (self.inputs if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT else self.outputs)[n] = buf
            self.ctx.set_tensor_address(n, buf.data_ptr())
        self.input_shape = tuple(self.inputs[self.in_name].shape)
        self.input_dtype = self.inputs[self.in_name].dtype
        self.stream = torch.cuda.Stream()       # 전용 스트림 (기본 스트림은 TRT 가 추가 동기화로 느려짐)

    def run(self, inp):
        torch = self.torch
        with torch.cuda.stream(self.stream):
            self.inputs[self.in_name].copy_(torch.from_numpy(np.ascontiguousarray(inp)).to(self.input_dtype))
            self.ctx.execute_async_v3(self.stream.cuda_stream)
            outs = {n: t.float().cpu() for n, t in self.outputs.items()}
        self.stream.synchronize()
        return {n: t.numpy() for n, t in outs.items()}


# ----------------------------------------------------------------------------- 검증
def verify_engine(onnx_path, engine_path, images=None, n=3, threshold=0.3, log=print):
    """같은 입력을 ONNX Runtime 과 엔진에 넣어 비교한다.

    DETR 은 쿼리 300개 중 대부분이 점수가 낮은 '버릴' 쿼리라 그 박스는 정밀도에 따라 크게 요동한다
    → 전체 최대 차는 의미가 없다. ONNX 기준 점수가 threshold 이상인 쿼리(실제 검출)만 골라
    점수 차·박스 차(정규화 좌표)를 잰다. 반환: {'score_diff', 'box_diff', 'ms', ...}.
    """
    import onnxruntime as ort
    runner = EngineRunner(engine_path)
    _, _, h, w = runner.input_shape
    sess = ort.InferenceSession(onnx_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    in_name = sess.get_inputs()[0].name
    out_names = [o.name for o in sess.get_outputs()]
    files = _list_images(images)[:n] if images else []
    rng = np.random.default_rng(0)
    score_diff = box_diff = 0.0
    n_ref = n_eng = 0
    times = []
    for i in range(n):
        x = _preprocess_file(files[i], h, w) if i < len(files) else rng.standard_normal((1, 3, h, w)).astype(np.float32)
        ref = dict(zip(out_names, sess.run(None, {in_name: x})))
        t0 = time.monotonic()
        out = runner.run(x)
        times.append((time.monotonic() - t0) * 1000)
        s_r = 1 / (1 + np.exp(-ref['labels'][0])); s_e = 1 / (1 + np.exp(-out['labels'][0]))   # (Q,C)
        q = np.where(s_r.max(axis=1) >= threshold)[0]            # ONNX 기준 실제 검출 쿼리
        n_ref += len(q); n_eng += int((s_e.max(axis=1) >= threshold).sum())
        if len(q):
            score_diff = max(score_diff, float(np.abs(s_e[q].max(axis=1) - s_r[q].max(axis=1)).max()))
            box_diff = max(box_diff, float(np.abs(out['dets'][0][q] - ref['dets'][0][q]).max()))
        log(f'[verify] 입력{i}: 검출 수 ONNX {len(q)} / 엔진 {int((s_e.max(axis=1) >= threshold).sum())}, '
            f'최고 점수 ONNX {s_r.max():.3f} / 엔진 {s_e.max():.3f}')
    ms = float(np.median(times[1:] or times))
    log(f'[verify] 검출 쿼리 기준 최대 차 — 점수 {score_diff:.4f}, 박스(정규화) {box_diff:.4f} '
        f'| 검출 수 합 ONNX {n_ref} / 엔진 {n_eng} | 엔진 지연 {ms:.1f} ms')
    return {'score_diff': score_diff, 'box_diff': box_diff, 'n_ref': n_ref, 'n_eng': n_eng, 'ms': ms}


# ----------------------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser(description='ONNX → TensorRT 엔진 빌드 (RF-DETR)',
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--onnx', help='입력 ONNX. 없으면 --model 로 rfdetr 에서 export')
    ap.add_argument('--model', default='medium', choices=['nano', 'small', 'medium', 'large'])
    ap.add_argument('--out', default='/overlay_ws/models/trt/manual', help='export/엔진 출력 디렉터리')
    ap.add_argument('--engine', help='엔진 파일 경로 (기본: <out>/engine_<precision>.trt)')
    ap.add_argument('--precision', default='fp16', choices=['fp32', 'fp16', 'int8'])
    ap.add_argument('--shape', type=int, nargs=4, metavar=('N', 'C', 'H', 'W'), help='정적 입력 shape (기본: ONNX 값)')
    ap.add_argument('--dynamic', action='store_true', help='동적 shape 엔진 (--min/--opt/--max-shape)')
    ap.add_argument('--min-shape', type=int, nargs=4, default=[1, 3, 576, 576])
    ap.add_argument('--opt-shape', type=int, nargs=4, default=[1, 3, 576, 576])
    ap.add_argument('--max-shape', type=int, nargs=4, default=[4, 3, 576, 576])
    ap.add_argument('--workspace', type=float, default=4.0, help='WORKSPACE GiB')
    ap.add_argument('--opt-level', type=int, default=3, choices=range(0, 6))
    ap.add_argument('--timing-cache', help='타이밍 캐시 파일 (재빌드 가속)')
    ap.add_argument('--no-tf32', action='store_true')
    ap.add_argument('--sparse-weights', action='store_true')
    ap.add_argument('--detailed-profiling', action='store_true')
    ap.add_argument('--dla-core', type=int)
    ap.add_argument('--calib-dir', help='INT8 캘리브레이션 이미지 디렉터리')
    ap.add_argument('--calib-cache', help='INT8 캘리브레이션 캐시 파일')
    ap.add_argument('--calib-batch', type=int, default=1)
    ap.add_argument('--calib-max-images', type=int, default=200)
    ap.add_argument('--no-verify', action='store_true', help='빌드 후 ONNX 대비 검증 생략')
    ap.add_argument('--verify-images', help='검증 입력 이미지 디렉터리 (기본: 난수 / int8 은 calib-dir)')
    ap.add_argument('--verify-num', type=int, default=3)
    a = ap.parse_args()

    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    onnx_path = a.onnx
    if not onnx_path:
        os.environ.setdefault('RF_HOME', '/overlay_ws/models')
        import rfdetr
        cls = {'nano': rfdetr.RFDETRNano, 'small': rfdetr.RFDETRSmall,
               'medium': rfdetr.RFDETRMedium, 'large': rfdetr.RFDETRLarge}[a.model]
        print(f'[export] rfdetr {a.model} → ONNX ({out})')
        onnx_path = str(cls().export(output_dir=str(out), format='onnx', verbose=False))
    engine = a.engine or str(out / f'engine_{a.precision}.trt')
    dynamic = {'min': a.min_shape, 'opt': a.opt_shape, 'max': a.max_shape} if a.dynamic else None
    build_engine(onnx_path, engine, precision=a.precision, shape=a.shape, dynamic=dynamic,
                 workspace_gib=a.workspace, opt_level=a.opt_level, timing_cache=a.timing_cache,
                 tf32=not a.no_tf32, sparse=a.sparse_weights, detailed_profiling=a.detailed_profiling,
                 calib_dir=a.calib_dir, calib_cache=a.calib_cache, calib_batch=a.calib_batch,
                 calib_max_images=a.calib_max_images, dla_core=a.dla_core)
    if not a.no_verify and not a.dynamic:
        verify_engine(onnx_path, engine, images=a.verify_images or (a.calib_dir if a.precision == 'int8' else None),
                      n=a.verify_num)


if __name__ == '__main__':
    main()
