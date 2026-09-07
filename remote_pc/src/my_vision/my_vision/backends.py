# =============================================================================
# backends.py — 검출 추론 백엔드: torch (rfdetr 그대로) / tensorrt (엔진)
#
# 둘 다 같은 인터페이스: infer(rgb HxWx3 uint8) → (xyxy 픽셀 (N,4), 점수 (N,), 클래스 id (N,))
#
# [TensorRT 는 왜 "이 컨테이너 안에서" 빌드하나]
#   TensorRT 엔진(.trt)은 빌드한 TensorRT 버전·CUDA·GPU 아키텍처에 묶여 이식되지 않는다.
#   그래서 이식 가능한 ONNX 만 산출물로 보고, 엔진은 "실행할 환경에서 만드는 캐시"로 취급한다:
#   첫 실행 때 rfdetr 의 export(format='tensorrt') 로 ONNX → 엔진을 이 프로세스 안에서 빌드
#   (polygraphy, trtexec 불필요)하고 models/trt/<크기-해상도-TRT버전-GPU>/ 에 캐시한다.
#   디렉터리 이름에 환경을 새겨 두어, 이미지나 GPU 가 바뀌면 자동으로 다시 빌드된다.
#
# [빌드 옵션] 엔진 빌드는 trt_build.build_engine (TensorRT Python API 직접) — fp32/fp16/int8,
#   최적화 레벨·workspace·타이밍 캐시·TF32 등을 노드 파라미터(trt_*)로 조절한다.
#
# [전처리·후처리는 rfdetr 의 함수를 재사용]
#   리사이즈 규약(bilinear, antialias 없음)·ImageNet 정규화·배경 클래스 제외·top-k 선택은
#   rfdetr 이 ONNX 추론용으로 제공하는 함수를 그대로 import 해 torch 경로와 수치가 일치하게 한다.
#   (직접 재구현하면 미묘한 차이로 점수가 어긋나기 쉽다 — rfdetr 문서에 명시된 함정)
# =============================================================================
import re
from pathlib import Path

import numpy as np


class TorchBackend:
    name = 'torch'

    def __init__(self, model, threshold):
        self.model, self.threshold = model, threshold

    def infer(self, rgb):
        d = self.model.predict(rgb, threshold=self.threshold)
        return d.xyxy, d.confidence, d.class_id


class TensorRTBackend:
    name = 'tensorrt'

    #: 노드 파라미터 → build_engine 인자 기본값 (trt_build.build_engine 참고)
    DEFAULT_OPTS = dict(precision='fp16', opt_level=3, workspace_gib=4.0, tf32=True,
                        timing_cache=True, calib_dir='', verify=True)

    def __init__(self, model, size, resolution, num_classes, weights_dir, threshold, opts=None, log=print):
        import tensorrt as trt
        import torch
        from rfdetr.export._onnx.inference import (_exclude_background_class, _preprocess_pil_to_nchw,
                                                    _select_topk_multiclass)
        from my_vision.trt_build import EngineRunner, build_engine, verify_engine
        self._pre, self._exclude, self._topk = _preprocess_pil_to_nchw, _exclude_background_class, _select_topk_multiclass
        self.threshold = threshold
        o = dict(self.DEFAULT_OPTS, **(opts or {}))

        # 캐시 키에 환경(TRT 버전·GPU)과 정밀도를 새긴다 → 바뀌면 자동 재빌드, fp16/int8 이 섞이지 않음
        gpu = re.sub(r'[^A-Za-z0-9]+', '-', torch.cuda.get_device_name(0)).strip('-').lower()
        self.dir = Path(weights_dir) / 'trt' / f'rf-detr-{size}-{resolution}-{o["precision"]}-trt{trt.__version__}-{gpu}'
        engine_path = self.dir / 'engine.trt'
        if not engine_path.exists():
            self.dir.mkdir(parents=True, exist_ok=True)
            onnx_path = next(self.dir.glob('*.onnx'), None)          # 이식 가능한 산출물: 있으면 재사용
            if onnx_path is None:
                log(f'ONNX 없음 → rfdetr export: {self.dir}')
                onnx_path = Path(model.export(output_dir=str(self.dir), format='onnx', verbose=False))
            log(f'TensorRT 엔진 없음 → 이 컨테이너에서 빌드 ({o["precision"]}, opt_level {o["opt_level"]})')
            build_engine(str(onnx_path), str(engine_path), precision=o['precision'],
                         opt_level=o['opt_level'], workspace_gib=o['workspace_gib'], tf32=o['tf32'],
                         timing_cache=str(self.dir / 'timing.cache') if o['timing_cache'] else None,
                         calib_dir=o['calib_dir'] or None, log=log)
            if o['verify']:   # 같은 입력으로 ONNX Runtime 과 비교해 수치 차이를 기록
                verify_engine(str(onnx_path), str(engine_path),
                              images=o['calib_dir'] or None, n=3, threshold=threshold, log=log)

        self.runner = EngineRunner(str(engine_path))
        _, _, self.h, self.w = self.runner.input_shape
        self.dets_name = next(n for n in self.runner.outputs if 'dets' in n)
        self.labels_name = next(n for n in self.runner.outputs if 'labels' in n)
        # 마지막 클래스 슬롯이 배경이면 제외 (logits 폭 = 클래스 수 + 1 인 경우)
        n_slots = self.runner.outputs[self.labels_name].shape[-1]
        self.background_id = -1 if n_slots == num_classes + 1 else None
        log(f'TensorRT {trt.__version__} 엔진 로드 ({o["precision"]}): 입력 {self.w}x{self.h}, '
            f'쿼리 {self.runner.outputs[self.dets_name].shape[1]}, 클래스 슬롯 {n_slots} — {engine_path}')
        self.infer(np.zeros((self.h, self.w, 3), np.uint8))   # 워밍업

    def infer(self, rgb):
        from PIL import Image
        inp = self._pre(Image.fromarray(rgb), self.h, self.w, 3)   # (1,3,H,W) float32, rfdetr 규약
        out = self.runner.run(inp)
        boxes, logits = out[self.dets_name][0], out[self.labels_name][0]   # (Q,4) 정규화 cxcywh, (Q,C)

        scores_all = 1.0 / (1.0 + np.exp(-np.clip(logits, -88, 88)))
        scores_all, class_ids = self._exclude(scores_all, self.background_id)
        scores, cls, qidx = self._topk(scores_all, self.threshold, num_select=boxes.shape[0])
        cls = class_ids[cls]
        cx, cy, bw, bh = boxes[qidx].T
        h, w = rgb.shape[:2]
        xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1) * np.array([w, h, w, h], np.float32)
        return xyxy, scores, cls.astype(int)
