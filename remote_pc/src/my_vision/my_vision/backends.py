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

    def __init__(self, model, size, resolution, num_classes, weights_dir, threshold, fp16=True, log=print):
        import tensorrt as trt
        import torch
        from rfdetr.export._onnx.inference import (_exclude_background_class, _preprocess_pil_to_nchw,
                                                    _select_topk_multiclass)
        self._pre, self._exclude, self._topk = _preprocess_pil_to_nchw, _exclude_background_class, _select_topk_multiclass
        self.threshold = threshold
        self.torch = torch

        gpu = re.sub(r'[^A-Za-z0-9]+', '-', torch.cuda.get_device_name(0)).strip('-').lower()
        self.dir = Path(weights_dir) / 'trt' / f'rf-detr-{size}-{resolution}-trt{trt.__version__}-{gpu}'
        engine_path = next(self.dir.glob('*.trt'), None)     # 파일명은 rfdetr 버전마다 달라 가정하지 않는다
        if engine_path is None:
            log(f'TensorRT 엔진 없음 → 이 컨테이너에서 ONNX export + 엔진 빌드 (수 분 소요): {self.dir}')
            self.dir.mkdir(parents=True, exist_ok=True)
            engine_path = Path(model.export(output_dir=str(self.dir), format='tensorrt', fp16=fp16, verbose=False))
            log(f'엔진 빌드 완료: {engine_path.name}')

        self.logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, '')
        with open(engine_path, 'rb') as f, trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()

        # I/O 텐서마다 GPU 버퍼(torch 텐서)를 잡고 주소를 엔진에 고정한다
        to_torch = {np.float32: torch.float32, np.float16: torch.float16,
                    np.int32: torch.int32, np.int64: torch.int64, np.bool_: torch.bool}
        self.inputs, self.outputs = {}, {}
        for i in range(self.engine.num_io_tensors):
            n = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(n))
            dtype = to_torch[np.dtype(trt.nptype(self.engine.get_tensor_dtype(n))).type]
            buf = torch.empty(shape, dtype=dtype, device='cuda')
            (self.inputs if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT else self.outputs)[n] = buf
            self.ctx.set_tensor_address(n, buf.data_ptr())
        self.in_name = next(iter(self.inputs))
        _, _, self.h, self.w = self.inputs[self.in_name].shape
        self.dets_name = next(n for n in self.outputs if 'dets' in n)
        self.labels_name = next(n for n in self.outputs if 'labels' in n)
        # 마지막 클래스 슬롯이 배경이면 제외 (logits 폭 = 클래스 수 + 1 인 경우)
        n_slots = self.outputs[self.labels_name].shape[-1]
        self.background_id = -1 if n_slots == num_classes + 1 else None
        self.stream = torch.cuda.Stream()   # 전용 스트림 (기본 스트림은 TensorRT 가 추가 동기화로 느려짐)
        log(f'TensorRT {trt.__version__} 엔진 로드: 입력 {self.w}x{self.h} '
            f'{"fp16" if self.inputs[self.in_name].dtype == torch.float16 else "fp32"} I/O, '
            f'쿼리 {self.outputs[self.dets_name].shape[1]}, 클래스 슬롯 {n_slots}')
        self.infer(np.zeros((self.h, self.w, 3), np.uint8))   # 워밍업

    def infer(self, rgb):
        from PIL import Image
        torch = self.torch
        inp = self._pre(Image.fromarray(rgb), self.h, self.w, 3)             # (1,3,H,W) float32, rfdetr 규약
        with torch.cuda.stream(self.stream):
            self.inputs[self.in_name].copy_(torch.from_numpy(inp).to(self.inputs[self.in_name].dtype))
            self.ctx.execute_async_v3(self.stream.cuda_stream)
            boxes = self.outputs[self.dets_name][0].float().cpu()            # (Q,4) 정규화 cxcywh
            logits = self.outputs[self.labels_name][0].float().cpu()         # (Q,C)
        self.stream.synchronize()
        boxes, logits = boxes.numpy(), logits.numpy()

        scores_all = 1.0 / (1.0 + np.exp(-np.clip(logits, -88, 88)))
        scores_all, class_ids = self._exclude(scores_all, self.background_id)
        scores, cls, qidx = self._topk(scores_all, self.threshold, num_select=boxes.shape[0])
        cls = class_ids[cls]
        cx, cy, bw, bh = boxes[qidx].T
        h, w = rgb.shape[:2]
        xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1) * np.array([w, h, w, h], np.float32)
        return xyxy, scores, cls.astype(int)
