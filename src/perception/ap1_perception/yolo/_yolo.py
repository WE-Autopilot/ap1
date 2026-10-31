"""YOLO object detection on ONNX Runtime.

Runs an Ultralytics-exported YOLO detection model (e.g. yolo11n.onnx, exported
with nms=False) without depending on ultralytics or torch. Pre- and
post-processing mirror ultralytics' predict defaults, so detections match what
`ultralytics.YOLO(model).predict(img)` returned before.

The same code runs on CPU and GPU. Which one is used depends on the installed
onnxruntime package (onnxruntime vs onnxruntime-gpu) and the `device` argument.
"""
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

from ..projection import box_select_points

DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "yolo11n.onnx"

CPU_PROVIDER = "CPUExecutionProvider"
CUDA_PROVIDER = "CUDAExecutionProvider"

# Ultralytics predict defaults
_IMGSZ = 640
_STRIDE = 32
_PAD_VALUE = 114
_MAX_NMS = 30000
_MAX_WH = 7680  # per-class box offset that makes NMS class-aware


def select_providers(device: str) -> list[str]:
    """Map a device name ("auto", "cuda", "cpu") to ONNX Runtime providers."""
    available = ort.get_available_providers()
    if device == "cpu":
        return [CPU_PROVIDER]
    if device == "cuda":
        if CUDA_PROVIDER not in available:
            raise RuntimeError(
                f"device='cuda' requested but onnxruntime has no CUDA support "
                f"(available providers: {available}). Use the GPU image, or device='cpu'."
            )
        return [CUDA_PROVIDER, CPU_PROVIDER]
    if device == "auto":
        return [CUDA_PROVIDER, CPU_PROVIDER] if CUDA_PROVIDER in available else [CPU_PROVIDER]
    raise ValueError(f"Unknown device '{device}', expected 'auto', 'cuda' or 'cpu'")


class YOLO:
    def __init__(self, classes=(11,), K=None, model_path=None, device="auto",
                 conf=0.25, iou=0.7, max_det=300):
        model_path = Path(model_path) if model_path else DEFAULT_MODEL_PATH
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found at '{model_path}'")

        self.session = ort.InferenceSession(str(model_path), providers=select_providers(device))
        # onnxruntime silently falls back to CPU if CUDA libraries fail to load
        if device == "cuda" and CUDA_PROVIDER not in self.session.get_providers():
            raise RuntimeError(
                "device='cuda' requested but the CUDA provider failed to load "
                "(check the onnxruntime log above for the missing library)."
            )

        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        # A fixed-size export needs exactly imgsz x imgsz; a dynamic one takes the
        # smallest stride-aligned rectangle, which is what ultralytics does.
        self.dynamic = not all(isinstance(d, int) for d in model_input.shape[2:])

        self.K = K
        self.classes = None if classes is None else np.asarray(classes)
        self.conf = conf
        self.iou = iou
        self.max_det = max_det

    @property
    def provider(self) -> str:
        """The execution provider inference actually runs on."""
        return self.session.get_providers()[0]

    def forward(self, img):
        """Detect objects in a BGR uint8 image.

        Returns (boxes, classes): boxes is (N, 4) xyxy in pixels of `img`, classes is (N,).
        """
        blob, gain, pad = self._preprocess(img)
        pred = self.session.run(None, {self.input_name: blob})[0][0]  # (4 + nc, anchors)
        boxes, _, cls = self._nms(pred.T)

        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad[0]) / gain[0]
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad[1]) / gain[1]
        h, w = img.shape[:2]
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h)
        return boxes, cls

    def __call__(self, img, points, K=None):
        if K is None and self.K is None:
            raise Exception("No camera intrensics (K) provided during call or init.")

        if K is None:
            K = self.K

        boxes, img_cls = self.forward(img)
        centroids = box_select_points(K, points, boxes)

        return centroids, img_cls

    def _preprocess(self, img):
        """Letterbox to the model input size; returns the NCHW blob, (gain_x, gain_y) and (pad_x, pad_y)."""
        h, w = img.shape[:2]
        r = min(_IMGSZ / h, _IMGSZ / w)
        new_w, new_h = max(round(w * r), 1), max(round(h * r), 1)
        dw, dh = _IMGSZ - new_w, _IMGSZ - new_h
        if self.dynamic:
            dw, dh = dw % _STRIDE, dh % _STRIDE
        dw, dh = dw / 2, dh / 2
        top, bottom = round(dh - 0.1), round(dh + 0.1)
        left, right = round(dw - 0.1), round(dw + 0.1)

        if (w, h) != (new_w, new_h):
            img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT,
                                 value=(_PAD_VALUE,) * 3)

        blob = img[..., ::-1].transpose(2, 0, 1)[np.newaxis]  # BGR HWC -> RGB NCHW
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        return blob, (new_w / w, new_h / h), (left, top)

    def _nms(self, pred):
        """Filter raw (anchors, 4 + nc) predictions into (boxes xyxy, scores, classes)."""
        scores_all = pred[:, 4:]
        keep = scores_all.max(1) > self.conf
        pred, scores_all = pred[keep], scores_all[keep]

        cls = scores_all.argmax(1)
        scores = scores_all[np.arange(len(cls)), cls]
        if self.classes is not None:
            keep = np.isin(cls, self.classes)
            pred, scores, cls = pred[keep], scores[keep], cls[keep]

        order = np.argsort(-scores, kind="stable")[:_MAX_NMS]
        xywh, scores, cls = pred[order, :4], scores[order], cls[order]
        boxes = np.empty_like(xywh)
        boxes[:, :2] = xywh[:, :2] - xywh[:, 2:] / 2
        boxes[:, 2:] = xywh[:, :2] + xywh[:, 2:] / 2

        kept = _greedy_nms(boxes + (cls * _MAX_WH)[:, None], self.iou)[:self.max_det]
        return boxes[kept], scores[kept], cls[kept].astype(np.float32)


def _greedy_nms(boxes, iou_thresh):
    """Standard greedy NMS over boxes already sorted by descending score."""
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1) * (y2 - y1)
    remaining = np.arange(len(boxes))
    kept = []
    while remaining.size:
        i = remaining[0]
        kept.append(i)
        rest = remaining[1:]
        inter_w = (np.minimum(x2[i], x2[rest]) - np.maximum(x1[i], x1[rest])).clip(0)
        inter_h = (np.minimum(y2[i], y2[rest]) - np.maximum(y1[i], y1[rest])).clip(0)
        inter = inter_w * inter_h
        iou = inter / (areas[i] + areas[rest] - inter)
        remaining = rest[iou <= iou_thresh]
    return np.array(kept, dtype=np.int64)
