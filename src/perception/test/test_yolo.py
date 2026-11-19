"""Unit tests for the YOLO wrapper's pre- and post-processing. No model or GPU needed."""
import numpy as np
import pytest

from ap1_perception.yolo import _yolo
from ap1_perception.yolo._yolo import CPU_PROVIDER, CUDA_PROVIDER, YOLO


def bare_yolo(dynamic=False, classes=None, conf=0.25, iou=0.7, max_det=300):
    """A YOLO without an onnxruntime session, for testing its pure methods."""
    yolo = YOLO.__new__(YOLO)
    yolo.dynamic = dynamic
    yolo.classes = None if classes is None else np.asarray(classes)
    yolo.conf = conf
    yolo.iou = iou
    yolo.max_det = max_det
    return yolo


# onnxruntime's real providers depend on the image (CPU or GPU), so fake them
@pytest.fixture
def cuda_available(monkeypatch):
    monkeypatch.setattr(_yolo.ort, 'get_available_providers', lambda: [CUDA_PROVIDER, CPU_PROVIDER])


@pytest.fixture
def cuda_missing(monkeypatch):
    monkeypatch.setattr(_yolo.ort, 'get_available_providers', lambda: [CPU_PROVIDER])


def test_cpu_never_uses_cuda(cuda_available):
    assert _yolo.select_providers('cpu') == [CPU_PROVIDER]


def test_cuda_with_cuda(cuda_available):
    assert _yolo.select_providers('cuda') == [CUDA_PROVIDER, CPU_PROVIDER]


def test_cuda_without_cuda_fails(cuda_missing):
    with pytest.raises(RuntimeError):
        _yolo.select_providers('cuda')


def test_auto_prefers_cuda(cuda_available):
    assert _yolo.select_providers('auto') == [CUDA_PROVIDER, CPU_PROVIDER]


def test_auto_falls_back_to_cpu(cuda_missing):
    assert _yolo.select_providers('auto') == [CPU_PROVIDER]


def test_unknown_device_fails():
    with pytest.raises(ValueError):
        _yolo.select_providers('tpu')


def test_greedy_nms_drops_overlap_keeps_disjoint():
    # Sorted by score. Boxes 0 and 1 overlap with IoU 0.81, box 2 is elsewhere
    boxes = np.array([[0, 0, 10, 10], [1, 1, 10, 10], [20, 20, 30, 30]], dtype=np.float32)
    assert _yolo._greedy_nms(boxes, iou_thresh=0.7).tolist() == [0, 2]
    assert _yolo._greedy_nms(boxes, iou_thresh=0.9).tolist() == [0, 1, 2]


def raw_predictions():
    """Raw model output rows: cx, cy, w, h, then one score per class (3 classes)."""
    return np.array([
        [50, 50, 20, 20, 0.90, 0.00, 0.00],  # class 0
        [51, 51, 20, 20, 0.80, 0.00, 0.00],  # class 0, overlaps the first: suppressed
        [50, 50, 20, 20, 0.00, 0.85, 0.00],  # class 1, same place: NMS is per class
        [90, 90, 10, 10, 0.10, 0.00, 0.00],  # below the confidence threshold
    ], dtype=np.float32)


def test_nms_per_class_and_confidence():
    boxes, scores, classes = bare_yolo()._nms(raw_predictions())
    assert classes.tolist() == [0, 1]
    assert scores.tolist() == pytest.approx([0.90, 0.85])
    np.testing.assert_allclose(boxes, [[40, 40, 60, 60], [40, 40, 60, 60]])


def test_nms_class_filter():
    _, _, classes = bare_yolo(classes=[1])._nms(raw_predictions())
    assert classes.tolist() == [1]


def test_preprocess_letterboxes_to_model_size():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[..., 0] = 255  # blue, in OpenCV's BGR order
    blob, gain, pad = bare_yolo(dynamic=False)._preprocess(img)

    assert blob.shape == (1, 3, 640, 640)
    assert blob.dtype == np.float32
    assert gain == (1.0, 1.0)
    assert pad == (0, 80)  # 160 rows of padding, split top and bottom
    assert blob[0, :, 0, 0] == pytest.approx([114 / 255] * 3)  # padding colour
    assert blob[0, :, 320, 320].tolist() == [0.0, 0.0, 1.0]   # BGR -> RGB


def test_preprocess_dynamic_pads_to_stride_only():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    blob, gain, pad = bare_yolo(dynamic=True)._preprocess(img)
    assert blob.shape == (1, 3, 480, 640)
    assert pad == (0, 0)


def test_preprocess_scales_down_large_frames():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    blob, gain, pad = bare_yolo(dynamic=False)._preprocess(img)
    assert blob.shape == (1, 3, 640, 640)
    assert gain == (0.5, 0.5)
    assert pad == (0, 140)
