"""Unit tests for the road segmentation wrapper and node helpers. No model or GPU needed."""
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from ap1_perception.road_seg import RoadSeg
from ap1_perception.road_seg_node import scale_camera_info


def bare_road_seg(input_h=448, input_w=768):
    """A RoadSeg without an onnxruntime session, for testing its pure methods."""
    seg = RoadSeg.__new__(RoadSeg)
    seg.input_h, seg.input_w = input_h, input_w
    return seg


class FakeSession:
    """Stands in for onnxruntime: returns a class map of the input's size, filled with 7s."""

    def run(self, _outputs, feeds):
        blob = next(iter(feeds.values()))
        return [np.full((1, *blob.shape[2:]), 7, dtype=np.uint8)]


def test_preprocess_1080p_resizes_and_pads():
    img = np.zeros((1080, 1920, 3), dtype=np.uint8)
    img[..., 0] = 255  # red, in RGB order
    blob, (top, left, h, w) = bare_road_seg()._preprocess(img)

    assert blob.shape == (1, 3, 448, 768)
    assert blob.dtype == np.float32
    assert (top, left, h, w) == (8, 0, 432, 768)  # 16 rows of padding, split top and bottom
    assert blob[0, :, 0, 0] == pytest.approx([114 / 255] * 3)  # padding colour
    assert blob[0, :, 447, 0] == pytest.approx([114 / 255] * 3)
    assert blob[0, :, 224, 384].tolist() == [1.0, 0.0, 0.0]  # RGB stays RGB


def test_preprocess_other_aspect_pads_sides():
    # 4:3 image: limited by height, so padding goes left and right
    blob, (top, left, h, w) = bare_road_seg()._preprocess(np.zeros((480, 640, 3), dtype=np.uint8))
    assert blob.shape == (1, 3, 448, 768)
    assert (top, h) == (0, 448)
    assert w == round(640 * 448 / 480)
    assert left == (768 - w) // 2


def test_call_crops_padding():
    seg = bare_road_seg()
    seg.session = FakeSession()
    seg.input_name = 'images'
    class_map = seg(np.zeros((1080, 1920, 3), dtype=np.uint8))
    assert class_map.shape == (432, 768)
    assert class_map.dtype == np.uint8
    assert class_map.flags['C_CONTIGUOUS']
    assert (class_map == 7).all()


def test_scale_camera_info_to_class_map():
    info = CameraInfo(width=1920, height=1080, distortion_model='plumb_bob', d=[0.1, 0.0, 0.0, 0.0, 0.0])
    info.k = [1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0]
    info.p = [1000.0, 0.0, 960.0, 0.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0]

    out = scale_camera_info(info, 768, 432)

    assert (out.width, out.height) == (768, 432)
    np.testing.assert_allclose(out.k, [400.0, 0.0, 384.0, 0.0, 400.0, 216.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(out.p, [400.0, 0.0, 384.0, 0.0, 0.0, 400.0, 216.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    assert list(out.d) == list(info.d)  # distortion doesn't scale
