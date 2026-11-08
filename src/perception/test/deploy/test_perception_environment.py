"""Checks that perception's dependencies import and work together, and that YOLO runs.

Needs no camera data or ROS graph.
"""
import os
from pathlib import Path

import numpy as np
from perception_common import MODEL_PATH, pick_device


def test_imports():
    import ap1_msgs.msg  # noqa: F401
    import cv2  # noqa: F401
    import onnxruntime  # noqa: F401
    import rclpy  # noqa: F401
    import sensor_msgs_py.point_cloud2  # noqa: F401

    import ap1_perception.yolo_node  # noqa: F401


def test_cv_bridge_roundtrip():
    # cv_bridge is built against apt's numpy; a pip numpy upgrade breaks this
    from cv_bridge import CvBridge
    bridge = CvBridge()
    img = np.random.randint(0, 255, (48, 64, 3), dtype=np.uint8)
    msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    assert np.array_equal(img, bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))


def test_gpu_image_has_cuda_onnxruntime():
    import onnxruntime
    available = onnxruntime.get_available_providers()
    if os.environ.get('AP1_GPU') == '1':
        assert 'CUDAExecutionProvider' in available, f'GPU image without CUDA provider: {available}'


def test_yolo_node_installed():
    from ament_index_python.packages import get_package_prefix
    exe = Path(get_package_prefix('ap1_perception')) / 'lib' / 'ap1_perception' / 'yolo_node'
    assert exe.is_file(), f'{exe} missing'


def test_yolo_inference():
    from ap1_perception.yolo import YOLO
    device = pick_device()
    detector = YOLO(model_path=MODEL_PATH, device=device)
    if device == 'cuda':
        assert detector.provider == 'CUDAExecutionProvider'
    boxes, classes = detector.forward(np.zeros((480, 640, 3), dtype=np.uint8))
    assert boxes.shape[1] == 4
    assert len(boxes) == len(classes)
