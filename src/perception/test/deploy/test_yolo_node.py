"""End-to-end test of yolo_node over ROS.

Launches yolo_node and plays the role of the RealSense driver: publishes a
stop-sign photo on the image topic the node subscribes to, then checks that it
publishes the stop sign on ap1/perception/objects_2d as a Detection2DArray
(issue #116): class ObjectClass.SIGN_STOP, the image's stamp and frame, and a
pixel box around the sign. It then feeds a blank frame and checks the node
still publishes, with an empty array.

The photo is COCO val2017 image 724. It's downloaded into a temporary folder,
checked against a fixed SHA-256, and deleted as soon as it's loaded. A frame
from our own campus dataset will replace it.
"""
import hashlib
import tempfile
import time
import unittest
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
from ap1_msgs.msg import ObjectClass
from cv_bridge import CvBridge
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from perception_common import MODEL_PATH, pick_device
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray

# Topics yolo_node uses (ap1_perception/yolo_node.py)
COLOR_TOPIC = 'camera/camera/color/image_raw'
DETECTIONS_TOPIC = 'ap1/perception/objects_2d'
CAMERA_FRAME = 'camera_color_optical_frame'

IMAGE_URL = 'http://images.cocodataset.org/val2017/000000000724.jpg'
IMAGE_SHA256 = '5c0e559c75d3969c8e3e297b61f61063f78045c9d4802b526ba616361f3823fd'

# The stop sign's pixel box in the photo, found from its red pixels, not from
# YOLO: (124, 76) to (251, 221).
SIGN_CENTER_PX = (187.5, 148.5)  # u, v
SIGN_SIZE_PX = (127.0, 145.0)    # width, height
PIXEL_TOL = 12.0  # px, a box-edge's worth of slack
PUBLISH_HZ = 5.0
MESSAGES_NEEDED = 5
TIMEOUT = 60.0     # s, for each wait


@pytest.mark.launch_test
def generate_test_description():
    yolo_node = Node(
        package='ap1_perception',
        executable='yolo_node',
        output='screen',
        parameters=[{'model_path': str(MODEL_PATH), 'device': pick_device()}],
    )
    return LaunchDescription([yolo_node, ReadyToTest()])


def load_test_image() -> np.ndarray:
    with tempfile.TemporaryDirectory() as tmp:
        with urllib.request.urlopen(IMAGE_URL, timeout=30) as response:
            data = response.read()
        digest = hashlib.sha256(data).hexdigest()
        if digest != IMAGE_SHA256:
            raise RuntimeError(f'{IMAGE_URL} has SHA-256 {digest}, expected {IMAGE_SHA256}')
        path = Path(tmp) / Path(IMAGE_URL).name
        path.write_bytes(data)
        img = cv2.imread(str(path))
    if img is None:
        raise RuntimeError(f'could not decode {IMAGE_URL}')
    return img


class CameraFeeder(RclpyNode):
    """Publishes one image as a camera stream and records detections."""

    def __init__(self, img: np.ndarray):
        super().__init__('perception_e2e_feeder')
        bridge = CvBridge()
        self._image = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self._blank = bridge.cv2_to_imgmsg(np.zeros_like(img), encoding='bgr8')
        self.send_blank = False
        self.sent_stamps: set[tuple[int, int]] = set()

        self.image_pub = self.create_publisher(Image, COLOR_TOPIC, 10)
        self.results: list[Detection2DArray] = []
        self.create_subscription(Detection2DArray, DETECTIONS_TOPIC, self.results.append, 10)

    def publish_frame(self):
        header = Header(frame_id=CAMERA_FRAME, stamp=self.get_clock().now().to_msg())
        image = self._blank if self.send_blank else self._image
        image.header = header
        self.sent_stamps.add((header.stamp.sec, header.stamp.nanosec))
        self.image_pub.publish(image)


def spin_until(node, condition, timeout=TIMEOUT, period=0.1) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        rclpy.spin_once(node, timeout_sec=period)
    return condition()


class TestYoloNode(unittest.TestCase):

    def test_stop_sign_detection(self):
        img = load_test_image()
        rclpy.init()
        try:
            node = CameraFeeder(img)
            try:
                self.assertTrue(
                    spin_until(node, lambda: node.image_pub.get_subscription_count() > 0),
                    f'yolo_node never subscribed to {COLOR_TOPIC}')
                node.create_timer(1.0 / PUBLISH_HZ, node.publish_frame)
                self.assertTrue(
                    spin_until(node, lambda: len(node.results) >= MESSAGES_NEEDED),
                    f'got {len(node.results)}/{MESSAGES_NEEDED} messages on {DETECTIONS_TOPIC}')
                stop_sign_msgs = list(node.results)

                # A frame with nothing in it must still produce a (empty) message
                node.send_blank = True
                node.results.clear()
                self.assertTrue(
                    spin_until(node, lambda: any(not m.detections for m in node.results)),
                    f'no empty Detection2DArray on {DETECTIONS_TOPIC} for a blank frame')
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

        for msg in stop_sign_msgs:
            self.assertEqual(msg.header.frame_id, CAMERA_FRAME)
            self.assertIn((msg.header.stamp.sec, msg.header.stamp.nanosec), node.sent_stamps,
                          'message stamp is not a source image stamp')

        stop_signs = [
            (det, res) for msg in stop_sign_msgs for det in msg.detections for res in det.results
            if res.hypothesis.class_id == ObjectClass.SIGN_STOP
        ]
        per_msg = [sum(r.hypothesis.class_id == ObjectClass.SIGN_STOP
                       for d in m.detections for r in d.results) for m in stop_sign_msgs]
        self.assertTrue(all(per_msg),
                        f'stop sign missed in some frames, stop signs per message: {per_msg}')

        for det, res in stop_signs:
            box = det.bbox
            self.assertAlmostEqual(box.center.position.x, SIGN_CENTER_PX[0], delta=PIXEL_TOL)
            self.assertAlmostEqual(box.center.position.y, SIGN_CENTER_PX[1], delta=PIXEL_TOL)
            self.assertAlmostEqual(box.size_x, SIGN_SIZE_PX[0], delta=2 * PIXEL_TOL)
            self.assertAlmostEqual(box.size_y, SIGN_SIZE_PX[1], delta=2 * PIXEL_TOL)
            self.assertEqual(det.id, '', 'detections are not tracked yet, so id must be empty')
            self.assertGreater(res.hypothesis.score, 0.0)
            self.assertLessEqual(res.hypothesis.score, 1.0)
