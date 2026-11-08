"""End-to-end test of yolo_node over ROS.

Launches yolo_node and plays the role of the RealSense driver: publishes a
stop-sign photo, matching camera info and a point cloud on the topics the node
subscribes to, then checks that it publishes the stop sign as an entity at the
right depth.

The photo is COCO val2017 image 724. It's downloaded into a temporary folder,
checked against a fixed SHA-256, and deleted as soon as it's loaded. A frame
from our own campus dataset will replace it.

The point cloud is a flat wall DEPTH metres in front of the camera, projected
with the same intrinsics as the image, so every detection box covers wall
points and the entity must come out at z = DEPTH.
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
from ap1_msgs.msg import EntityStateArray
from cv_bridge import CvBridge
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from perception_common import MODEL_PATH, pick_device
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

# Topics yolo_node uses (ap1_perception/yolo_node.py)
INFO_TOPIC = 'camera/camera/aligned_depth_to_color/camera_info'
COLOR_TOPIC = 'camera/camera/color/image_raw'
PC_TOPIC = '/camera/camera/depth/color/points'
ENTITY_TOPIC = 'ap1/perception/entities'

IMAGE_URL = 'http://images.cocodataset.org/val2017/000000000724.jpg'
IMAGE_SHA256 = '5c0e559c75d3969c8e3e297b61f61063f78045c9d4802b526ba616361f3823fd'

FOCAL = 600.0      # px
DEPTH = 5.0        # m, distance of the synthetic wall
DEPTH_TOL = 0.05   # m
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
    """Publishes one image as a synchronized camera stream and records entities."""

    def __init__(self, img: np.ndarray):
        super().__init__('perception_e2e_feeder')
        h, w = img.shape[:2]
        cx, cy = w / 2, h / 2

        self._info = CameraInfo(width=w, height=h)
        self._info.k = [FOCAL, 0.0, cx, 0.0, FOCAL, cy, 0.0, 0.0, 1.0]
        self._image = CvBridge().cv2_to_imgmsg(img, encoding='bgr8')

        # Wall points every 4 px, back-projected to DEPTH
        us, vs = np.meshgrid(np.arange(0, w, 4), np.arange(0, h, 4))
        z = np.full(us.size, DEPTH, dtype=np.float32)
        self._points = np.stack(
            [(us.ravel() - cx) * z / FOCAL, (vs.ravel() - cy) * z / FOCAL, z], axis=1
        ).astype(np.float32)

        self._info_pub = self.create_publisher(CameraInfo, INFO_TOPIC, 10)
        self.image_pub = self.create_publisher(Image, COLOR_TOPIC, 10)
        self._pc_pub = self.create_publisher(PointCloud2, PC_TOPIC, 10)
        self.results: list[EntityStateArray] = []
        self.create_subscription(EntityStateArray, ENTITY_TOPIC, self.results.append, 10)

    def publish_frame(self):
        # Same stamp on all three so the node's ApproximateTimeSynchronizer pairs them
        header = Header(frame_id='camera_color_optical_frame', stamp=self.get_clock().now().to_msg())
        self._info.header = header
        self._image.header = header
        self._info_pub.publish(self._info)
        self.image_pub.publish(self._image)
        self._pc_pub.publish(pc2.create_cloud_xyz32(header, self._points))


def spin_until(node, condition, timeout=TIMEOUT, period=0.1) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        rclpy.spin_once(node, timeout_sec=period)
    return condition()


class TestYoloNode(unittest.TestCase):

    def test_stop_sign_at_wall_depth(self):
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
                    f'got {len(node.results)}/{MESSAGES_NEEDED} messages on {ENTITY_TOPIC}')
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

        counts = [len(m.entities) for m in node.results]
        self.assertTrue(all(counts), f'stop sign missed in some frames, entities per message: {counts}')
        depths = [e.z for m in node.results for e in m.entities]
        for z in depths:
            self.assertAlmostEqual(z, DEPTH, delta=DEPTH_TOL, msg=f'entity depths {depths}, expected {DEPTH} m')
