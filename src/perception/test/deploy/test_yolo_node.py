"""End-to-end test of yolo_node over ROS.

Launches yolo_node and plays the role of the RealSense driver: publishes a
stop-sign photo, matching camera info and a point cloud on the topics the node
subscribes to, then checks that it publishes the stop sign on
ap1/perception/objects as a Detection3DArray per the perception object contract
(issue #66): class ObjectClass.SIGN_STOP, base_link frame, the image's stamp.
It then feeds a blank frame and checks the node still publishes, with an
empty array.

The photo is COCO val2017 image 724. It's downloaded into a temporary folder,
checked against a fixed SHA-256, and deleted as soon as it's loaded. A frame
from our own campus dataset will replace it.

The point cloud is a flat wall DEPTH metres in front of the camera, projected
with the same intrinsics as the image, so every detection box covers wall
points and the stop sign must come out DEPTH metres ahead (base_link x), at
the sideways and height position its pixels map to on the wall.
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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray

# Topics yolo_node uses (ap1_perception/yolo_node.py)
INFO_TOPIC = 'camera/camera/aligned_depth_to_color/camera_info'
COLOR_TOPIC = 'camera/camera/color/image_raw'
PC_TOPIC = '/camera/camera/depth/color/points'
OBJECTS_TOPIC = 'ap1/perception/objects'

IMAGE_URL = 'http://images.cocodataset.org/val2017/000000000724.jpg'
IMAGE_SHA256 = '5c0e559c75d3969c8e3e297b61f61063f78045c9d4802b526ba616361f3823fd'

FOCAL = 600.0      # px
DEPTH = 5.0        # m, distance of the synthetic wall
DEPTH_TOL = 0.05   # m
# The stop sign's pixel box in the photo, found from its red pixels, not from
# YOLO: (124, 76) to (251, 221). Its center sits on the image's vertical
# center line and 101.5 px above the horizontal one, so on the wall it's
# straight ahead (y = 0) and 101.5 * DEPTH / FOCAL = 0.85 m up (z), in base_link.
SIGN_CENTER_PX = (187.5, 148.5)  # u, v
POSITION_TOL = 0.1  # m, about 12 px at DEPTH: a box-edge's worth of slack
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
    """Publishes one image as a synchronized camera stream and records detections."""

    def __init__(self, img: np.ndarray):
        super().__init__('perception_e2e_feeder')
        h, w = img.shape[:2]
        cx, cy = w / 2, h / 2

        self._info = CameraInfo(width=w, height=h)
        self._info.k = [FOCAL, 0.0, cx, 0.0, FOCAL, cy, 0.0, 0.0, 1.0]
        bridge = CvBridge()
        self._image = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self._blank = bridge.cv2_to_imgmsg(np.zeros_like(img), encoding='bgr8')
        self.send_blank = False
        self.sent_stamps: set[tuple[int, int]] = set()

        # Wall points every 4 px, back-projected to DEPTH
        us, vs = np.meshgrid(np.arange(0, w, 4), np.arange(0, h, 4))
        z = np.full(us.size, DEPTH, dtype=np.float32)
        self._points = np.stack(
            [(us.ravel() - cx) * z / FOCAL, (vs.ravel() - cy) * z / FOCAL, z], axis=1
        ).astype(np.float32)

        self._info_pub = self.create_publisher(CameraInfo, INFO_TOPIC, 10)
        self.image_pub = self.create_publisher(Image, COLOR_TOPIC, 10)
        self._pc_pub = self.create_publisher(PointCloud2, PC_TOPIC, 10)
        self.results: list[Detection3DArray] = []
        self.create_subscription(Detection3DArray, OBJECTS_TOPIC, self.results.append, 10)

    def publish_frame(self):
        # Same stamp on all three so the node's ApproximateTimeSynchronizer pairs them
        header = Header(frame_id='camera_color_optical_frame', stamp=self.get_clock().now().to_msg())
        image = self._blank if self.send_blank else self._image
        self._info.header = header
        image.header = header
        self.sent_stamps.add((header.stamp.sec, header.stamp.nanosec))
        self._info_pub.publish(self._info)
        self.image_pub.publish(image)
        self._pc_pub.publish(pc2.create_cloud_xyz32(header, self._points))


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
                    f'got {len(node.results)}/{MESSAGES_NEEDED} messages on {OBJECTS_TOPIC}')
                stop_sign_msgs = list(node.results)

                # A frame with nothing in it must still produce a (empty) message
                node.send_blank = True
                node.results.clear()
                self.assertTrue(
                    spin_until(node, lambda: any(not m.detections for m in node.results)),
                    f'no empty Detection3DArray on {OBJECTS_TOPIC} for a blank frame')
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

        for msg in stop_sign_msgs:
            self.assertEqual(msg.header.frame_id, 'base_link')
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

        # Optical (right, down) offsets from the image center -> base_link (left, up)
        h, w = img.shape[:2]
        expected_y = -(SIGN_CENTER_PX[0] - w / 2) * DEPTH / FOCAL
        expected_z = -(SIGN_CENTER_PX[1] - h / 2) * DEPTH / FOCAL
        for det, res in stop_signs:
            pos = det.bbox.center.position
            # The wall is DEPTH ahead of the camera: base_link +X is forward
            self.assertAlmostEqual(pos.x, DEPTH, delta=DEPTH_TOL,
                                   msg=f'stop sign at x={pos.x}, expected {DEPTH} m')
            self.assertAlmostEqual(pos.y, expected_y, delta=POSITION_TOL,
                                   msg=f'stop sign at y={pos.y}, expected {expected_y:.3f} m')
            self.assertAlmostEqual(pos.z, expected_z, delta=POSITION_TOL,
                                   msg=f'stop sign at z={pos.z}, expected {expected_z:.3f} m')
            q = det.bbox.center.orientation
            self.assertEqual((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0),
                             'orientation must be identity')
            self.assertEqual(det.id, '', 'detections are not tracked yet, so id must be empty')
            self.assertGreater(res.hypothesis.score, 0.0)
            self.assertLessEqual(res.hypothesis.score, 1.0)
            self.assertTrue(any(res.pose.covariance), 'covariance must never be all zeros')
