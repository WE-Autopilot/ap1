"""End-to-end test of road_seg_node over ROS.

Launches road_seg_node and plays the role of the RealSense driver: publishes a
street photo and matching camera info on the topics the node subscribes to,
then checks the class map, the scaled camera info and the class names it publishes.

The photo is Cityscapes aachen_000006_000019 (a straight road ahead), from the
cityscapes8 sample that ultralytics hosts. The zip is downloaded into a temporary
folder, checked against a fixed SHA-256, and deleted as soon as the photo is loaded.
It's cropped to 16:9 and resized to 1920x1080, the size of the AP1 camera.
"""
import hashlib
import tempfile
import time
import unittest
import urllib.request
import zipfile
from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from perception_common import ROAD_SEG_MODEL_PATH, pick_device
from rclpy.node import Node as RclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from vision_msgs.msg import LabelInfo

# Topics road_seg_node uses (ap1_perception/road_seg_node.py, config/road_seg.yaml)
COLOR_TOPIC = 'camera/camera/color/image_raw'
INFO_TOPIC = 'camera/camera/color/camera_info'
CLASS_MAP_TOPIC = 'ap1/perception/road_seg/class_map'
SEG_INFO_TOPIC = 'ap1/perception/road_seg/camera_info'
LABEL_INFO_TOPIC = 'ap1/perception/road_seg/label_info'

ZIP_URL = 'https://github.com/ultralytics/assets/releases/download/v0.0.0/cityscapes8.zip'
ZIP_SHA256 = 'ece03d0c49df70cfa5463919f1e4e7420c545dd91d4af581e144bb422c8d3bb5'
IMAGE_IN_ZIP = 'cityscapes8/images/val/aachen_000006_000019.png'

WIDTH, HEIGHT = 1920, 1080      # camera resolution
MAP_W, MAP_H = 768, 432         # class map resolution (the model input without padding)
FOCAL = 1000.0                  # px
ROAD = 0                        # Cityscapes class ID
MIN_ROAD_FRACTION = 0.8         # of the patch straight ahead of the car
PUBLISH_HZ = 5.0
MESSAGES_NEEDED = 5
TIMEOUT = 60.0                  # s, for each wait


@pytest.mark.launch_test
def generate_test_description():
    road_seg_node = Node(
        package='ap1_perception',
        executable='road_seg_node',
        output='screen',
        parameters=[{'model_path': str(ROAD_SEG_MODEL_PATH), 'device': pick_device(),
                     'image_topic': COLOR_TOPIC, 'camera_info_topic': INFO_TOPIC}],
    )
    return LaunchDescription([road_seg_node, ReadyToTest()])


def load_test_image() -> np.ndarray:
    with tempfile.TemporaryDirectory() as tmp:
        with urllib.request.urlopen(ZIP_URL, timeout=60) as response:
            data = response.read()
        digest = hashlib.sha256(data).hexdigest()
        if digest != ZIP_SHA256:
            raise RuntimeError(f'{ZIP_URL} has SHA-256 {digest}, expected {ZIP_SHA256}')
        path = Path(tmp) / 'cityscapes8.zip'
        path.write_bytes(data)
        with zipfile.ZipFile(path) as z:
            png = np.frombuffer(z.read(IMAGE_IN_ZIP), dtype=np.uint8)
    img = cv2.imdecode(png, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f'could not decode {IMAGE_IN_ZIP}')
    # 2048x1024 (2:1) -> crop the sides to 16:9 -> 1920x1080
    h, w = img.shape[:2]
    crop_w = h * 16 // 9
    x0 = (w - crop_w) // 2
    return cv2.resize(img[:, x0:x0 + crop_w], (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)


class CameraFeeder(RclpyNode):
    """Publishes one image as a camera stream and records the node's outputs."""

    def __init__(self, img: np.ndarray):
        super().__init__('road_seg_e2e_feeder')
        self._info = CameraInfo(width=WIDTH, height=HEIGHT, distortion_model='plumb_bob', d=[0.0] * 5)
        self._info.k = [FOCAL, 0.0, WIDTH / 2, 0.0, FOCAL, HEIGHT / 2, 0.0, 0.0, 1.0]
        self._info.p = [FOCAL, 0.0, WIDTH / 2, 0.0, 0.0, FOCAL, HEIGHT / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._image = CvBridge().cv2_to_imgmsg(img, encoding='bgr8')
        self.sent_stamps = set()

        self._info_pub = self.create_publisher(CameraInfo, INFO_TOPIC, 10)
        self.image_pub = self.create_publisher(Image, COLOR_TOPIC, 10)

        self.class_maps: list[Image] = []
        self.seg_infos: list[CameraInfo] = []
        self.label_infos: list[LabelInfo] = []
        self.create_subscription(Image, CLASS_MAP_TOPIC, self.class_maps.append, 10)
        self.create_subscription(CameraInfo, SEG_INFO_TOPIC, self.seg_infos.append, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LabelInfo, LABEL_INFO_TOPIC, self.label_infos.append, latched)

    def publish_frame(self):
        header = Header(frame_id='camera_color_optical_frame', stamp=self.get_clock().now().to_msg())
        self.sent_stamps.add((header.stamp.sec, header.stamp.nanosec))
        self._info.header = header
        self._image.header = header
        self._info_pub.publish(self._info)
        self.image_pub.publish(self._image)


def spin_until(node, condition, timeout=TIMEOUT, period=0.1) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        rclpy.spin_once(node, timeout_sec=period)
    return condition()


class TestRoadSegNode(unittest.TestCase):

    def test_road_ahead_is_segmented(self):
        img = load_test_image()
        rclpy.init()
        try:
            node = CameraFeeder(img)
            try:
                self.assertTrue(
                    spin_until(node, lambda: node.image_pub.get_subscription_count() > 0),
                    f'road_seg_node never subscribed to {COLOR_TOPIC}')
                node.create_timer(1.0 / PUBLISH_HZ, node.publish_frame)
                self.assertTrue(
                    spin_until(node, lambda: len(node.class_maps) >= MESSAGES_NEEDED and node.label_infos),
                    f'got {len(node.class_maps)}/{MESSAGES_NEEDED} messages on {CLASS_MAP_TOPIC}, '
                    f'{len(node.label_infos)} on {LABEL_INFO_TOPIC}')
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

        for msg in node.class_maps:
            self.assertEqual((msg.width, msg.height, msg.encoding), (MAP_W, MAP_H, 'mono8'))
            # The class map keeps the camera image's header: capture stamp, camera frame
            self.assertIn((msg.header.stamp.sec, msg.header.stamp.nanosec), node.sent_stamps)
            self.assertEqual(msg.header.frame_id, 'camera_color_optical_frame')

        # The patch straight ahead of the car (bottom middle of the image) is road
        class_map = np.frombuffer(bytes(node.class_maps[-1].data), dtype=np.uint8).reshape(MAP_H, MAP_W)
        ahead = class_map[int(0.75 * MAP_H):int(0.95 * MAP_H), int(0.3 * MAP_W):int(0.7 * MAP_W)]
        road_fraction = float(np.mean(ahead == ROAD))
        self.assertGreater(road_fraction, MIN_ROAD_FRACTION, f'only {road_fraction:.0%} of the patch ahead is road')

        # Camera info scaled to the class map: 1920 -> 768 is a factor of 0.4
        self.assertTrue(node.seg_infos, f'nothing on {SEG_INFO_TOPIC}')
        info = node.seg_infos[-1]
        self.assertEqual((info.width, info.height), (MAP_W, MAP_H))
        np.testing.assert_allclose([info.k[0], info.k[2], info.k[4], info.k[5]],
                                   [FOCAL * 0.4, MAP_W / 2, FOCAL * 0.4, MAP_H / 2])

        # Class names: the 19 Cityscapes classes, road first
        names = {c.class_id: c.class_name for c in node.label_infos[-1].class_map}
        self.assertEqual(len(names), 19)
        self.assertEqual(names[ROAD], 'road')
