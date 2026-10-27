"""End-to-end test for the running YOLO node. Run with: ./ap1d e2e perception [--gpu]

Plays the role of the RealSense driver: publishes a stop-sign photo, matching
camera info and a point cloud on the topics yolo_node subscribes to, then checks
that the node publishes the stop sign as an entity at the right depth.

The photo is COCO val2017 image 724. It's downloaded into a temporary folder,
checked against a fixed SHA-256, and deleted when the test ends. Pass --image to
use a local image instead (e.g. a frame from one of our own recordings).

The point cloud is a flat wall DEPTH metres in front of the camera, projected
with the same intrinsics as the image, so every detection box covers wall
points and the entity must come out at z = DEPTH.
"""
import argparse
import hashlib
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ap1_msgs.msg import EntityStateArray
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

# Topics yolo_node uses (ap1_perception/yolo_node.py)
INFO_TOPIC = "camera/camera/aligned_depth_to_color/camera_info"
COLOR_TOPIC = "camera/camera/color/image_raw"
PC_TOPIC = "/camera/camera/depth/color/points"
ENTITY_TOPIC = "ap1/perception/entities"

IMAGE_URL = "http://images.cocodataset.org/val2017/000000000724.jpg"
IMAGE_SHA256 = "5c0e559c75d3969c8e3e297b61f61063f78045c9d4802b526ba616361f3823fd"

FOCAL = 600.0      # px
DEPTH = 5.0        # m, distance of the synthetic wall
DEPTH_TOL = 0.05   # m
PUBLISH_HZ = 5.0
MESSAGES_NEEDED = 5


def download_image(url: str, sha256: str, folder: Path) -> Path:
    path = folder / Path(url).name
    with urllib.request.urlopen(url, timeout=30) as response:
        data = response.read()
    digest = hashlib.sha256(data).hexdigest()
    if digest != sha256:
        raise RuntimeError(f"{url} has SHA-256 {digest}, expected {sha256}")
    path.write_bytes(data)
    return path


class CameraFeeder(Node):
    """Publishes one image as a synchronized camera stream and records entities."""

    def __init__(self, img: np.ndarray):
        super().__init__("perception_e2e_feeder")
        h, w = img.shape[:2]
        cx, cy = w / 2, h / 2

        self._info = CameraInfo(width=w, height=h)
        self._info.k = [FOCAL, 0.0, cx, 0.0, FOCAL, cy, 0.0, 0.0, 1.0]
        self._image = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

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
        header = Header(frame_id="camera_color_optical_frame", stamp=self.get_clock().now().to_msg())
        self._info.header = header
        self._image.header = header
        self._info_pub.publish(self._info)
        self.image_pub.publish(self._image)
        self._pc_pub.publish(pc2.create_cloud_xyz32(header, self._points))


def spin_until(node, condition, timeout, period=0.1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        rclpy.spin_once(node, timeout_sec=period)
    return condition()


def run(img: np.ndarray, timeout: float) -> bool:
    rclpy.init()
    node = CameraFeeder(img)
    try:
        print(f"waiting up to {timeout:.0f}s for yolo_node to subscribe to {COLOR_TOPIC} ...")
        if not spin_until(node, lambda: node.image_pub.get_subscription_count() > 0, timeout):
            print("FAIL  yolo_node never subscribed. Is it running, on the same ROS_DOMAIN_ID?")
            return False

        node.create_timer(1.0 / PUBLISH_HZ, node.publish_frame)
        if not spin_until(node, lambda: len(node.results) >= MESSAGES_NEEDED, timeout):
            print(f"FAIL  got {len(node.results)}/{MESSAGES_NEEDED} messages on {ENTITY_TOPIC}")
            return False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    counts = [len(m.entities) for m in node.results]
    entities = [e for m in node.results for e in m.entities]
    print(f"entities per message: {counts}")
    ok = True
    if not all(counts):
        print("FAIL  stop sign missed in some frames")
        ok = False
    bad_depth = [e.z for e in entities if abs(e.z - DEPTH) > DEPTH_TOL]
    if bad_depth:
        print(f"FAIL  entity depth {bad_depth[0]:.3f} m, expected {DEPTH} m (wall)")
        ok = False
    if ok:
        e = entities[0]
        print(f"ok    stop sign at x={e.x:.2f} y={e.y:.2f} z={e.z:.2f} m in every frame")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", type=Path, help="local image with a stop sign; skips the download")
    parser.add_argument("--timeout", type=float, default=60.0, help="seconds to wait at each step")
    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tmp:
        path = args.image or download_image(IMAGE_URL, IMAGE_SHA256, Path(tmp))
        img = cv2.imread(str(path))
        if img is None:
            sys.exit(f"could not read image {path}")
        print(f"test image: {path} ({img.shape[1]}x{img.shape[0]})")
        ok = run(img, args.timeout)
    # The downloaded image is gone once the with-block exits, pass or fail
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
