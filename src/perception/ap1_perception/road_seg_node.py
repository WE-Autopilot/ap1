"""Road segmentation ROS 2 node (model = YOLO26n-sem).

Segments each camera frame as it arrives, at most rate_hz frames per second (frames
that come sooner are skipped), and publishes:
- CLASS_MAP_TOPIC: sensor_msgs/Image, mono8, one class ID per pixel, at the model's
  resolution (768x432 for a 1920x1080 camera). The header is the camera image's:
  stamp = capture time, frame_id = camera optical frame.
- SEG_INFO_TOPIC: the camera's CameraInfo scaled to the class map's resolution, so
  lidar points can be projected straight onto the class map.
- LABEL_INFO_TOPIC: vision_msgs/LabelInfo mapping class IDs to names, latched.

Once a second it logs where the time goes and where frames go missing:
- received / skipped (by the rate cap) / processed frames
- missed: frames that never reached the node, counted from gaps in the camera
  timestamps (lost in transport, or replaced in the queue while the node was busy)
- time per processed frame for each step: unpack, convert, model, publish
- busy: share of the second the node spent working. Near 100% means the node itself
  is the bottleneck; low busy with missed frames means they're lost before the node.
"""
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import LabelInfo, VisionClass

from .road_seg import RoadSeg

CLASS_MAP_TOPIC = "ap1/perception/road_seg/class_map"
SEG_INFO_TOPIC = "ap1/perception/road_seg/camera_info"
LABEL_INFO_TOPIC = "ap1/perception/road_seg/label_info"

# The only place the road segmentation model's file name appears. Model files live
# in src/perception/models/ (Git LFS) and are installed to the package's share folder.
ROAD_SEG_MODEL_FILE = "yolo26n-sem.onnx"


def default_road_seg_model() -> str:
    """Path to the road segmentation model installed with the package: <ap1_perception share>/models."""
    return str(Path(get_package_share_directory("ap1_perception")) / "models" / ROAD_SEG_MODEL_FILE)


def scale_camera_info(info: CameraInfo, width: int, height: int) -> CameraInfo:
    """Rescale CameraInfo to an image resized to width x height.

    Intrinsics (K, P) scale linearly with resolution; distortion and rectification don't change.
    """
    sx = width / info.width
    sy = height / info.height

    out = CameraInfo()
    out.header = info.header
    out.width, out.height = width, height
    out.distortion_model = info.distortion_model
    out.d = list(info.d)
    out.r = list(info.r)

    k = np.array(info.k, dtype=np.float64).reshape(3, 3)
    k[0] *= sx
    k[1] *= sy
    out.k = k.flatten().tolist()

    p = np.array(info.p, dtype=np.float64).reshape(3, 4)
    p[0] *= sx
    p[1] *= sy
    out.p = p.flatten().tolist()
    return out


class RoadSegNode(Node):
    def __init__(self):
        super().__init__('ap1_perception_road_seg')

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._last_processed: float | None = None  # time.monotonic() of the last segmented frame

        # Camera timestamps, to count frames that never reached the node
        self._last_stamp: float | None = None
        self._frame_period: float | None = None  # smallest gap seen between camera frames

        # Counters for the once-a-second log line (reset after each line)
        self._received = 0
        self._skipped = 0
        self._processed = 0
        self._missed = 0
        self._step_time = defaultdict(float)  # seconds per step, summed over processed frames
        self._busy_time = 0.0                 # seconds spent inside the image callback
        self._stats_start = time.perf_counter()

        # Startup-only parameters: changing these needs a node restart (not a rebuild).
        # model_path defaults to the model installed with the package.
        model_path = self._declare('model_path', default_road_seg_model(),
                                   'Road segmentation .onnx model; defaults to the one installed with the package', True)
        device = self._declare('device', 'auto', "'auto', 'tensorrt', 'cuda' or 'cpu'", True)
        image_topic = self._declare('image_topic', 'camera/camera/color/image_raw', 'Camera image to segment', True)
        info_topic = self._declare('camera_info_topic', 'camera/camera/color/camera_info',
                                   'CameraInfo of image_topic', True)
        rate_hz = self._declare('rate_hz', 10.0, 'Max segmentation rate; frames arriving sooner are skipped', True)
        threads = self._declare('threads', 4, 'CPU threads onnxruntime uses for inference', True)
        self._min_period = 1.0 / rate_hz

        self._model = RoadSeg(model_path=model_path, device=device, threads=threads)
        self.get_logger().info(
            f"Road segmentation loaded from {model_path}, running on {self._model.provider} "
            f"({len(self._model.names)} classes), listening on {image_topic}."
        )

        self._class_map_pub = self.create_publisher(Image, CLASS_MAP_TOPIC, 10)
        self._info_pub = self.create_publisher(CameraInfo, SEG_INFO_TOPIC, 10)

        # LabelInfo only changes with the model, so latch it for late subscribers
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._label_pub = self.create_publisher(LabelInfo, LABEL_INFO_TOPIC, latched)
        self._label_pub.publish(self._label_info())

        # Depth 1: while a frame is being segmented, keep only the newest waiting frame,
        # never a backlog of stale ones. Reliable, not best effort: an image is hundreds of
        # packets, and best effort drops the whole image when any one of them is lost
        # (on the KITTI bag in Docker that lost over half the frames). A best-effort
        # publisher still works with this: delivery then falls back to best effort.
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        # raw=True: the callback gets the serialized bytes and unpacks them itself, so the
        # unpacking (slow in Python for big images) shows up in the step timings
        self.create_subscription(Image, image_topic, self._image_callback, image_qos, raw=True)
        self.create_subscription(CameraInfo, info_topic, self._camera_info_callback, 10)

        self.create_timer(1.0, self._log_stats)

    def _declare(self, name, default, description, read_only=False):
        """Declare a parameter and return its value."""
        descriptor = ParameterDescriptor(description=description, read_only=read_only)
        return self.declare_parameter(name, default, descriptor).value

    def _label_info(self) -> LabelInfo:
        msg = LabelInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.class_map = [VisionClass(class_id=int(i), class_name=name) for i, name in self._model.names.items()]
        return msg

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _image_callback(self, data: bytes) -> None:
        start = time.perf_counter()
        msg = deserialize_message(data, Image)
        unpack = time.perf_counter() - start
        self._received += 1
        self._count_missed(msg)

        now = time.monotonic()
        # Cap at rate_hz: skip frames that come too soon after the last segmented one.
        # The 0.9 is slack for jitter, so a 10 Hz camera still gets every frame while a
        # 20 Hz camera (the AP1 camera) gets every other one.
        if self._last_processed is not None and now - self._last_processed < 0.9 * self._min_period:
            self._skipped += 1
        else:
            self._last_processed = now
            self._step_time['unpack'] += unpack
            self._process(msg)
        self._busy_time += time.perf_counter() - start

    def _count_missed(self, msg: Image) -> None:
        """Count camera frames that never reached the node, from gaps in their timestamps."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_stamp is not None:
            gap = stamp - self._last_stamp
            if gap > 0:  # a bag played with --loop jumps back at each restart: ignore that
                self._frame_period = gap if self._frame_period is None else min(self._frame_period, gap)
                self._missed += max(round(gap / self._frame_period) - 1, 0)
        self._last_stamp = stamp

    def _process(self, image_msg: Image) -> None:
        t0 = time.perf_counter()
        rgb = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
        t1 = time.perf_counter()
        class_map = self._model(rgb)  # preprocessing + inference
        t2 = time.perf_counter()
        self._processed += 1

        # Built by hand: the pip cv_bridge (native uv setup) calls ndarray.tostring()
        # in cv2_to_imgmsg, which numpy 2 removed.
        out = Image()
        out.header = image_msg.header
        out.height, out.width = class_map.shape
        out.encoding = 'mono8'
        out.step = out.width
        out.data = class_map.tobytes()
        self._class_map_pub.publish(out)

        if self._camera_info is not None:
            info = scale_camera_info(self._camera_info, out.width, out.height)
            info.header = image_msg.header
            self._info_pub.publish(info)
        t3 = time.perf_counter()

        self._step_time['convert'] += t1 - t0
        self._step_time['model'] += t2 - t1
        self._step_time['publish'] += t3 - t2

    def _log_stats(self) -> None:
        """Log the last second's frame counts, step times and busy share, then reset them."""
        elapsed = time.perf_counter() - self._stats_start
        if self._received:
            n = max(self._processed, 1)
            steps = {name: 1000 * self._step_time[name] / n for name in ('unpack', 'convert', 'model', 'publish')}
            total = sum(steps.values())
            self.get_logger().info(
                f"last {elapsed:.1f} s: received {self._received} (missed {self._missed}, skipped {self._skipped}), "
                f"processed {self._processed} | "
                + ", ".join(f"{name} {ms:.0f}" for name, ms in steps.items())
                + f" = {total:.0f} ms/frame | busy {100 * self._busy_time / elapsed:.0f}%")
        self._received = self._skipped = self._processed = self._missed = 0
        self._step_time.clear()
        self._busy_time = 0.0
        self._stats_start = time.perf_counter()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoadSegNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C, or launch stopping the node: exit without a traceback
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
