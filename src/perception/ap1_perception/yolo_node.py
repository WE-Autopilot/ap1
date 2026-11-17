from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from ap1_msgs.msg import ObjectClass

from .yolo import YOLO
from .projection import box_geometry, cam_optical_to_base_link, check_box, pointcloud_to_pixel


DEPTH_TOPIC = "camera/camera/aligned_depth_to_color/image_raw"
COLOR_TOPIC = "camera/camera/color/image_raw"
PC_TOPIC = "/camera/camera/depth/color/points"
INFO_TOPIC = "camera/camera/aligned_depth_to_color/camera_info"

# Perception object contract (issue #66): consumed by Mapping & Localization
PUBLISH_TOPIC = "ap1/perception/objects"
OUTPUT_FRAME = "base_link"

YOLO_MODEL_FILE = "yolo26n.onnx"
MODEL_CLASSES = {11: ObjectClass.SIGN_STOP,}

ORIENTATION_VARIANCE = 1e6

# Fixed for now, not ROS parameters: the association node (#90) replaces this
# box-to-point code. Boxes with fewer cloud points have no reliable 3D position.
MIN_BOX_POINTS = 3
POSITION_STDDEV = 0.3  # m, 1 sigma, reported in each detection's covariance


def default_yolo_model() -> str:
    """Path to the YOLO model installed with the package: <ap1_perception share>/models."""
    return str(Path(get_package_share_directory("ap1_perception")) / "models" / YOLO_MODEL_FILE)


def pose_covariance() -> list[float]:
    """Row-major 6x6 covariance: POSITION_STDDEV on x, y, z, unknown orientation."""
    cov = np.zeros((6, 6))
    cov[[0, 1, 2], [0, 1, 2]] = POSITION_STDDEV ** 2
    cov[[3, 4, 5], [3, 4, 5]] = ORIENTATION_VARIANCE
    return cov.ravel().tolist()


COVARIANCE = pose_covariance()


class YoloNode(Node):
    def __init__(self):
        super().__init__('ap1_perception_yolo')

        self._bridge = CvBridge()
        self._K: np.ndarray | None = None

        # Startup-only parameters: changing these needs a node restart (not a rebuild).
        # model_path defaults to the model installed with the package, so
        # full_system.launch.py can start this node without parameters.
        model_path = self._declare('model_path', default_yolo_model(),
                                   'YOLO .onnx model; defaults to the one installed with the package', True)
        device = self._declare('device', 'auto', "'auto', 'cuda' or 'cpu'", True)
        sync_queue_size = self._declare('sync_queue_size', 5, 'Image/point cloud sync queue length', True)
        sync_slop = self._declare('sync_slop', 0.05, 'Max image/point cloud stamp gap (s)', True)

        # Tuning parameters: can also be changed live with `ros2 param set`.
        self._declare('conf_threshold', 0.25, 'Min detection confidence, 0-1')

        self._model = YOLO(model_path=model_path, classes=list(MODEL_CLASSES), device=device)
        self._apply_tuning()
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(lambda _params: self._apply_tuning())
        self.get_logger().info(
            f"YOLO loaded from {model_path}, running on {self._model.provider} "
            f"(conf={self._model.conf}, classes={list(MODEL_CLASSES.values())})."
        )

        self._pub = self.create_publisher(Detection3DArray, PUBLISH_TOPIC, 10)

        self.create_subscription(CameraInfo, INFO_TOPIC, self._camera_info_callback, 10)

        color_sub = Subscriber(self, Image, COLOR_TOPIC)
        pc_sub = Subscriber(self, PointCloud2, PC_TOPIC)
        self._sync = ApproximateTimeSynchronizer(
            [color_sub, pc_sub], queue_size=sync_queue_size, slop=sync_slop
        )
        self._sync.registerCallback(self._sync_callback)

    def _declare(self, name, default, description, read_only=False):
        """Declare a parameter and return its value."""
        descriptor = ParameterDescriptor(description=description, read_only=read_only)
        return self.declare_parameter(name, default, descriptor).value

    def _validate_parameters(self, params) -> SetParametersResult:
        """Reject out-of-range values before they reach the model."""
        for p in params:
            if p.name == 'conf_threshold' and not 0.0 <= p.value <= 1.0:
                return SetParametersResult(successful=False, reason='conf_threshold must be in [0, 1]')
        return SetParametersResult(successful=True)

    def _apply_tuning(self) -> None:
        """Copy the current tuning parameters onto the model."""
        self._model.conf = self.get_parameter('conf_threshold').value

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._K is not None:
            return
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._model.K = self._K
        self.get_logger().info("Camera intrinsics received - YOLO node ready.")

    def _sync_callback(self, color_msg: Image, pc_msg: PointCloud2) -> None:
        if self._K is None:
            return

        # Always publish, stamped with the source image: an empty array means
        # nothing was detected in this frame.
        out = Detection3DArray()
        out.header.stamp = color_msg.header.stamp
        out.header.frame_id = OUTPUT_FRAME

        # YOLO expects bgr encoding not rgb
        color = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

        # read_points_numpy returns a plain (N, 3) float array directly
        xyz = pc2.read_points_numpy(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float32)

        if xyz.shape[0] > 0:
            boxes, scores, classes = self._model.forward(color)
            pixel_coords = pointcloud_to_pixel(self._K, xyz)  # (N, 2)
            for box, score, cls in zip(boxes, scores, classes):
                box_xyz = xyz[check_box(pixel_coords, box)]
                if len(box_xyz) < MIN_BOX_POINTS:
                    continue  # too little depth in this box for a reliable 3D position
                out.detections.append(self._detection(box, box_xyz, score, cls, out.header))

        self._pub.publish(out)

    def _detection(self, box, box_xyz, score, cls, header) -> Detection3D:
        """One Detection3D in base_link from a pixel box and the cloud points inside it."""
        center_cam, size_cam = box_geometry(self._K, box, box_xyz)
        center = cam_optical_to_base_link(center_cam[None])[0]
        # Optical (width, height, depth) -> base_link (x forward, y left, z up)
        size = (size_cam[2], size_cam[0], size_cam[1])

        det = Detection3D()
        det.header = header
        det.id = ''  # not tracked yet
        det.bbox.center.position.x = float(center[0])
        det.bbox.center.position.y = float(center[1])
        det.bbox.center.position.z = float(center[2])
        det.bbox.center.orientation.w = 1.0
        det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = (float(v) for v in size)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = MODEL_CLASSES[int(cls)]
        hyp.hypothesis.score = float(score)
        hyp.pose.pose = det.bbox.center
        hyp.pose.covariance = COVARIANCE
        det.results.append(hyp)
        return det


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C, or launch stopping the node: exit without a traceback
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
