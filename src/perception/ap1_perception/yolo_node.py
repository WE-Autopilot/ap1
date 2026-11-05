import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from geometry_msgs.msg import PoseArray, PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from ap1_msgs.msg import EntityStateArray, EntityState

from .yolo import YOLO
from .projection import pointcloud_to_pixel, check_box
from .ransac import GroundRANSAC
from .ransac._plane import Plane


DEPTH_TOPIC = "camera/camera/aligned_depth_to_color/image_raw"
COLOR_TOPIC = "camera/camera/color/image_raw"
PC_TOPIC = "/camera/camera/depth/color/points"
INFO_TOPIC = "camera/camera/aligned_depth_to_color/camera_info"

#PUBLISH_TOPIC = "perception/entities"
PUBLISH_TOPIC = "ap1/perception/entities"


def normal_to_quaternion(normal: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a unit normal vector to a quaternion with the z-axis aligned to the normal.

    Returns (qx, qy, qz, qw).
    """
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)

    z = np.array([0.0, 0.0, 1.0])
    dot = np.dot(z, normal)

    if dot > 1.0 - 1e-6:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1e-6:
        return (1.0, 0.0, 0.0, 0.0)

    axis = np.cross(z, normal)
    axis /= np.linalg.norm(axis)
    angle = np.arccos(np.clip(dot, -1.0, 1.0))

    s = np.sin(angle / 2.0)
    return (axis[0] * s, axis[1] * s, axis[2] * s, np.cos(angle / 2.0))


def ransac_plane_normal(points: np.ndarray, max_retry: int = 20, p_thresh: float = 1.0,
                        r_thresh: float = 0.7, l_thresh: float = 0.01) -> np.ndarray:
    """Fit a plane to 3D points using GroundRANSAC and return the unit normal.

    The normal is oriented to face the camera (negative z in camera frame).
    """
    centroid = np.median(points, axis=0)
    # Initial estimate: plane at the centroid with normal pointing toward the camera.
    initial = Plane([0.0, 0.0, -1.0], centroid)
    ransac = GroundRANSAC(estimate=initial, max_retry=max_retry, p_thresh=p_thresh,
                          r_thresh=r_thresh, l_thresh=l_thresh)
    plane = ransac(points)

    normal = plane.normal
    if normal[2] > 0:
        normal = -normal
    return normal


class YoloNode(Node):
    def __init__(self):
        super().__init__('ap1_perception_yolo')

        self._bridge = CvBridge()
        self._K: np.ndarray | None = None

        # Startup-only parameters: changing these needs a node restart (not a rebuild).
        # Empty model_path falls back to the model bundled in the package.
        model_path = self._declare('model_path', '', 'YOLO .onnx model; empty uses the bundled model', True)
        device = self._declare('device', 'auto', "'auto', 'cuda' or 'cpu'", True)
        sync_queue_size = self._declare('sync_queue_size', 5, 'Image/point cloud sync queue length', True)
        sync_slop = self._declare('sync_slop', 0.05, 'Max image/point cloud stamp gap (s)', True)

        # Tuning parameters: can also be changed live with `ros2 param set`.
        self._declare('conf_threshold', 0.25, 'Min detection confidence, 0-1')
        self._declare('iou_threshold', 0.7, 'NMS IoU threshold, 0-1 (ignored by end-to-end exports)')
        self._declare('max_detections', 300, 'Max detections per frame')
        self._declare('classes', [11], 'COCO class ids to keep (11 = stop sign)')
        self._declare('min_box_points', 3, 'Min point cloud points inside a box to publish it')
        self._declare('ransac.max_retry', 20, 'RANSAC iterations per box')
        self._declare('ransac.p_thresh', 1.0, 'RANSAC prune distance (m)')
        self._declare('ransac.r_thresh', 0.7, 'RANSAC inlier ratio to accept a plane')
        self._declare('ransac.l_thresh', 0.01, 'RANSAC inlier distance (m)')

        self._model = YOLO(model_path=model_path or None, device=device)
        self._apply_tuning()
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(lambda _params: self._apply_tuning())
        self.get_logger().info(
            f"YOLO loaded, running on {self._model.provider} "
            f"(conf={self._model.conf}, classes={self._model.classes.tolist()})."
        )

        self._pub = self.create_publisher(EntityStateArray, PUBLISH_TOPIC, 10)

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
            if p.name in ('conf_threshold', 'iou_threshold') and not 0.0 <= p.value <= 1.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be in [0, 1]')
            if p.name in ('max_detections', 'min_box_points', 'ransac.max_retry') and p.value < 1:
                return SetParametersResult(successful=False, reason=f'{p.name} must be >= 1')
            if p.name == 'classes' and len(p.value) == 0:
                return SetParametersResult(successful=False, reason='classes must not be empty')
        return SetParametersResult(successful=True)

    def _apply_tuning(self) -> None:
        """Copy the current tuning parameters onto the model and RANSAC settings."""
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self._model.conf = get('conf_threshold')
        self._model.iou = get('iou_threshold')
        self._model.max_det = get('max_detections')
        self._model.classes = np.asarray(get('classes'))
        self._min_box_points = get('min_box_points')
        self._ransac_args = {
            'max_retry': get('ransac.max_retry'),
            'p_thresh': get('ransac.p_thresh'),
            'r_thresh': get('ransac.r_thresh'),
            'l_thresh': get('ransac.l_thresh'),
        }

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._K is not None:
            return
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._model.K = self._K
        self.get_logger().info("Camera intrinsics received - YOLO node ready.")

    def _sync_callback(self, color_msg: Image, pc_msg: PointCloud2) -> None:
        if self._K is None:
            return

        # YOLO expects bgr encoding not rgb
        color = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

        # read_points_numpy returns a plain (N, 3) float array directly
        xyz = pc2.read_points_numpy(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float32)

        if xyz.shape[0] == 0:
            return

        entity_array = EntityStateArray()
        entity_array.header = color_msg.header

        boxes, _ = self._model.forward(color)

        if boxes.shape[0] == 0:
            self._pub.publish(entity_array)
            return

        pixel_coords = pointcloud_to_pixel(self._K, xyz)  # (N, 2)

        # aly edit
        # changing this PoseArray to EntityStateArray
        # == ENTITYSTATEARRAY START
        for box in boxes:
            mask = check_box(pixel_coords, box)
            box_xyz = xyz[mask]
            if len(box_xyz) < self._min_box_points:
                continue
            centroid = np.median(box_xyz, axis=0)
            normal = ransac_plane_normal(box_xyz, **self._ransac_args)
            qx, qy, qz, qw = normal_to_quaternion(normal)
            # convert quaternion to yaw angle (gamma)
            gamma = 2.0 * np.arctan2(qz, qw)
            entity = EntityState()
            entity.x     = float(centroid[0])
            entity.y     = float(centroid[1])
            entity.z     = float(centroid[2])
            entity.gamma = float(gamma)
            entity_array.entities.append(entity)
        self._pub.publish(entity_array)
        return
        # == ENTITYSTATEARRAY END
        # old code:
        pose_array = PoseArray()
        pose_array.header = color_msg.header

        for box in boxes:
            mask = check_box(pixel_coords, box)
            box_xyz = xyz[mask]

            if len(box_xyz) < 3:
                continue

            centroid = np.median(box_xyz, axis=0)
            normal = ransac_plane_normal(box_xyz)
            qx, qy, qz, qw = normal_to_quaternion(normal)

            pose = PoseStamped().pose
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = float(centroid[2])
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)

            pose_array.poses.append(pose)

        self._pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
