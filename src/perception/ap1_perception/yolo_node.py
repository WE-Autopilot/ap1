from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ap1_msgs.msg import ObjectClass

from .yolo import YOLO


COLOR_TOPIC = "camera/camera/color/image_raw"

# 2D boxes in image pixels (#116); the association node (#90) turns them into
# 3D objects on ap1/perception/objects
PUBLISH_TOPIC = "ap1/perception/objects_2d"

YOLO_MODEL_FILE = "yolo26n.onnx"
MODEL_CLASSES = {11: ObjectClass.SIGN_STOP,}


def default_yolo_model() -> str:
    """Path to the YOLO model installed with the package: <ap1_perception share>/models."""
    return str(Path(get_package_share_directory("ap1_perception")) / "models" / YOLO_MODEL_FILE)


class YoloNode(Node):
    def __init__(self):
        super().__init__('ap1_perception_yolo')

        self._bridge = CvBridge()

        # Startup-only parameters: changing these needs a node restart (not a rebuild).
        # model_path defaults to the model installed with the package, so
        # full_system.launch.py can start this node without parameters.
        model_path = self._declare('model_path', default_yolo_model(),
                                   'YOLO .onnx model; defaults to the one installed with the package', True)
        device = self._declare('device', 'auto', "'auto', 'cuda' or 'cpu'", True)

        # Tuning parameter: can also be changed live with `ros2 param set`.
        self._declare('conf_threshold', 0.25, 'Min detection confidence, 0-1')

        # Detected classes are fixed by MODEL_CLASSES, not a parameter
        self._model = YOLO(model_path=model_path, classes=list(MODEL_CLASSES), device=device)
        self._apply_tuning()
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(lambda _params: self._apply_tuning())
        self.get_logger().info(
            f"YOLO loaded from {model_path}, running on {self._model.provider} "
            f"(conf={self._model.conf}, classes={list(MODEL_CLASSES.values())})."
        )

        self._pub = self.create_publisher(Detection2DArray, PUBLISH_TOPIC, 10)
        self.create_subscription(Image, COLOR_TOPIC, self._image_callback, 10)

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

    def _image_callback(self, color_msg: Image) -> None:
        # Always publish, with the source image's header (stamp and camera
        # frame): an empty array means nothing was detected in this frame.
        out = Detection2DArray()
        out.header = color_msg.header

        # YOLO expects bgr encoding not rgb
        color = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        boxes, scores, classes = self._model.forward(color)
        for box, score, cls in zip(boxes, scores, classes):
            out.detections.append(self._detection(box, score, cls, out.header))

        self._pub.publish(out)

    def _detection(self, box, score, cls, header) -> Detection2D:
        """One Detection2D from an xyxy pixel box."""
        x1, y1, x2, y2 = (float(v) for v in box)
        det = Detection2D()
        det.header = header
        det.id = ''  # not tracked yet
        det.bbox.center.position.x = (x1 + x2) / 2
        det.bbox.center.position.y = (y1 + y2) / 2
        det.bbox.size_x = x2 - x1
        det.bbox.size_y = y2 - y1

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = MODEL_CLASSES[int(cls)]
        hyp.hypothesis.score = float(score)
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
