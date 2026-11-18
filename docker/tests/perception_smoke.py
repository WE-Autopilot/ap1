"""Smoke test for the perception image. Runs inside the container.

    python3 perception_smoke.py --model /opt/ap1/models/yolo11n.onnx --device cpu

Checks that the runtime dependencies import and work together, that GPU images
have CUDA-enabled onnxruntime, and that YOLO runs end to end. Doesn't need
camera data or a ROS graph.
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np


def check(name, fn):
    try:
        detail = fn()
    except Exception as e:  # noqa: BLE001 - report every failure, then exit non-zero
        print(f"FAIL  {name}: {type(e).__name__}: {e}")
        return False
    print(f"ok    {name}" + (f": {detail}" if detail else ""))
    return True


def imports():
    import ap1_msgs.msg  # noqa: F401
    import cv2
    import onnxruntime
    import rclpy  # noqa: F401
    import sensor_msgs_py.point_cloud2  # noqa: F401
    import ap1_perception.yolo_node  # noqa: F401
    return f"numpy {np.__version__}, opencv {cv2.__version__}, onnxruntime {onnxruntime.__version__}"


def cv_bridge_roundtrip():
    # cv_bridge is built against apt's numpy; a pip numpy upgrade breaks this
    from cv_bridge import CvBridge
    bridge = CvBridge()
    img = np.random.randint(0, 255, (48, 64, 3), dtype=np.uint8)
    out = bridge.imgmsg_to_cv2(bridge.cv2_to_imgmsg(img, encoding="bgr8"), desired_encoding="bgr8")
    assert np.array_equal(img, out), "image changed in round trip"


def providers():
    import onnxruntime
    available = onnxruntime.get_available_providers()
    if os.environ.get("AP1_GPU") == "1":
        assert "CUDAExecutionProvider" in available, f"GPU image without CUDA provider: {available}"
    return ", ".join(available)


def node_executable():
    from ament_index_python.packages import get_package_prefix
    exe = Path(get_package_prefix("ap1_perception")) / "lib" / "ap1_perception" / "yolo_node"
    assert exe.is_file(), f"{exe} missing"
    return str(exe)


def yolo(model, device):
    from ap1_perception.yolo import YOLO
    detector = YOLO(model_path=model, device=device)
    boxes, classes = detector.forward(np.zeros((480, 640, 3), dtype=np.uint8))
    assert boxes.shape[1] == 4 and len(boxes) == len(classes)
    return f"ran on {detector.provider}, {len(boxes)} detection(s) on a blank frame"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", help="YOLO .onnx model; skips the inference check if omitted")
    parser.add_argument("--device", default="cpu", choices=["cpu", "cuda", "auto"])
    args = parser.parse_args()

    results = [
        check("imports", imports),
        check("cv_bridge + numpy", cv_bridge_roundtrip),
        check("onnxruntime providers", providers),
        check("yolo_node installed", node_executable),
    ]
    if args.model:
        results.append(check(f"YOLO inference (device={args.device})", lambda: yolo(args.model, args.device)))
    else:
        print("skip  YOLO inference: no --model given")

    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
