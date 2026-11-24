"""Unit tests for the geometry yolo_node uses to place detections. No ROS or model needed."""
import numpy as np
import pytest

from ap1_perception.projection import box_geometry, cam_optical_to_base_link

FOCAL = 600.0
K = np.array([[FOCAL, 0, 320], [0, FOCAL, 240], [0, 0, 1]])


def test_optical_to_base_link_axes():
    # Optical +Z (ahead), +X (right), +Y (down) -> base_link +X, -Y, -Z
    pts = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)
    np.testing.assert_allclose(cam_optical_to_base_link(pts), [[1, 0, 0], [0, -1, 0], [0, 0, -1]])


def test_box_geometry_flat_face():
    # A 60 x 30 px box on a wall 5 m away is 0.5 m wide and 0.25 m tall, with no depth extent
    depth = 5.0
    box = np.array([290, 225, 350, 255], dtype=float)
    us, vs = np.meshgrid(np.arange(291, 350), np.arange(226, 255))
    pts = np.stack([(us.ravel() - 320) * depth / FOCAL,
                    (vs.ravel() - 240) * depth / FOCAL,
                    np.full(us.size, depth)], axis=1)
    center, size = box_geometry(K, box, pts)
    assert center[2] == pytest.approx(depth)
    np.testing.assert_allclose(size, [0.5, 0.25, 0.0], atol=1e-9)


def test_box_geometry_ignores_background_outliers():
    # 90 points on the object at 5 m, 10 background points at 30 m: median and
    # the middle-80% depth spread both stay on the object
    pts = np.zeros((100, 3))
    pts[:90, 2] = 5.0
    pts[90:, 2] = 30.0
    center, size = box_geometry(K, np.array([0, 0, 60, 60], dtype=float), pts)
    assert center[2] == pytest.approx(5.0)
    assert size[2] < 25.0
