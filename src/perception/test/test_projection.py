"""Unit tests for projecting points between the camera image and 3D."""
import numpy as np
import pytest

from ap1_perception.projection import box_select_points, check_box, get_dir_vec, pointcloud_to_pixel

K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])


def wall(depth=5.0, width=640, height=480, step=4):
    """Points of a flat wall `depth` metres in front of the camera, one every `step` px."""
    us, vs = np.meshgrid(np.arange(0, width, step), np.arange(0, height, step))
    z = np.full(us.size, depth)
    return np.stack([(us.ravel() - K[0, 2]) * z / K[0, 0], (vs.ravel() - K[1, 2]) * z / K[1, 1], z], axis=1)


def test_pointcloud_to_pixel():
    points = np.array([[0.0, 0.0, 5.0], [1.0, -0.5, 5.0]])
    np.testing.assert_allclose(pointcloud_to_pixel(K, points), [[320, 240], [440, 180]])


def test_dir_vec_round_trip():
    pixels = np.array([[0.0, 0.0], [320.0, 240.0], [600.0, 50.0]])
    rays = get_dir_vec(K, pixels)
    np.testing.assert_allclose(np.linalg.norm(rays, axis=1), 1.0)
    points = rays * (5.0 / rays[:, 2:3])  # scale each ray to z = 5 m
    np.testing.assert_allclose(pointcloud_to_pixel(K, points), pixels)


def test_check_box_is_half_open():
    box = np.array([10, 20, 30, 40])  # x1, y1, x2, y2
    coords = np.array([[15, 25], [10, 20], [30, 25], [15, 40], [5, 25], [15, 45]])
    assert check_box(coords, box).tolist() == [True, True, False, False, False, False]


def test_box_select_points_on_wall():
    centroids = box_select_points(K, wall(depth=5.0), np.array([[300, 220, 340, 260]]))
    assert centroids.shape == (1, 3)
    assert centroids[0, 2] == pytest.approx(5.0)
    np.testing.assert_allclose(centroids[0, :2], 0.0, atol=0.05)


def test_box_select_points_box_without_points():
    centroids = box_select_points(K, wall(), np.array([[2000, 2000, 2100, 2100]]))
    np.testing.assert_array_equal(centroids, np.zeros((1, 3)))


def test_box_select_points_no_boxes():
    assert box_select_points(K, wall(), np.zeros((0, 4))).shape == (0, 3)
