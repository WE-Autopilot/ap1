"""Planning outline for non-ground point-in-box association v0.

Signatures and contracts only; numerical operations are not implemented.
The object-selection responsibilities currently in projection.py and the YOLO
callback are duplicated here as proposed interfaces for sprint review. Existing
callers stay intact. Deprecation of projection.py is a later integration change.

No ROS messages, YOLO inference, ground filtering, clustering, KITTI or Numba
belong in this module. All output geometry is calculated in base_link.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]
BoolArray = NDArray[np.bool_]


class AssociationMatch(NamedTuple):
    """Proposed result record; detection_index refers to the original box row."""

    detection_index: int
    point_indices: IntArray
    position_xyz: FloatArray
    extent_xyz: FloatArray


def transform_points(
    points_xyz: FloatArray,
    transform_target_from_source: FloatArray,
) -> FloatArray:
    """Express corresponding XYZ rows in a different coordinate frame.

    Inputs:
        points_xyz: (N, 3), coordinates in metres in the source frame.
        transform_target_from_source: (4, 4), a rigid homogeneous transform.

    Return:
        (N, 3) points in the target frame, without modifying the input.

    Preserve ordering and point count, including invalid rows, so projection
    indices still identify the original base_link points. Reject invalid
    transform dimensions or values. Filtering belongs to the projection stage.
    """
    raise NotImplementedError('Planning outline only.')


def pointcloud_to_pixel(
    points_camera: FloatArray,
    projection_matrix: FloatArray,
    *,
    rectification_matrix: FloatArray,
    image_size: tuple[int, int],
    min_depth_m: float = 0.1,
) -> tuple[FloatArray, IntArray]:
    """Planned successor to projection.pointcloud_to_pixel, with validity checks.

    Inputs:
        points_camera: (N, 3), points in the camera optical frame.
        projection_matrix: (3, 4), obtained from image_geometry and matched
            to the image coordinate system used by the detection boxes.
        rectification_matrix: (3, 3), rotation required by that projection.
            Identity if already accounted for upstream; do not apply it twice.
        image_size: (width, height) of that image, in pixels.
        min_depth_m: positive threshold for usable camera depth.

    Return:
        pixels_xy: (M, 2), finite, in-image pixel coordinates, without rounding.
        point_indices: (M,), corresponding rows of points_camera.

    Exclude nonfinite points, points behind/near the projection plane and
    out-of-image projections. Empty input produces shaped empty outputs.
    Validate calibration rather than interpreting invalid calibration as an
    empty scene. This contract uses rectified-image boxes; raw distorted-image
    projection would require an explicit alternative agreed at integration.

    The signature intentionally extends the old helper; it is not a drop-in
    replacement for existing projection.py callers.
    """
    raise NotImplementedError('Planning outline only.')


def check_box(coords: FloatArray, box: FloatArray) -> BoolArray:
    """Duplicate the responsibility of projection.check_box in the outline.

    Inputs:
        coords: (M, 2), valid projected (u, v) pixels.
        box: (4,), [xmin, ymin, xmax, ymax] in the same pixel coordinates.

    Return:
        (M,) boolean mask. True means the projected point is inside the box.

    Planned boundary convention: xmin <= u < xmax and ymin <= v < ymax.
    Reject malformed array shapes. An empty/reversed/nonfinite box selects
    nothing. These validity rules are intentional; compatibility with the
    legacy helper is required for valid inputs, not its invalid-input quirks.

    This function only constructs a mask. The caller uses it to select the
    corresponding original XYZ rows; the median is not calculated on pixels.
    """
    raise NotImplementedError('Planning outline only.')


def get_centroid(points: FloatArray) -> FloatArray:
    """Duplicate projection.get_centroid's coordinate-wise median operation.

    Input:
        points: (M, 3), finite associated points already in base_link.

    Return:
        (3,), coordinate-wise median XYZ in metres, not the arithmetic mean.

    The caller enforces the minimum support count before invoking this helper.
    Reject empty/malformed/nonfinite input. Do not return a zero placeholder.
    Calculate the median after transforming points into the output frame;
    rotating a coordinate-wise median is not generally equivalent.
    """
    raise NotImplementedError('Planning outline only.')


def get_point_extent(points: FloatArray) -> FloatArray:
    """Calculate the observed axis-aligned size of associated base_link points.

    Input:
        points: (M, 3), finite nonempty associated points in metres.

    Return:
        (3,), [max(x)-min(x), max(y)-min(y), max(z)-min(z)].

    Zero extent along an axis is valid. No inferred size, rotation, padding or
    outlier trimming is applied. This describes observed returns, not the full
    physical object. Reject empty/malformed/nonfinite input.
    """
    raise NotImplementedError('Planning outline only.')


def box_select_points(
    points_base: FloatArray,
    boxes_xyxy: FloatArray,
    transform_camera_from_base: FloatArray,
    projection_matrix: FloatArray,
    *,
    rectification_matrix: FloatArray,
    image_size: tuple[int, int],
    min_points: int = 3,
    min_depth_m: float = 0.1,
) -> list[AssociationMatch]:
    """Proposed home for the box-selection/median stage moving out of YOLO.

    Inputs:
        points_base: (N, 3), finite non-ground points expressed in base_link.
        boxes_xyxy: (B, 4), original-image pixel boxes, in detection order.
        transform_camera_from_base: (4, 4), camera optical <- base_link.
        projection_matrix, rectification_matrix, image_size:
            One consistent camera-calibration snapshot for this cycle.
        min_points: positive minimum support; exactly three passes by default.
        min_depth_m: positive projection-depth threshold.

    Planned sequence, duplicated from the current object-association path:
        1. Transform and project the cloud once, retaining original indices.
        2. For each box, call check_box on the projected coordinates.
        3. Apply the mask to the projection's point_indices, then select the
           corresponding points_base rows. Do not take the median of pixels.
        4. Omit boxes with fewer than min_points selected points.
        5. Calculate get_centroid and get_point_extent on selected XYZ rows.

    Return:
        Supported AssociationMatch records in original detection order.
        detection_index preserves correspondence when other boxes are skipped.
        point_indices refers to points_base; its length is the support count.
        position_xyz is used for both result pose and box centre under #90.

    Empty clouds/boxes return an empty list. Invalid shapes, parameters or
    calibration raise errors. Boxes may share points; no clustering, background
    rejection or exclusive assignment is implied.

    Unlike the legacy box_select_points helper, unsupported detections are
    omitted rather than represented by [0, 0, 0]. No ROS messages are created.
    """
    raise NotImplementedError('Planning outline only.')
