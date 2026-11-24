# AP1 Perception
---
Contains the perception team's codebase, revolving around the package 'ap1_perception', our ROS2 package.

## Running in Docker

Perception builds and runs in containers on CPU or GPU. See [docker/README.md](../../docker/README.md).

## Road segmentation (v0)

`road_seg_node` labels every pixel of the front camera image with a class (road,
sidewalk, car, ...) using YOLO26n-sem, pretrained on Cityscapes (19 classes).

```bash
ros2 launch ap1_perception road_seg.launch.py            # device from AP1_GPU, model from the package
ros2 launch ap1_perception road_seg.launch.py device:=cpu model_path:=/opt/ap1/models/yolo26n-sem.onnx
```

Topics and rate are in [config/road_seg.yaml](config/road_seg.yaml). The model is
`models/yolo26n-sem.onnx` (Git LFS); rebuild it with
[tools/export_roadseg.py](tools/export_roadseg.py).

### Input

The camera image (RealSense colour stream by default) and its `CameraInfo`. The
node keeps only the latest frame and segments it at `rate_hz` (10 Hz). Each frame
is resized keeping its aspect ratio (1920x1080 → 768x432) and padded to the model
input (768x448).

### Output (internal, for the sensor association node)

| Topic | Type | Content |
|---|---|---|
| `ap1/perception/road_seg/class_map` | `sensor_msgs/Image`, `mono8` | One class ID per pixel, 768x432 for a 1920x1080 camera (padding removed). Header copied from the camera image: `stamp` is the capture time, `frame_id` the camera optical frame. |
| `ap1/perception/road_seg/camera_info` | `sensor_msgs/CameraInfo` | The camera's calibration scaled to 768x432 (fx, fy, cx, cy × 0.4), same header. Project lidar points with this to land on class map pixels. |
| `ap1/perception/road_seg/label_info` | `vision_msgs/LabelInfo`, latched | Class ID → name. Read the IDs from here rather than hard-coding them. |

Classes used in v0: `0 road` (drivable area) and `1 sidewalk` (its border with
road is roughly the curb). The other 17 Cityscapes classes are published too.

Known limits of the pretrained model: the ego car's hood is labelled as road (mask
the bottom rows), there's no curb, lane-marking or stop-line class, and no
ego/oncoming split. These need fine-tuning (follow-up to #91).

### Towards `ap1/perception/LaneDetected`

How the association node can turn the class map into `LaneDetected`:

- **Road area:** class map pixels with class `road`, matched to the ground points
  from the ground filter (or projected onto the ground plane), in `base_link`.
- **Left / right boundaries:** for each image row, the leftmost and rightmost road
  pixel of the road region in front of the car (the road/sidewalk edge is about
  the curb), projected to the ground the same way.
- **Stop line:** optional in `LaneDetected`; not available from Cityscapes classes,
  so empty in v0.

For Planning & Control, the proposal is a `nav_msgs/OccupancyGrid`: a bird's-eye
grid in `base_link` of the area ahead, cells marked drivable from the road pixels.
Format to be agreed with P&C; not implemented in v0.
