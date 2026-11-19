# RViz2 for bag playback

`bags.rviz` shows a bag that follows the AP1.26 sensing contract (KITTI bags
from `scripts/fetch_bags.py`, CARLA or kart recordings) next to perception's
output: the front image, a camera overlay, the lidar cloud, TF,
`/ground_truth/objects` and `/ap1/perception/objects`. Fixed frame is
`base_link`.

The camera overlay draws the cloud and boxes through the camera model on top
of the image. Points landing on the objects they hit means the extrinsics on
`/tf_static` are right.

## Run it

The images are ros-base and have no RViz2. Run RViz2 on the host (native
Jazzy on Linux, or WSLg on Windows) and play the bag in the container. Host
networking and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (the compose
defaults) let them see each other; use the same `ROS_DOMAIN_ID` in both.

Host, once:

```bash
sudo apt install ros-jazzy-rviz2 ros-jazzy-vision-msgs-rviz-plugins
python3 scripts/fetch_bags.py kitti
```

Container:

```bash
ros2 bag play /ws/bags/kitti/2011_09_26_drive_0005 --clock --loop
```

Host:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rviz2 -d src/perception/tools/rviz/bags.rviz --ros-args -p use_sim_time:=true
```

## Gotchas

- `--clock` + `use_sim_time:=true`: without them RViz2 compares bag stamps
  (2011 for KITTI) with wall time and drops every message on TF lookups.
- `/tf_static` only reaches RViz2 if the bag stores it as transient_local, as
  the published bags do. A bag without that QoS metadata plays it as volatile
  and late subscribers never get it: play it with
  `--qos-profile-overrides-path`.
- To see the vehicle move in a world frame, add
  `--remap /ground_truth/tf:=/tf` to `ros2 bag play` and set the fixed frame
  to the ground-truth parent frame (`kitti_map` for KITTI). Only when no
  localization is running (ICD #12).
- Lanes have no message type yet; show them as a debug marker topic until
  they do.
- macOS: Docker Desktop's VM network hides the container's topics from the
  host. Open the `.mcap` in Foxglove instead.
