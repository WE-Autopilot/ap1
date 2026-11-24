# Scripts

This is for scripts.

## Fetch bags

`fetch_bags.py` downloads the bags listed in `bags.toml` from Hugging Face
into `bags/` (mounted at `/ws/bags` in the dev containers), pinned to a
dataset commit and hash-checked. Run it on the host from the repo root (any
Python 3.11+, no packages); the containers see the bags at `/ws/bags`:

```bash
python3 scripts/fetch_bags.py                                  # every set
python3 scripts/fetch_bags.py kitti --bag 2011_09_26_drive_0005
```

The datasets are built and uploaded by
[ap1-datasets](https://github.com/WE-Autopilot/ap1-datasets). To view a bag
in RViz2, see `src/perception/tools/rviz/README.md`.

## Software smoke test

`software_smoke_test.sh` validates the AP1 software stack up to the hardware
boundary using synthetic inputs:

```bash
AP1_WORKSPACE=~/weap/ap1 ./scripts/software_smoke_test.sh
```

It checks that these topics publish at least once:

```text
/ap1/mapping/odometer
/ap1/mapping/entities
/ap1/mapping/lanes
/ap1/planning/target_path
/ap1/control/motor_power
/ap1/control/turn_angle
```

This requires the software smoke-test PRs that add synthetic perception,
synthetic odometry, synthetic actuation feedback, and full-system launch flags.

## RealSense IMU bag summary

`realsense_imu_bag_summary.py` summarizes accel/gyro streams in a RealSense
`.bag` file:

```bash
cd ~/weap/ap1/src/perception
.venv/bin/python ~/weap/ap1/scripts/realsense_imu_bag_summary.py \
  /mnt/c/Users/LukeB/Downloads/20260323_151704.bag
```

The `20260323_151704.bag` recording is motion-only. It can validate accel/gyro
sample counts, timestamps, axes, bias, and IMU fusion experiments. It cannot
exercise RGB/depth perception, lanes, entities, or point-cloud projection.

Current blocker: `realsense2_camera` can fail on IMU-only bags with
`No known base_stream found for transformations`, so use `pyrealsense2`
directly until AP1 has a dedicated IMU bag publisher or IMU-to-odometry fusion
node.
