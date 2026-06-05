# Scripts

This is for scripts.

`perception_setup.sh` is for getting perceptions dependencies in order. If on a fresh machine run it. Must be ubuntu.

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
