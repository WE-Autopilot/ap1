# Simulation

Turns a raw CARLA test-drive bag into one that looks like it came from the golf
kart's sensors, so maps and LIO-SAM / NDT params carry over between sim and real.

| Sensor | Real hardware | Config |
|---|---|---|
| LiDAR | RoboSense Airy Lite | [config/airy_lite.yaml](config/airy_lite.yaml) |
| IMU | MicroStrain 3DM-CV7-AHRS | [config/cv7_ahrs.yaml](config/cv7_ahrs.yaml) |
| Mount positions | where both sit on the kart | [config/mounts.yaml](config/mounts.yaml) |

```
CARLA test drive ──ros2 bag record──> raw bag ──filter_bag.py──> filtered bag ──> LIO-SAM / NDT
```

## Bag names

Bags live outside Git and are named `<date>_<map>_<lidar config>-v<N>_<imu
config>-v<N>_<raw|filtered>`, with `N` from each config's `version`, e.g.
`2026-10-29_town01_airylite-v1_cv7-v1_raw`. The commands below use:

```bash
BAG=bags/2026-10-29_town01_airylite-v1_cv7-v1
```

## Running the filter

```bash
source /opt/ros/jazzy/setup.bash
python3 simulation/filter_bag.py ${BAG}_raw ${BAG}_filtered
```

- With no `--lidar`/`--imu` flags it uses the kart's sensors,
  [config/airy_lite.yaml](config/airy_lite.yaml) and
  [config/cv7_ahrs.yaml](config/cv7_ahrs.yaml).
- Pass configs to use other sensors. Both flags can be repeated, and only the
  sensors you pass are filtered:

  ```bash
  python3 simulation/filter_bag.py ${BAG}_raw ${BAG}_filtered \
    --lidar simulation/config/front_lidar.yaml --lidar simulation/config/rear_lidar.yaml \
    --imu   simulation/config/cv7_ahrs.yaml
  ```
- The output is packaged like a kart recording, so LIO-SAM uses one config for
  both (see "What's in the filtered bag" below).
- The raw lidar/IMU topics are dropped unless `--keep-raw` is passed. Any other
  topic is copied through.
- Output is written as `mcap` by default (`--storage sqlite3` to change).
- Same bag + same `seed` in the configs = identical output.
- At the end it prints a summary (sweeps, points per sweep, IMU rates) and warns
  if the raw bag doesn't meet the requirements below.
- If CARLA's frame or topic names change: `--vehicle-frame` (default
  `carla_base_link`) and `--ground-truth-odom` (default `/carla/hero/odometry`).

### What's in the filtered bag

| Topic | Contents | In a real kart bag? |
|---|---|---|
| `/sensing/lidar/pointcloud` | sweeps, frame `lidar_link` | yes, same |
| `/sensing/imu/data` | IMU, frame `imu_link` | yes, same |
| `/tf_static` | `base_link → lidar_link`, `base_link → imu_link`, from `mounts.yaml` | yes, same |
| `/ground_truth/tf` | CARLA's true pose, `carla_map → base_link` | no; LIO-SAM ignores it |
| `/ground_truth/odometry` | CARLA's true pose as odometry | no; LIO-SAM ignores it |

There is no `/tf`: on the kart, and in a LIO-SAM run, LIO-SAM itself publishes
`map → odom → base_link`. Ground truth is kept off `/tf` so it can't fight
that. Every message is stamped with CARLA's sim time and `/clock` is dropped,
which is why `ros2 bag info` shows dates in 1970.

### Playing it back

With LIO-SAM, exactly like a real bag:

```bash
ros2 bag play ${BAG}_filtered --clock
```

To view it in rviz without LIO-SAM, plug ground truth into the TF tree and use
fixed frame `carla_map`:

```bash
ros2 bag play ${BAG}_filtered --clock --remap /ground_truth/tf:=/tf
```

Never use that remap while LIO-SAM runs: ground truth and LIO-SAM would both
claim to be `base_link`'s parent, and lookups would flip between them.

- It plays at real driving speed (sim time), even though recording took
  about 3x longer.
- To compare with the raw data, filter with `--keep-raw`; the raw CARLA topics
  are then in the same bag with matching timestamps.

## What the raw bag must contain

The filter can only remove and degrade data, so CARLA has to produce at least
what the real sensors do.

**LiDAR** (`sensor.lidar.ray_cast`):
- A beam within `beams.tolerance_deg` (0.1°) of every Airy Lite beam, which in
  practice means matching them exactly: `channels=24`, `upper_fov=33.5`,
  `lower_fov=-11.5`.
- `rotation_frequency=10`.
- Horizontal resolution of 0.6° or finer: `points_per_second` ≥ 144,000 for 24
  channels (more is fine, the filter thins it).
- `range` ≥ 60.
- `noise_stddev=0` and `dropoff_general_rate=0`; the filter adds range noise.

**IMU** (`sensor.other.imu`):
- At least 100 Hz (`sensor_tick` ≤ 0.01). Faster input is decimated.
- All `noise_*` attributes 0; the filter adds all IMU errors.

**World:** synchronous mode with `fixed_delta_seconds=0.01`. This is what gives
the 100 Hz IMU, and it makes each lidar message a 36° slice of the rotation,
which the filter stitches into full sweeps. Per-point `time` comes from which
tick a point was in, so it has 10 ms resolution. If CARLA ticks at 0.1 s, each
message is a whole sweep and every point gets the same time (the filter warns).

**Topics and frames (assumed).** There is no official script for recording
the raw bag yet; one is the next task. Until then, the filter assumes the raw
bag looks like this:

| Topic | Contents |
|---|---|
| `/carla/hero/lidar` | `sensor_msgs/PointCloud2` in ROS convention (x forward, y left, z up), frame `lidar_link` |
| `/carla/hero/imu` | `sensor_msgs/Imu` in ROS convention, frame `imu_link` |
| `/carla/hero/odometry` | ground-truth `nav_msgs/Odometry`, child frame `carla_base_link` |
| `/tf` | ground truth, `carla_map → carla_base_link` |
| `/tf_static` | `carla_base_link → lidar_link` / `imu_link`, matching [config/mounts.yaml](config/mounts.yaml) |
| `/clock` | sim time (dropped by the filter) |

If the recording script uses other names, change `topics.input` in the
configs and pass `--vehicle-frame` / `--ground-truth-odom`; nothing else needs
editing. The filter turns `carla_base_link` into `base_link` and moves ground
truth off `/tf` (see "What's in the filtered bag").

Recording with the synchronous sim can run slower than real time (about a
third of real time over Tailscale), so allow for that when deciding how long
to record.

## Mount positions

[config/mounts.yaml](config/mounts.yaml) is the one place that says where each
sensor sits relative to `base_link`, in ROS axes (x forward, y left, z up,
metres; roll/pitch/yaw in degrees). Whatever records the raw bag is assumed
to use it for two things that must agree: where the sensors are attached in
CARLA, and the `/tf_static` in the bag. The filter passes that `/tf_static`
through, so nothing else needs editing.

Mounts can be tilted, rotated, or even upside down: set `rpy`, e.g.
`[0.0, 10.0, 0.0]` tilts forward 10° and `[180.0, 0.0, 0.0]` is upside down.
Keep variants in separate files.

The current values are estimates (lidar 2.5 m up, IMU 1.5 m up, both level
and centred). When the real ones are known:

1. Put them in `mounts.yaml` and change the `[est]` tags to their source.
2. Check where `base_link` is defined on the kart. The CARLA car's origin is
   assumed to be `base_link`; if they differ, the mounts need that offset.
3. Record a new raw bag and filter it. Existing bags keep the old positions.
4. Check in rviz that `lidar_link` / `imu_link` sit in the right places, and
   that the empty ring around the car has a radius of about
   lidar height / tan(11.5°) (≈ 4.9 × height).

## What the filter does

Numbers are the default configs' values; each comes from the sensor's config.

**LiDAR**, per sweep:
1. Stitch CARLA's per-tick slices into one 10 Hz rotation.
2. Keep points from 0.1 m (blind zone) to 60 m (max range), inside the
   horizontal FOV.
3. Snap each point to the nearest configured beam → `ring` (Airy Lite: 0 = lowest beam).
   Points not within tolerance of a beam are dropped.
4. Keep at most one point per ring per 0.6° of azimuth.
5. Add Gaussian range noise along each ray, sized by the accuracy table for
   that point's channel and distance (optional dropout).
6. Scale intensity from CARLA's 0–1 to 0–255.
7. Publish on `/sensing/lidar/pointcloud` as `x y z intensity` (float32),
   `ring` (uint16), `time` (float32, seconds since the header stamp). This is
   the Velodyne-style layout LIO-SAM reads (`sensor: velodyne`).

**IMU**, per sample:
1. Decimate to 100 Hz.
2. Accel and gyro: add a turn-on bias (drawn once per run), a slowly
   drifting bias (first-order Gauss-Markov), and white noise.
3. Orientation: apply a slowly drifting roll/pitch/heading error to CARLA's
   orientation. If the input has no orientation, the output marks it unknown
   (`orientation_covariance[0] = -1`) and the filter warns.
4. Set covariances from the datasheet.
5. Publish on `/sensing/imu/data`.

## Adding a sensor

Copy the closest existing config, change the values to the new datasheet, and
pass it with `--lidar` or `--imu`. No code changes are needed. Every sensor
needs its own `topics.input` and `topics.output`. Keep a source comment on each
value, as the existing configs do.

### LiDAR config

| Key | Required | Meaning |
|---|---|---|
| `name` | no | label in the summary output |
| `topics.input` / `topics.output` | yes | raw CARLA topic in, filtered topic out |
| `frame_id` | yes | frame id of the output clouds |
| `input_left_handed` | no (false) | flip y if the bag is in CARLA's left-handed frame |
| `rotation_hz` | yes | sweeps per second |
| `beams.count`, `beams.lower_deg`, `beams.upper_deg` | one of these two | evenly spaced beams |
| `beams.elevations_deg: [...]` | | explicit beam angles, for uneven spacing; list order = ring order |
| `beams.tolerance_deg` | yes | max distance from a CARLA ray to a real beam |
| `min_range_m` | no (0) | blind zone |
| `max_range_m` | yes | max range |
| `horizontal_fov_deg: [start, end]` | no (360°) | azimuth limits, counter-clockwise, 0° = forward |
| `azimuth_resolution_deg` | yes | horizontal angle between points on a ring |
| `noise.range_accuracy` | no (no noise) | list of zones: `channels: [first, last]` (1-based, optional = all beams) and `steps: [[up_to_m, stddev_m], ...]` |
| `noise.dropout_rate` | no (0) | fraction of returns removed at random |
| `intensity_scale` | no (1) | multiplier on CARLA intensity |
| `seed` | no (0) | random seed |

Not supported: non-repetitive scan patterns (e.g. Livox), dual returns, and
output layouts other than XYZIRT.

### IMU config

Values can be bare numbers (SI) or strings with a unit as the datasheet
prints them, e.g. `30 ug/rtHz`, `0.14 deg/rt-h`, `0.75 mg`. Units: `m/s^2`,
`g`, `mg`, `ug`, `rad/s`, `deg/s`, `deg/h`, `rad`, `deg`, `s`, `min`, `h`,
each with an optional `/rtHz`, plus the random walks `m/s/rt-h`, `deg/rt-h`,
`rad/rt-h`.

| Key | Required | Meaning |
|---|---|---|
| `name` | no | label in the summary output |
| `topics.input` / `topics.output` | yes | raw CARLA topic in, filtered topic out |
| `frame_id` | yes | frame id of the output messages |
| `input_left_handed` | no (false) | convert from CARLA's left-handed frame |
| `rate_hz` | yes | output rate; faster input is decimated |
| `accel.noise_density` / `gyro.noise_density` | no (0) | white noise; per-sample σ = density × √rate. Also accepts the random-walk figure (`deg/rt-h`) |
| `accel.white_noise_stddev` / `gyro.white_noise_stddev` | no | per-sample σ at `rate_hz`, e.g. read off an Allan plot; replaces `noise_density` |
| `accel.turn_on_bias` / `gyro.turn_on_bias` | no (0) | bias drawn once per run, as 1σ |
| `accel.bias_instability` / `gyro.bias_instability` | no (0) | datasheet bias instability B; Gauss-Markov σ = B × 0.664 / 0.618 |
| `accel.bias_correlation_time` / `gyro.bias_correlation_time` | if bias_instability set | Gauss-Markov τ |
| `orientation` | no | leave out if the sensor doesn't output orientation; output is then marked unknown |
| `orientation.roll_pitch_stddev`, `.heading_stddev` | no (0) | orientation error σ |
| `orientation.correlation_time` | if the stddevs are set | how fast the error wanders |
| `seed` | no (0) | random seed |

## Specs and sources

### RoboSense Airy Lite

Source: [RoboSense Airy Lite product manual](https://robosense-robotics.github.io/product-manual/en/AiryLite/)
unless noted.

| Spec | Datasheet | Config value |
|---|---|---|
| Channels | 24 | `beams.count: 24` |
| Vertical FOV | −11.5° to +33.5° | `beams.lower_deg/upper_deg` |
| Vertical resolution | 1.96° | evenly spaced: 45° / 23 = 1.96° |
| Horizontal FOV | 360° | — |
| Horizontal resolution | 0.6° | `azimuth_resolution_deg: 0.6` |
| Frame rate | 10 Hz | `rotation_hz: 10` |
| Points per second | 144,000 (24 × 600 × 10) | result of the above |
| Max range | 60 m (30 m @10% outer zones, 40 m @10% central −0.25° to +22.25°) | `max_range_m: 60` |
| Blind zone | < 0.1 m | `min_range_m: 0.1` |
| Range accuracy, channels 7–18 | 0.5 cm (0–10 m), 1 cm (10–40 m) | `noise.range_accuracy` |
| Range accuracy, channels 1–6, 19–24 | 1 cm (0–30 m), 1.5 cm (30–40 m) | `noise.range_accuracy` |
| Return mode | strongest only | single return, as CARLA does |
| Reflectivity | 1–255 | `intensity_scale: 255` |

### MicroStrain 3DM-CV7-AHRS

Source: [Parker MicroStrain 3DM-CV7 datasheet, doc 8400-0141 Rev C (2023)](https://www.mouser.com/datasheet/2/1083/Parker_8400_0141_3DM_CV7_Datasheet-3239993.pdf)
unless noted.

| Spec | Datasheet | Config value | Used as |
|---|---|---|---|
| Output rate | 1–1000 Hz (configurable) | `rate_hz: 100` | MicroStrain ROS driver default for `/imu/data` |
| Accel noise density | 30 µg/√Hz | `accel.noise_density: 30 ug/rtHz` | documentation; overridden below |
| Accel white noise at 100 Hz | Allan deviation ~150 µg at 0.01 s | `accel.white_noise_stddev: 150 ug` | per-sample σ = 1.5e-3 m/s² |
| Accel turn-on bias | 40 µg | `accel.turn_on_bias: 13.3 ug` | constant bias per run, 40 µg as a 3σ bound |
| Accel bias instability | 18 µg | `accel.bias_instability: 18 ug` | Gauss-Markov σ = 19 µg |
| Accel bias correlation time | Allan minimum ~20 s | `accel.bias_correlation_time: 10 s` | Gauss-Markov τ = 20 s / 1.89 |
| Gyro noise density | 8.5 °/h/√Hz (ARW 0.14 °/√h) | `gyro.noise_density: 8.5 deg/h/rtHz` | documentation; overridden below |
| Gyro white noise at 100 Hz | Allan deviation ~75 °/h at 0.01 s | `gyro.white_noise_stddev: 75 deg/h` | per-sample σ = 3.6e-4 rad/s |
| Gyro turn-on bias | 0.004 °/s | `gyro.turn_on_bias: 0.00133 deg/s` | constant bias per run, 0.004 °/s as a 3σ bound |
| Gyro bias instability | 1.5 °/h | `gyro.bias_instability: 1.5 deg/h` | Gauss-Markov σ = 1.6 °/h |
| Gyro bias correlation time | Allan minimum ~600 s | `gyro.bias_correlation_time: 300 s` | Gauss-Markov τ = 600 s / 1.89 |
| Roll/pitch accuracy | 0.25° static, 0.5° dynamic | `orientation.roll_pitch_stddev: 0.167 deg` | orientation error σ, 0.5° as a 3σ bound |
| Heading accuracy (AHRS) | 0.5° static, 2° dynamic | `orientation.heading_stddev: 0.667 deg` | orientation error σ, 2° as a 3σ bound |

Not modelled: scale-factor error, temperature effects, accel/gyro range
saturation (±4–16 g, ±250–1000 °/s), magnetometer.

### Not from the spec sheets

Values that aren't printed as numbers in the spec sheets, and why each one is
set the way it is. Tagged in the configs. Revisit them once there is real
sensor data.

**Rule:** where something is unknown or can't be defended, choose the
cleaner option. A sim that is noisier than the real sensor would throw off
LIO-SAM for reasons the kart never sees, while a slightly clean sim only
makes tuning a little optimistic.

**Backed by another source**
- **Ring 0 is the lowest beam.** RoboSense's
  [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver) (`chan_angles.hpp`,
  `genUserChan`) numbers rings by ascending vertical angle, and has an Airy
  Lite loader.
- **Lidar accuracy is 1σ.** RoboSense states Airy accuracy as "1cm@1σ" and
  resellers quote these Airy Lite figures as 1σ.
- **IMU rate 100 Hz.** The default `imu_data_rate` in MicroStrain's
  [ROS driver params](https://github.com/LORD-MicroStrain/microstrain_inertial_driver_common/blob/main/config/params.yml).
- **Dynamic orientation accuracies (0.5°, 2°) rather than static.** The
  datasheet footnotes them as "automotive conditions", which is what a drive
  is.

**Derived from the datasheet with standard methods**
- **Bias drift** is a first-order Gauss-Markov process. Its σ is the
  datasheet bias instability × 0.664 / 0.618: the datasheet number is the
  Allan floor / 0.664 (IEEE 952 convention), and a Gauss-Markov process's
  Allan deviation peaks at 0.618σ. Its τ is the Allan minimum's averaging
  time / 1.89, where a Gauss-Markov Allan curve peaks
  ([Li, Du, Zhai 2016](https://download.atlantis-press.com/article/25854246.pdf)).
  The minima (~20 s accel, ~600 s gyro) are read by eye off the plots.
- **White noise at 100 Hz** is read off the Allan plots at τ = 0.01 s (accel
  ~150 µg, gyro ~75 °/h). That is below noise density × √rate (300 µg,
  85 °/h): the curves flatten at short τ, so the CV7's internal filtering makes
  each sample quieter than the density alone predicts. The override only holds
  at 100 Hz.

**Judgment calls with no source**, each set to the clean side
- **Turn-on bias and orientation accuracy as 3σ bounds.** The datasheet
  doesn't say whether these are 1σ or limits. Reading them as limits gives a
  third of the error.
- **Orientation error τ = 10 s**, about the length of a manoeuvre. AHRS
  errors come mainly from accelerations and magnetic disturbance while
  driving, which change on that scale. With σ fixed, τ changes how the error
  moves, not how big it is, so neither direction is clearly cleaner.
- **White noise read off the Allan plots** instead of the noise-density
  formula. The plot gives half the accel noise; it is the datasheet's own
  measurement, and the cleaner of the two.
- **Lidar accuracy past 40 m** reuses the 40 m value. The manual stops at
  40 m because that is where 10% targets drop out.
- **`dropout_rate: 0`.** Real misses come from dark, shiny or glass surfaces,
  which CARLA doesn't model. There is no figure to base a rate on.
- **`intensity_scale: 255`** only puts CARLA's 0–1 intensity in the Airy's
  0–255 range. LIO-SAM and NDT don't use intensity, so this mainly affects
  rviz colouring.
- **`beams.tolerance_deg: 0.1`.** 0.1° is 10 cm of height at 60 m, so rings
  stay sharper than the range noise. It needs CARLA's beams to match exactly.
- **Per-point `time`** has one CARLA tick (10 ms) of resolution; the real
  sensor stamps each 2.7 ms packet.

### Topics and frames

| | Value | Source |
|---|---|---|
| LiDAR output topic | `/sensing/lidar/pointcloud` | ICD |
| IMU output topic | `/sensing/imu/data` | ICD |
| LiDAR frame id | `lidar_link` | ICD Frames & Transforms page; the raw bag must use the same name |
| IMU frame id | `imu_link` | ICD Frames & Transforms page; the raw bag must use the same name |
| Input topics | `/carla/hero/lidar`, `/carla/hero/imu` | assumed (no official recording script yet) |
