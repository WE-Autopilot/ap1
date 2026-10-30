#!/usr/bin/env python3
"""
Offline filter: raw CARLA test-drive bag in, bag that looks like real
sensors out. Each lidar and IMU is described by its own YAML config; the
defaults are the golf kart's RoboSense Airy Lite and MicroStrain
3DM-CV7-AHRS.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 simulation/filter_bag.py <raw_bag> <filtered_bag>      # default configs
    python3 simulation/filter_bag.py <raw_bag> <filtered_bag> \
        --lidar config/front.yaml --lidar config/rear.yaml --imu config/imu.yaml

Lidar: CARLA's per-tick slices are stitched into full sweeps, cropped to the
sensor's range and horizontal FOV, snapped to its beams (ring), thinned to
its horizontal resolution, given per-point time and range noise. Output is
Velodyne-style XYZIRT (x y z intensity ring time), the layout LIO-SAM reads.

IMU: turn-on bias, drifting bias (Gauss-Markov) and white noise on accel and
gyro, orientation error, datasheet covariances, decimated to the output rate.

The output is packaged like a kart recording, so LIO-SAM runs on it with the
same config: /tf_static uses base_link, ground truth moves to /ground_truth/*
(off /tf), and messages carry sim time with /clock dropped, so it plays with
`ros2 bag play <bag> --clock`. Other topics are copied through.
simulation/README.md documents the config format and where each default
value comes from.

ROS is imported only by the bag I/O code, so the filter classes can be used
and tested with just numpy and PyYAML.
"""
import argparse
import math
import os

import numpy as np
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config')
DEFAULT_LIDARS = [os.path.join(CONFIG_DIR, 'airy_lite.yaml')]
DEFAULT_IMUS = [os.path.join(CONFIG_DIR, 'cv7_ahrs.yaml')]

G = 9.80665
DEG = math.pi / 180.0

# SI factor for each unit IMU configs may use. "<unit>/rtHz" is also accepted
# for any of these (noise densities); "/rt-h" units are random walks.
UNITS = {
    'm/s^2': 1.0, 'g': G, 'mg': 1e-3 * G, 'ug': 1e-6 * G,
    'rad/s': 1.0, 'deg/s': DEG, 'deg/h': DEG / 3600.0,
    'rad': 1.0, 'deg': DEG,
    's': 1.0, 'min': 60.0, 'h': 3600.0,
    'm/s/rt-h': 1.0 / 60.0, 'deg/rt-h': DEG / 60.0, 'rad/rt-h': 1.0 / 60.0,
}


def quantity(value):
    """'30 ug/rtHz' -> SI float. Bare numbers are taken as SI already."""
    if isinstance(value, (int, float)):
        return float(value)
    number, _, unit = str(value).strip().partition(' ')
    unit = unit.strip()
    base = unit[:-len('/rtHz')] if unit.endswith('/rtHz') else unit
    if base not in UNITS:
        raise ValueError(f'unknown unit "{unit}" in "{value}"; known: {sorted(UNITS)} (+ /rtHz)')
    return float(number) * UNITS[base]


# ---------- lidar ----------

def beam_elevations(beams):
    """Beam elevations (rad) in ring order: an explicit list, or evenly spaced."""
    if 'elevations_deg' in beams:
        return np.radians(np.asarray(beams['elevations_deg'], dtype=np.float64))
    return np.radians(np.linspace(beams['lower_deg'], beams['upper_deg'], beams['count']))


def range_crop(pts, r_min, r_max):
    r = np.linalg.norm(pts[:, :3], axis=1)
    return pts[(r >= r_min) & (r <= r_max)]


def azimuth_crop(pts, start_deg, end_deg):
    """Keep points with azimuth from start to end, counter-clockwise (x forward, y left)."""
    width = end_deg - start_deg
    if width >= 360.0:
        return pts
    az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
    return pts[np.mod(az - start_deg, 360.0) <= width]


def snap_to_beams(pts, beam_elev, tol):
    """Assign each point to the nearest real beam (ring); drop points between beams."""
    elev = np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1]))
    d = np.abs(elev[:, None] - beam_elev[None, :])
    ring = d.argmin(axis=1)
    keep = d[np.arange(len(pts)), ring] <= tol
    return pts[keep], ring[keep].astype(np.uint16)


def thin_azimuth(pts, ring, az_res):
    """Keep at most one point per (ring, azimuth bin)."""
    n_bins = int(math.ceil(2.0 * math.pi / az_res))
    az = np.mod(np.arctan2(pts[:, 1], pts[:, 0]), 2.0 * math.pi)
    key = ring.astype(np.int64) * n_bins + (az // az_res).astype(np.int64)
    _, idx = np.unique(key, return_index=True)
    return pts[idx], ring[idx]


def range_stddev(ring, r, accuracy):
    """
    Per-point range stddev from an accuracy table: a list of zones, each
    {channels: [first, last] (1-based, optional = all), steps: [[up_to_m,
    stddev_m], ...]}. Past a zone's last step its last value is used.
    """
    sigma = np.zeros(len(r))
    for zone in accuracy:
        if 'channels' in zone:
            lo, hi = zone['channels']
            in_zone = (ring >= lo - 1) & (ring <= hi - 1)
        else:
            in_zone = np.ones(len(r), dtype=bool)
        steps = np.asarray(zone['steps'], dtype=np.float64)
        idx = np.minimum(np.searchsorted(steps[:, 0], r[in_zone]), len(steps) - 1)
        sigma[in_zone] = steps[idx, 1]
    return sigma


def add_range_noise(pts, stddev, rng):
    """Move each point along its ray by N(0, stddev); stddev is per point or scalar."""
    r = np.linalg.norm(pts[:, :3], axis=1)
    scale = np.maximum(r + rng.normal(0.0, stddev, len(r)), 0.0) / np.maximum(r, 1e-9)
    out = pts.copy()
    out[:, :3] *= scale[:, None]
    return out


class LidarFilter:
    """
    Feed raw CARLA lidar messages in time order with add(). It returns a
    finished sweep (stamp_ns, points Nx5 [x y z intensity time], ring) once
    a full rotation has been collected, else None.

    CARLA returns only the part of the rotation swept during each sim tick.
    Each slice's points get time = middle of that tick, relative to the sweep
    start, so per-point time resolution is one CARLA tick (10 ms at 100 Hz).
    """

    def __init__(self, cfg, rng):
        b = cfg['beams']
        self.beams = beam_elevations(b)
        self.beam_tol = math.radians(b['tolerance_deg'])
        self.period_ns = int(round(1e9 / cfg['rotation_hz']))
        self.r_min = cfg.get('min_range_m', 0.0)
        self.r_max = cfg['max_range_m']
        self.fov = cfg.get('horizontal_fov_deg', [-180.0, 180.0])
        self.az_res = math.radians(cfg['azimuth_resolution_deg'])
        noise = cfg.get('noise') or {}
        self.accuracy = noise.get('range_accuracy', [])
        self.dropout = noise.get('dropout_rate', 0.0)
        self.intensity_scale = cfg.get('intensity_scale', 1.0)
        self.left_handed = cfg.get('input_left_handed', False)
        self.rng = rng

        self._slices = []          # (stamp_ns, Nx4 xyzi)
        self._start_ns = None      # sweep start (sim time)
        self._last_ns = None
        self.full_sweep_input = False  # set if each CARLA message already covers a full rotation

    def max_points(self):
        width = min(self.fov[1] - self.fov[0], 360.0)
        return len(self.beams) * round(math.radians(width) / self.az_res)

    def add(self, stamp_ns, xyzi):
        xyzi = np.asarray(xyzi, dtype=np.float64)
        if self.left_handed:
            xyzi = xyzi.copy()
            xyzi[:, 1] *= -1.0

        if self._last_ns is None:
            # First message: tick length unknown until the next one, hold it.
            self._last_ns = stamp_ns
            self._slices.append((stamp_ns, xyzi))
            return None

        tick_ns = stamp_ns - self._last_ns
        self._last_ns = stamp_ns
        if tick_ns <= 0:
            return None
        if tick_ns >= self.period_ns:
            self.full_sweep_input = True
        if self._start_ns is None:
            self._start_ns = self._slices[0][0] - tick_ns
        elif tick_ns > 2 * self.period_ns:
            # Gap in the bag: drop the partial sweep and start over.
            self._slices = []
            self._start_ns = stamp_ns - tick_ns

        self._slices.append((stamp_ns, xyzi))
        # Half a tick of slack: stamps are rounded to the ns, so ten 10 ms
        # ticks can add up to 1 ns short of a 100 ms rotation.
        if stamp_ns - self._start_ns < self.period_ns - tick_ns // 2:
            return None

        sweep = self._finish(tick_ns)
        self._slices = []
        self._start_ns = stamp_ns
        return sweep

    def _finish(self, tick_ns):
        parts = []
        for stamp_ns, xyzi in self._slices:
            t = (stamp_ns - tick_ns / 2 - self._start_ns) * 1e-9
            t = min(max(t, 0.0), self.period_ns * 1e-9)
            parts.append(np.hstack([xyzi, np.full((len(xyzi), 1), t)]))
        pts = np.vstack(parts) if parts else np.empty((0, 5))

        pts = range_crop(pts, self.r_min, self.r_max)
        pts = azimuth_crop(pts, *self.fov)
        pts, ring = snap_to_beams(pts, self.beams, self.beam_tol)
        pts, ring = thin_azimuth(pts, ring, self.az_res)
        if self.accuracy:
            sigma = range_stddev(ring, np.linalg.norm(pts[:, :3], axis=1), self.accuracy)
            pts = add_range_noise(pts, sigma, self.rng)
        if self.dropout > 0.0:
            keep = self.rng.random(len(pts)) >= self.dropout
            pts, ring = pts[keep], ring[keep]
        pts[:, 3] *= self.intensity_scale

        order = np.lexsort((pts[:, 4], ring))  # by ring, then time
        return self._start_ns, pts[order], ring[order]


# ---------- IMU ----------

class GaussMarkov:
    """First-order Gauss-Markov process: wanders with stddev sigma, correlation time tau."""

    def __init__(self, sigma, tau, rng):
        self.sigma = np.asarray(sigma, dtype=np.float64)
        self.tau = tau
        self.rng = rng
        self.x = rng.normal(0.0, 1.0, self.sigma.shape) * self.sigma  # start in steady state
        self._t = None

    def step(self, t):
        if self._t is not None and t > self._t and np.any(self.sigma > 0.0):
            a = math.exp(-(t - self._t) / self.tau)
            self.x = a * self.x + self.sigma * math.sqrt(1.0 - a * a) * self.rng.normal(
                0.0, 1.0, self.x.shape)
        self._t = t
        return self.x


def quat_mul(a, b):
    """Hamilton product of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


# Datasheet bias instability B is the Allan deviation floor / 0.664 (IEEE
# 952); a first-order Gauss-Markov process peaks at 0.618 sigma. So the
# Gauss-Markov sigma that reproduces the datasheet floor is B * 0.664 / 0.618.
BIAS_INSTABILITY_TO_GM_SIGMA = 0.664 / 0.618


class SensorErrors:
    """Turn-on bias + Gauss-Markov bias + white noise for one 3-axis sensor (accel or gyro)."""

    def __init__(self, cfg, rate, rng):
        if 'white_noise_stddev' in cfg:
            self.white = quantity(cfg['white_noise_stddev'])
        else:
            self.white = quantity(cfg.get('noise_density', 0.0)) * math.sqrt(rate)
        self.turn_on = rng.normal(0.0, quantity(cfg.get('turn_on_bias', 0.0)), 3)
        gm_sigma = quantity(cfg.get('bias_instability', 0.0)) * BIAS_INSTABILITY_TO_GM_SIGMA
        self.bias = GaussMarkov(np.full(3, gm_sigma),
                                quantity(cfg.get('bias_correlation_time', 1.0)), rng)
        self.cov = np.diag(np.full(3, self.white ** 2))
        self.rng = rng

    def apply(self, t, v):
        return v + self.turn_on + self.bias.step(t) + self.rng.normal(0.0, self.white, 3)


class ImuFilter:
    """
    Feed raw CARLA IMU samples in time order with apply(). It returns
    (accel, gyro, quat) with sensor errors added, or None if the sample is
    dropped to reach rate_hz. quat is None when the sensor has no
    orientation output (no `orientation` section) or the input had none.
    """

    def __init__(self, cfg, rng):
        rate = cfg['rate_hz']
        self.period = 1.0 / rate
        self.left_handed = cfg.get('input_left_handed', False)
        self.accel = SensorErrors(cfg.get('accel') or {}, rate, rng)
        self.gyro = SensorErrors(cfg.get('gyro') or {}, rate, rng)

        o = cfg.get('orientation')
        self.reports_orientation = o is not None
        if o is not None:
            rp = quantity(o.get('roll_pitch_stddev', 0.0))
            hd = quantity(o.get('heading_stddev', 0.0))
            self.att_err = GaussMarkov([rp, rp, hd], quantity(o.get('correlation_time', 1.0)), rng)
            self.att_cov = np.diag([rp ** 2, rp ** 2, hd ** 2])
        self._next = None

    def apply(self, t, acc, gyr, quat):
        if self._next is not None and t < self._next:
            return None
        self._next = t + 0.75 * self.period  # tolerate jitter, still drops 2x-rate input

        acc = np.asarray(acc, dtype=np.float64)
        gyr = np.asarray(gyr, dtype=np.float64)
        if quat is not None and self.reports_orientation:
            quat = np.asarray(quat, dtype=np.float64)
        else:
            quat = None
        if self.left_handed:
            # Mirror y: vectors flip y, axial vectors and rotations flip x and z.
            acc = acc * [1.0, -1.0, 1.0]
            gyr = gyr * [-1.0, 1.0, -1.0]
            if quat is not None:
                quat = quat * [-1.0, 1.0, -1.0, 1.0]

        acc = self.accel.apply(t, acc)
        gyr = self.gyro.apply(t, gyr)
        if quat is not None:
            quat = quat_mul(quat_from_rpy(*self.att_err.step(t)), quat)  # error in world frame
            quat /= np.linalg.norm(quat)
        return acc, gyr, quat


# ---------- bag I/O (needs ROS) ----------

def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def ns_to_stamp(ns):
    from builtin_interfaces.msg import Time
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def cloud_to_xyzi(msg):
    from sensor_msgs_py import point_cloud2
    names = [f.name for f in msg.fields]
    fields = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in names else [])
    arr = point_cloud2.read_points(msg, field_names=fields, skip_nans=True)
    xyzi = np.zeros((len(arr), 4))
    for i, name in enumerate(fields):
        xyzi[:, i] = arr[name]
    return xyzi


XYZIRT = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24,
})


def xyzirt_to_cloud(stamp_ns, frame_id, pts, ring):
    from sensor_msgs.msg import PointCloud2, PointField
    data = np.zeros(len(pts), dtype=XYZIRT)
    for i, name in enumerate(['x', 'y', 'z', 'intensity']):
        data[name] = pts[:, i]
    data['ring'] = ring
    data['time'] = pts[:, 4]

    msg = PointCloud2()
    msg.header.stamp = ns_to_stamp(stamp_ns)
    msg.header.frame_id = frame_id
    msg.height, msg.width = 1, len(pts)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
        PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = XYZIRT.itemsize
    msg.row_step = XYZIRT.itemsize * len(pts)
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


class LidarStream:
    """One configured lidar: raw topic in, filtered topic out."""

    msg_type = 'sensor_msgs/msg/PointCloud2'

    def __init__(self, cfg):
        self.cfg = cfg
        self.name = cfg.get('name', cfg['topics']['output'])
        self.input, self.output = cfg['topics']['input'], cfg['topics']['output']
        self.filter = LidarFilter(cfg, np.random.default_rng(cfg.get('seed', 0)))
        self.n_in = self.n_out = self.n_points = 0

    def process(self, data, t, writer):
        from rclpy.serialization import deserialize_message, serialize_message
        from sensor_msgs.msg import PointCloud2
        msg = deserialize_message(data, PointCloud2)
        self.n_in += 1
        sweep = self.filter.add(stamp_to_ns(msg.header.stamp), cloud_to_xyzi(msg))
        if sweep is None:
            return
        stamp_ns, pts, ring = sweep
        self.n_out += 1
        self.n_points += len(pts)
        out = xyzirt_to_cloud(stamp_ns, self.cfg['frame_id'], pts, ring)
        writer.write(self.output, serialize_message(out), t)

    def report(self):
        per_sweep = self.n_points / self.n_out if self.n_out else 0
        print(f'{self.name}: {self.n_in} CARLA msgs -> {self.n_out} sweeps on {self.output}, '
              f'{per_sweep:.0f} pts/sweep (sensor max {self.filter.max_points()}; fewer is '
              f'normal for open sky and the {self.filter.r_max} m range cap)')
        if self.filter.full_sweep_input:
            print(f'  warning: CARLA messages on {self.input} each cover a full rotation, so '
                  'per-point time is one value per sweep. Run CARLA with a faster fixed tick.')


class ImuStream:
    """One configured IMU: raw topic in, filtered topic out."""

    msg_type = 'sensor_msgs/msg/Imu'

    def __init__(self, cfg):
        self.cfg = cfg
        self.name = cfg.get('name', cfg['topics']['output'])
        self.input, self.output = cfg['topics']['input'], cfg['topics']['output']
        self.filter = ImuFilter(cfg, np.random.default_rng(cfg.get('seed', 0)))
        self.n_in = self.n_out = self.n_no_orientation = 0
        self.first = self.last = None

    def process(self, data, t, writer):
        from rclpy.serialization import deserialize_message, serialize_message
        from sensor_msgs.msg import Imu
        msg = deserialize_message(data, Imu)
        self.n_in += 1
        stamp = stamp_to_ns(msg.header.stamp) * 1e-9
        self.first = stamp if self.first is None else self.first
        self.last = stamp

        o = msg.orientation
        q = [o.x, o.y, o.z, o.w]
        has_orientation = msg.orientation_covariance[0] != -1.0 and np.linalg.norm(q) > 0.5
        if self.filter.reports_orientation and not has_orientation:
            self.n_no_orientation += 1
        a, g = msg.linear_acceleration, msg.angular_velocity
        res = self.filter.apply(stamp, [a.x, a.y, a.z], [g.x, g.y, g.z],
                                q if has_orientation else None)
        if res is None:
            return
        acc, gyr, quat = res
        msg.header.frame_id = self.cfg['frame_id']
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyr
        msg.linear_acceleration_covariance = self.filter.accel.cov.flatten().tolist()
        msg.angular_velocity_covariance = self.filter.gyro.cov.flatten().tolist()
        if quat is not None:
            o.x, o.y, o.z, o.w = quat
            msg.orientation_covariance = self.filter.att_cov.flatten().tolist()
        else:
            o.x, o.y, o.z, o.w = 0.0, 0.0, 0.0, 1.0
            msg.orientation_covariance = [-1.0] + [0.0] * 8  # ROS: orientation unknown
        self.n_out += 1
        writer.write(self.output, serialize_message(msg), t)

    def report(self):
        span = (self.last - self.first) if self.first is not None else 0.0
        rate_in = (self.n_in - 1) / span if span > 0 else 0.0
        rate = self.cfg['rate_hz']
        print(f'{self.name}: {self.n_in} msgs at {rate_in:.1f} Hz -> {self.n_out} msgs on '
              f'{self.output} at target {rate} Hz')
        if rate_in and rate_in < 0.95 * rate:
            print(f'  warning: input on {self.input} is slower than {rate} Hz; output is too')
        if self.n_no_orientation:
            print(f'  warning: {self.n_no_orientation} msgs had no orientation; output marks '
                  'orientation unknown (covariance[0] = -1)')


def topic_metadata(name, type_name):
    import rosbag2_py
    try:
        return rosbag2_py.TopicMetadata(id=0, name=name, type=type_name, serialization_format='cdr')
    except TypeError:  # rosbag2 before Jazzy has no id
        return rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format='cdr')


GROUND_TRUTH_TF = '/ground_truth/tf'
GROUND_TRUTH_ODOM = '/ground_truth/odometry'


def open_reader(uri, topics=None):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def first_clock_ns(uri):
    """Sim time of the bag's first /clock message, or None if it has none."""
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    reader = open_reader(uri, ['/clock'])
    if not reader.has_next():
        return None
    _, data, _ = reader.read_next()
    return stamp_to_ns(deserialize_message(data, Clock).clock)


def rename_frame(tf_msg, old, new):
    for tf in tf_msg.transforms:
        if tf.header.frame_id == old:
            tf.header.frame_id = new
        if tf.child_frame_id == old:
            tf.child_frame_id = new
    return tf_msg


def filter_bag(in_uri, out_uri, streams, storage_id, keep_raw,
               vehicle_frame, base_frame, gt_odom):
    """
    Write a bag LIO-SAM can't tell from a kart recording:
      - sensor outputs from `streams`
      - /tf_static with the CARLA vehicle frame renamed to base_frame
      - CARLA's /tf (ground truth) on GROUND_TRUTH_TF, and gt_odom on
        GROUND_TRUTH_ODOM, so ground truth never fights LIO-SAM's own
        map -> odom -> base_link
      - every message stamped with sim time from /clock, and /clock dropped,
        so it plays with `ros2 bag play --clock` like a real bag
    Any other topic is copied through.
    """
    from rclpy.serialization import deserialize_message, serialize_message
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage

    by_input = {s.input: s for s in streams}
    outputs = {s.output for s in streams} | {GROUND_TRUTH_TF, GROUND_TRUTH_ODOM}
    if len(by_input) != len(streams) or len(outputs) != len(streams) + 2:
        raise SystemExit('each sensor config needs its own topics.input and topics.output')

    reader = open_reader(in_uri)
    topics = {m.name: m for m in reader.get_all_topics_and_types()}
    for s in streams:
        if s.input not in topics:
            raise SystemExit(f'{s.name}: {s.input} is not in {in_uri}. Topics: {sorted(topics)}')
    clock_ns = first_clock_ns(in_uri)
    if clock_ns is None:
        print('warning: no /clock in the input; keeping recording timestamps')

    import rosbag2_py
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out_uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for s in streams:
        writer.create_topic(topic_metadata(s.output, s.msg_type))
    if '/tf_static' in topics:
        writer.create_topic(topics['/tf_static'])  # keeps its latched QoS
    if '/tf' in topics:
        writer.create_topic(topic_metadata(GROUND_TRUTH_TF, 'tf2_msgs/msg/TFMessage'))
    if gt_odom in topics:
        writer.create_topic(topic_metadata(GROUND_TRUTH_ODOM, 'nav_msgs/msg/Odometry'))
    handled = {'/clock', '/tf', '/tf_static', gt_odom}
    passthrough = {n for n in topics
                   if n not in outputs and n not in handled and (keep_raw or n not in by_input)}
    for name in sorted(passthrough):
        writer.create_topic(topics[name])
    for name in sorted(set(topics) & outputs):
        print(f'warning: input already has {name}; dropping the input copy')

    n_gt = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/clock':
            clock_ns = stamp_to_ns(deserialize_message(data, Clock).clock)
            continue
        if clock_ns is not None:
            t = clock_ns  # messages before the first tick get its time
        if topic in passthrough:
            writer.write(topic, data, t)
        if topic in by_input:
            by_input[topic].process(data, t, writer)
        elif topic in ('/tf', '/tf_static'):
            msg = rename_frame(deserialize_message(data, TFMessage), vehicle_frame, base_frame)
            writer.write(GROUND_TRUTH_TF if topic == '/tf' else '/tf_static',
                         serialize_message(msg), t)
            n_gt += topic == '/tf'
        elif topic == gt_odom:
            msg = deserialize_message(data, Odometry)
            if msg.child_frame_id == vehicle_frame:
                msg.child_frame_id = base_frame
            writer.write(GROUND_TRUTH_ODOM, serialize_message(msg), t)

    del writer  # closes the bag
    for s in streams:
        s.report()
    print(f'ground truth: {n_gt} /tf msgs -> {GROUND_TRUTH_TF}, {gt_odom} -> {GROUND_TRUTH_ODOM}; '
          f'{vehicle_frame} renamed to {base_frame}; /clock dropped (play with --clock)')


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('input', help='raw CARLA bag (directory)')
    p.add_argument('output', help='filtered bag to create (directory, must not exist)')
    p.add_argument('--lidar', action='append', default=[],
                   help='lidar config; repeat for several lidars (default: config/airy_lite.yaml)')
    p.add_argument('--imu', action='append', default=[],
                   help='IMU config; repeat for several IMUs (default: config/cv7_ahrs.yaml)')
    p.add_argument('--storage', default='mcap', help='output storage plugin (default: mcap)')
    p.add_argument('--keep-raw', action='store_true',
                   help='also copy the raw CARLA lidar/IMU topics into the output')
    p.add_argument('--vehicle-frame', default='carla_base_link',
                   help='CARLA vehicle frame in the raw bag (default: carla_base_link)')
    p.add_argument('--base-frame', default='base_link',
                   help='robot frame it becomes in the output (default: base_link)')
    p.add_argument('--ground-truth-odom', default='/carla/hero/odometry',
                   help=f'raw ground-truth odometry topic, written as {GROUND_TRUTH_ODOM}')
    args = p.parse_args()
    if not args.lidar and not args.imu:
        args.lidar, args.imu = DEFAULT_LIDARS, DEFAULT_IMUS

    streams = [LidarStream(load(path)) for path in args.lidar]
    streams += [ImuStream(load(path)) for path in args.imu]
    filter_bag(args.input, args.output, streams, args.storage, args.keep_raw,
               args.vehicle_frame, args.base_frame, args.ground_truth_odom)


if __name__ == '__main__':
    main()
