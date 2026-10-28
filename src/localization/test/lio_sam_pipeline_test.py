"""LIO-SAM (+ GTSAM) end-to-end test.

Publishes a synthetic VLP-16-style scan (with the ring/time fields LIO-SAM
needs) and a stationary IMU stream into the four LIO-SAM nodes, and checks that
  * lio_sam/mapping/odometry  arrives (mapOptimization: GTSAM ISAM2), and
  * odometry/imu_incremental  arrives (imuPreintegration: GTSAM IMU preintegration),
with the pose staying near the origin, since nothing moves. Run through
sanity_check.sh, which starts the LIO-SAM nodes.
"""
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

N_SCAN, HORIZON_SCAN = 16, 1800        # VLP-16: rings at -15..+15 deg, 0.2 deg columns
SENSOR_HEIGHT = 1.8
SCAN_PERIOD = 0.1
GRAVITY = 9.80511                      # must match LIO-SAM's imuGravity
MAX_DRIFT_M = 0.5

POINT_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                        'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                        'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})


def make_boxes(rng):
    boxes = []
    while len(boxes) < 25:
        c = rng.uniform(-40, 40, 2)
        if np.linalg.norm(c) < 8:      # keep the sensor outside every box
            continue
        half = np.r_[rng.uniform(1.5, 6, 2), 0]
        boxes.append((np.r_[c - half[:2], 0.0], np.r_[c + half[:2], rng.uniform(3, 12)]))
    return boxes


def ray_cast(boxes):
    """Ranges for a VLP-16 at SENSOR_HEIGHT over flat ground + boxes."""
    elev = np.radians(np.linspace(-15, 15, N_SCAN))
    azim = np.linspace(-math.pi, math.pi, HORIZON_SCAN, endpoint=False)
    el, az = np.meshgrid(elev, azim, indexing='ij')         # (ring, column)
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1)
    o = np.array([0, 0, SENSOR_HEIGHT])
    t = np.full(el.shape, np.inf)
    down = d[..., 2] < -1e-6
    t[down] = -SENSOR_HEIGHT / d[..., 2][down]               # ground plane z = 0
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / d
        for lo, hi in boxes:                                 # slab test per box
            t1, t2 = (lo - o) * inv, (hi - o) * inv
            tn = np.nanmax(np.minimum(t1, t2), -1)
            tf = np.nanmin(np.maximum(t1, t2), -1)
            hit = (tf >= np.maximum(tn, 0)) & (tn > 0)
            t = np.where(hit & (tn < t), tn, t)
    t[t > 80] = np.inf
    return d, t


def build_scan(rng):
    d, t = ray_cast(make_boxes(rng))
    ring = np.broadcast_to(np.arange(N_SCAN)[:, None], t.shape)
    col = np.broadcast_to(np.arange(HORIZON_SCAN)[None, :], t.shape)
    # Order points by firing time (column-major), as a spinning lidar does,
    # so the last point carries the end-of-scan time LIO-SAM reads.
    order = np.argsort((col * N_SCAN + ring).ravel())
    valid = np.isfinite(t).ravel()[order]
    idx = order[valid]
    pts = d.reshape(-1, 3)[idx] * t.ravel()[idx, None]  # in the lidar frame
    arr = np.zeros(len(idx), dtype=POINT_DTYPE)
    arr['x'], arr['y'], arr['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
    arr['intensity'] = 100.0
    arr['ring'] = ring.ravel()[idx]
    arr['time'] = (col.ravel()[idx] / HORIZON_SCAN) * SCAN_PERIOD
    return arr


class Tester(Node):
    def __init__(self, scan):
        super().__init__('lio_sam_sanity_tester')
        self.scan = scan
        self.rng = np.random.default_rng(2)
        self.cloud_pub = self.create_publisher(PointCloud2, '/points', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.map_odom, self.imu_odom = [], []
        # Best effort: LIO-SAM publishes its odometry best-effort, and a
        # reliable subscriber would silently receive nothing.
        self.create_subscription(Odometry, 'lio_sam/mapping/odometry',
                                 lambda m: self.map_odom.append(m), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odometry/imu_incremental',
                                 lambda m: self.imu_odom.append(m), qos_profile_sensor_data)
        self.create_timer(0.005, self.publish_imu)        # 200 Hz
        self.create_timer(SCAN_PERIOD, self.publish_scan)  # 10 Hz

    def publish_imu(self):
        # Stationary and level, plus sensor-like noise. The noise matters:
        # with bit-exact level data LIO-SAM's roll/pitch slerp (tf2 acos of a
        # dot product rounding to just over 1) returns NaN and mapOptimization
        # aborts. Real IMUs are never that clean.
        n = self.rng.normal(size=8)
        roll, pitch = np.radians(0.05) * n[0], np.radians(0.05) * n[1]
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        msg.orientation.x, msg.orientation.y = sr * cp, cr * sp
        msg.orientation.z, msg.orientation.w = -sr * sp, cr * cp
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = 0.002 * n[2:5]
        msg.linear_acceleration.x, msg.linear_acceleration.y = 0.02 * n[5:7]
        msg.linear_acceleration.z = GRAVITY + 0.02 * n[7]
        self.imu_pub.publish(msg)

    def publish_scan(self):
        # Stamp = scan start, one period back, so IMU data already covers the
        # whole sweep (imageProjection drops scans the IMU doesn't cover).
        now = self.get_clock().now().nanoseconds - int(2 * SCAN_PERIOD * 1e9)
        msg = PointCloud2()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(now, 1_000_000_000)
        msg.header.frame_id = 'lidar_link'
        msg.height, msg.width = 1, len(self.scan)
        types = {'<f4': PointField.FLOAT32, '<u2': PointField.UINT16}
        msg.fields = [PointField(name=n, offset=POINT_DTYPE.fields[n][1],
                                 datatype=types[POINT_DTYPE.fields[n][0].str], count=1)
                      for n in POINT_DTYPE.names]
        msg.point_step, msg.row_step = POINT_DTYPE.itemsize, POINT_DTYPE.itemsize * len(self.scan)
        msg.is_dense = True
        msg.data = self.scan.tobytes()
        self.cloud_pub.publish(msg)


def main():
    rclpy.init()
    node = Tester(build_scan(np.random.default_rng(1)))
    end = time.time() + 30
    while time.time() < end and (len(node.map_odom) < 5 or len(node.imu_odom) < 20):
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'scan has {len(node.scan)} points; received {len(node.map_odom)} '
          f'lio_sam/mapping/odometry and {len(node.imu_odom)} odometry/imu_incremental')
    if not node.map_odom or not node.imu_odom:
        print('missing LIO-SAM output (see node logs above)')
        sys.exit(1)
    p = node.map_odom[-1].pose.pose.position
    drift = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
    print(f'final mapping pose ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), drift {drift:.3f} m (limit {MAX_DRIFT_M} m)')
    sys.exit(0 if drift < MAX_DRIFT_M else 1)


if __name__ == '__main__':
    main()
