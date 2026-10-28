"""Autoware NDT accuracy test for ap1_localization.

Builds a synthetic city map, publishes a lidar scan taken from a known pose,
starts the localizer ~1 m / 3 deg off, and checks /ndt_pose converges to the
truth. Run through sanity_check.sh, which starts localization_node.

  ndt_synthetic_test.py write-map MAP.pcd   # writes map, prints initial_pose param
  ndt_synthetic_test.py run                 # publishes scans, checks /ndt_pose
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

TRUE_X, TRUE_Y, TRUE_YAW = 20.0, 8.0, 0.30
INITIAL_POSE = [20.8, 7.4, 0.3, 0.25]  # [x, y, z, yaw]: ~1 m and ~3 deg off
MAX_POS_ERR_M, MAX_YAW_ERR_DEG = 0.2, 1.0


def make_map(rng):
    pts = []
    g = np.arange(-60, 60, 0.7)
    gx, gy = np.meshgrid(g, g)
    pts.append(np.c_[gx.ravel(), gy.ravel(), 0.05 * np.sin(gx.ravel() / 7.0)])
    for _ in range(30):  # box "buildings"
        cx, cy = rng.uniform(-55, 55, 2)
        w, d, h = rng.uniform(4, 14), rng.uniform(4, 14), rng.uniform(4, 15)
        walls = [(cx - w / 2, cx + w / 2, cy - d / 2, cy - d / 2), (cx - w / 2, cx + w / 2, cy + d / 2, cy + d / 2),
                 (cx - w / 2, cx - w / 2, cy - d / 2, cy + d / 2), (cx + w / 2, cx + w / 2, cy - d / 2, cy + d / 2)]
        for (x0, x1, y0, y1) in walls:
            n = int(max(abs(x1 - x0), abs(y1 - y0)) / 0.4) + 1
            s = np.linspace(0, 1, n)
            for z in np.arange(0, h, 0.4):
                pts.append(np.c_[x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, np.full(n, z)])
    for _ in range(80):  # poles
        px, py = rng.uniform(-55, 55, 2)
        z = np.arange(0, 6, 0.2)
        pts.append(np.c_[np.full_like(z, px), np.full_like(z, py), z])
    return np.vstack(pts).astype(np.float32)


def write_pcd(path, pts):
    with open(path, 'w') as f:
        f.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n')
        np.savetxt(f, pts, fmt='%.3f')


def scan_from(map_pts, rng):
    c, s = math.cos(TRUE_YAW), math.sin(TRUE_YAW)
    rel = map_pts - np.array([TRUE_X, TRUE_Y, 0.0], dtype=np.float32)
    rel = rel[np.linalg.norm(rel[:, :2], axis=1) < 60.0]
    # map -> base_link: R^T * (p - t)
    bx = c * rel[:, 0] + s * rel[:, 1]
    by = -s * rel[:, 0] + c * rel[:, 1]
    scan = np.c_[bx, by, rel[:, 2]].astype(np.float32)
    scan = scan[rng.random(len(scan)) < 0.5]  # a scan only sees part of the map
    return scan + rng.normal(0, 0.03, scan.shape).astype(np.float32)


class Tester(Node):
    def __init__(self, scan):
        super().__init__('ndt_sanity_tester')
        self.scan = scan
        self.pub = self.create_publisher(PointCloud2, '/points', 10)
        self.create_subscription(PoseStamped, '/ndt_pose', self.on_pose, 10)
        self.poses = []
        self.create_timer(0.2, self.publish_scan)

    def publish_scan(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.height, msg.width = 1, len(self.scan)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.point_step, msg.row_step = 12, 12 * len(self.scan)
        msg.is_dense = True
        msg.data = self.scan.tobytes()
        self.pub.publish(msg)

    def on_pose(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.poses.append((msg.pose.position.x, msg.pose.position.y, yaw))


def main():
    rng = np.random.default_rng(0)
    pts = make_map(rng)  # same seed in both modes, so the scan matches the map file
    if sys.argv[1] == 'write-map':
        write_pcd(sys.argv[2], pts)
        print('[' + ', '.join(str(v) for v in INITIAL_POSE) + ']')
        return

    rclpy.init()
    node = Tester(scan_from(pts, rng))
    end = time.time() + 20
    while time.time() < end and len(node.poses) < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.poses:
        print('no /ndt_pose received (node not running, or every match was rejected)')
        sys.exit(1)
    x, y, yaw = node.poses[-1]
    pos_err = math.hypot(x - TRUE_X, y - TRUE_Y)
    yaw_err = abs(math.degrees(yaw - TRUE_YAW))
    print(f'{len(node.poses)} poses, final error {pos_err:.3f} m / {yaw_err:.2f} deg '
          f'(limit {MAX_POS_ERR_M} m / {MAX_YAW_ERR_DEG} deg)')
    sys.exit(0 if pos_err < MAX_POS_ERR_M and yaw_err < MAX_YAW_ERR_DEG else 1)


if __name__ == '__main__':
    main()
