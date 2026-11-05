#!/usr/bin/env python3
"""
Record a raw CARLA test-drive bag for filter_bag.py.

Connects to a running CARLA server, loads the map (Town01 by default), spawns
the ego with the LiDAR and IMU described by the filter's sensor configs at the
poses in the mounts config, and drives a loop at 10 km/h that covers every
road (or, with --short, only the smallest closed loop through the spawn point)
and comes back to the start (so LIO-SAM's loop closure triggers). Every
sim tick is written straight into a rosbag2 bag, so no LiDAR slice is lost.

    source /opt/ros/jazzy/setup.bash
    python3 -m pip install carla==0.9.16 --break-system-packages  # match the server version, skip if already installed
    python3 simulation/record_bag.py                              # whole map
    python3 simulation/record_bag.py --short                      # smallest closed loop
    python3 simulation/record_bag.py --host my-pc --filter

See simulation/README.md ("Recording a raw bag").
"""
import argparse
import heapq
import math
import os
import queue
import subprocess
import sys
from dataclasses import dataclass
from datetime import date

import numpy as np
import yaml

from filter_bag import (CONFIG_DIR, DEFAULT_IMUS, DEFAULT_LIDARS, ConfigError, beam_elevations,
                        check_imu_config, check_lidar_config, load, ns_to_stamp, quat_mul,
                        quat_from_rpy, ring_max_range, topic_metadata)

HERE = os.path.dirname(os.path.abspath(__file__))

FIXED_DELTA_S = 0.01      # synchronous, 100 Hz ticks
LIDAR_RANGE_M = 60.0      # raw range; the filter crops to the config's dark-target ranges
# At exactly the config's resolution CARLA's rays land on the filter's azimuth
# bin edges and some bins come out empty; twice the rays puts one mid-bin.
LIDAR_OVERSAMPLE = 2

SPEED_KMH = 10.0          # the kart's cap
SETTLE_S = 1.0            # not recorded: the car drops onto its wheels
STILL_S = 2.0             # recorded standing still at both ends (IMU init for LIO-SAM)
LOOKAHEAD_M = 4.5         # pure-pursuit lookahead
MAX_OFF_ROUTE_M = 5.0
STUCK_S = 15.0
SENSOR_TIMEOUT_S = 20.0

MAP_FRAME = 'carla_map'
VEHICLE_FRAME = 'carla_base_link'     # filter_bag.py --vehicle-frame default
ODOM_TOPIC = '/carla/hero/odometry'   # filter_bag.py --ground-truth-odom default
DEFAULT_MOUNTS = os.path.join(CONFIG_DIR, 'mounts.yaml')


# ---------- CARLA <-> ROS frames ----------
# CARLA is left-handed (x forward, y right, z up, degrees); ROS is x forward,
# y left, z up. Same conversions as the CARLA ROS bridge.

def ros_pose(transform):
    """carla.Transform -> (xyz, quaternion xyzw) in ROS convention."""
    loc, rot = transform.location, transform.rotation
    q = quat_from_rpy(math.radians(rot.roll), -math.radians(rot.pitch), -math.radians(rot.yaw))
    return np.array([loc.x, -loc.y, loc.z]), q


def carla_mount(xyz, rpy_deg):
    """A mounts-config pose (ROS axes, degrees) -> carla.Transform for attaching a sensor."""
    import carla
    return carla.Transform(carla.Location(x=xyz[0], y=-xyz[1], z=xyz[2]),
                           carla.Rotation(roll=rpy_deg[0], pitch=-rpy_deg[1], yaw=-rpy_deg[2]))


def rot_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def body_rate(q_prev, q, dt):
    """Angular velocity in the body frame (rad/s) that turns q_prev into q in dt."""
    dq = quat_mul(np.array([-q_prev[0], -q_prev[1], -q_prev[2], q_prev[3]]), q)
    return 2.0 * (-dq if dq[3] < 0 else dq)[:3] / dt


# ---------- sensors ----------

def load_configs(lidar_path, imu_path, mounts_path):
    try:
        lidar, imu = load(lidar_path), load(imu_path)
        with open(mounts_path) as f:
            mounts = yaml.safe_load(f) or {}
    except OSError as e:
        raise SystemExit(e) from None
    for path, cfg, check in ((lidar_path, lidar, check_lidar_config), (imu_path, imu, check_imu_config)):
        try:
            check(cfg)
        except ConfigError as e:
            raise SystemExit(f'{path}: {e}') from None
    for cfg in (lidar, imu):
        m = mounts.get(cfg['frame_id'])
        if not (isinstance(m, dict) and len(m.get('xyz') or []) == 3 and len(m.get('rpy') or []) == 3):
            raise SystemExit(f'{mounts_path}: needs "{cfg["frame_id"]}" with xyz and rpy '
                             '(3 numbers each)')
    if 1.0 / imu['rate_hz'] < FIXED_DELTA_S - 1e-9:
        raise SystemExit(f'{imu_path}: rate_hz {imu["rate_hz"]} is faster than CARLA\'s '
                         f'{1 / FIXED_DELTA_S:g} Hz ticks')
    return lidar, imu, mounts


def lidar_settings(cfg):
    """CARLA sensor.lidar.ray_cast attributes that copy the config's beams exactly."""
    elev = np.degrees(beam_elevations(cfg['beams']))
    if len(elev) > 2 and np.ptp(np.diff(np.sort(elev))) > 1e-6:
        raise SystemExit('CARLA\'s lidar only has evenly spaced beams, so it can\'t match '
                         'beams.elevations_deg in this config')
    pps = LIDAR_OVERSAMPLE * len(elev) * 360.0 / cfg['azimuth_resolution_deg'] * cfg['rotation_hz']
    return {
        'channels': len(elev),
        'upper_fov': float(elev.max()),
        'lower_fov': float(elev.min()),
        'rotation_frequency': cfg['rotation_hz'],
        'points_per_second': int(round(pps)),
        'range': max(LIDAR_RANGE_M, float(ring_max_range(cfg['max_range_m'], len(elev)).max())),
        'horizontal_fov': 360.0,
        'sensor_tick': 0.0,            # every tick: one slice per 10 ms
        # No CARLA noise or drops; the filter adds them from the datasheet.
        'noise_stddev': 0.0,
        'dropoff_general_rate': 0.0,
        'dropoff_intensity_limit': 1.0,    # CARLA also drops low-intensity points otherwise
        'dropoff_zero_intensity': 0.0,
    }


def make_blueprint(bp_lib, name, settings):
    bp = bp_lib.find(name)
    for key, value in settings.items():
        bp.set_attribute(key, str(value))
    for attr in bp:  # every noise source off, whatever this CARLA version has
        if attr.id.startswith('noise_') and attr.id != 'noise_seed':
            bp.set_attribute(attr.id, '0')
    return bp


# ---------- route ----------

@dataclass
class Edge:
    """One lane from a junction to the next, as dense (x, y) points in CARLA coordinates."""
    u: int
    v: int
    pts: np.ndarray
    road: tuple          # (road_id, section_id): covering one lane covers the road
    lane: int
    junction: bool


def length(pts):
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def map_edges(carla_map):
    """The map's lane graph from CARLA's topology; nodes are merged by position."""
    nodes = []

    def node(loc):
        for i, (x, y) in enumerate(nodes):
            if math.hypot(x - loc.x, y - loc.y) < 0.5:
                return i
        nodes.append((loc.x, loc.y))
        return len(nodes) - 1

    edges = []
    for a, b in carla_map.get_topology():
        pts = [(w.transform.location.x, w.transform.location.y)
               for w in [a] + a.next_until_lane_end(1.0)]
        end = b.transform.location
        if math.hypot(pts[-1][0] - end.x, pts[-1][1] - end.y) > 0.05:
            pts.append((end.x, end.y))
        edges.append(Edge(node(a.transform.location), node(end), np.array(pts),
                          (a.road_id, a.section_id), a.lane_id, a.is_junction))
    return edges


def dijkstra(edges, src):
    """Shortest distances from src and the edge used to reach each node."""
    adj = {}
    for i, e in enumerate(edges):
        adj.setdefault(e.u, []).append(i)
    dist, via, heap = {src: 0.0}, {}, [(0.0, src)]
    while heap:
        d, n = heapq.heappop(heap)
        if d > dist[n]:
            continue
        for i in adj.get(n, []):
            m = edges[i].v
            nd = d + length(edges[i].pts)
            if nd < dist.get(m, math.inf):
                dist[m], via[m] = nd, i
                heapq.heappush(heap, (nd, m))
    return dist, via


def path_to(edges, via, src, dst):
    path = []
    while dst != src:
        path.append(via[dst])
        dst = edges[via[dst]].u
    return path[::-1]


def plan_route(edges, start_edge, start_idx, short=False):
    """
    A loop from point start_idx of start_edge back to the same point. Greedily
    drives to the nearest road not yet driven until all are covered, then heads
    home. With short, it adds no roads and just heads home.
    Returns (points Nx2, roads covered, roads total).
    """
    e0 = edges[start_edge]
    home = e0.u
    roads = {e.road for e in edges if not e.junction}

    pieces = [e0.pts[start_idx:]]
    covered = {e0.road}
    cur = e0.v

    def drive(path):
        for i in path:
            pieces.append(edges[i].pts)
            if not edges[i].junction:
                covered.add(edges[i].road)

    while covered != roads and not short:
        dist, via = dijkstra(edges, cur)
        pick = min(((dist[e.u] + length(e.pts), i) for i, e in enumerate(edges)
                    if not e.junction and e.road not in covered and e.u in dist),
                   default=(0, None))[1]
        if pick is None:
            break
        drive(path_to(edges, via, cur, edges[pick].u) + [pick])
        cur = edges[pick].v

    dist, via = dijkstra(edges, cur)
    if home not in dist:
        raise SystemExit('the route can\'t get back to the start; try another --spawn-point')
    drive(path_to(edges, via, cur, home))
    pieces.append(e0.pts[:start_idx + 1])

    pts = [pieces[0]]
    for p in pieces[1:]:
        pts.append(p[1:] if len(p) and np.hypot(*(p[0] - pts[-1][-1])) < 0.1 else p)
    return np.vstack(pts), len(covered), len(roads)


def nearest_edge(edges, x, y, road_id, lane_id):
    """(edge index, point index) on the lane closest to (x, y)."""
    best = (math.inf, None, None)
    for i, e in enumerate(edges):
        if e.junction or (e.road[0], e.lane) != (road_id, lane_id):
            continue
        d = np.hypot(e.pts[:, 0] - x, e.pts[:, 1] - y)
        if d.min() < best[0]:
            best = (d.min(), i, int(d.argmin()))
    return best[1], best[2]


class RouteFollower:
    """Pure pursuit for steering, PI for speed. Works in CARLA coordinates."""

    KP, KI = 0.6, 0.15
    MAX_THROTTLE = 0.75
    MAX_STEER_STEP = 0.03     # per tick
    ACCEL = 0.8               # m/s^2, how fast the speed target ramps up (~3.5 s to 10 km/h)
    STOP_DECEL = 1.0          # m/s^2, slowing for the end of the route

    def __init__(self, pts, speed, wheelbase, max_steer_deg):
        self.pts = pts
        self.s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        self.speed, self.wheelbase, self.max_steer = speed, wheelbase, max_steer_deg
        self.i = 0
        self.integral = 0.0
        self.steer = 0.0
        self.v_ref = 0.0

    @property
    def remaining(self):
        return self.s[-1] - self.s[self.i]

    def step(self, x, y, yaw_deg, v, dt):
        """(throttle, steer, brake) for this tick; raises RuntimeError if off the route."""
        # Search forward only, so overlapping parts of the route can't be confused.
        ahead = slice(self.i, min(self.i + 40, len(self.pts)))
        d = np.hypot(self.pts[ahead, 0] - x, self.pts[ahead, 1] - y)
        self.i += int(d.argmin())
        if d.min() > MAX_OFF_ROUTE_M:
            raise RuntimeError(f'{d.min():.1f} m off the route at ({x:.1f}, {y:.1f})')

        k = min(int(np.searchsorted(self.s, self.s[self.i] + LOOKAHEAD_M)), len(self.pts) - 1)
        dx, dy = self.pts[k, 0] - x, self.pts[k, 1] - y
        yaw = math.radians(yaw_deg)
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy      # + = right (CARLA)
        alpha = math.atan2(ly, lx)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), max(math.hypot(lx, ly), 1.0))
        target = float(np.clip(math.degrees(delta) / self.max_steer, -1.0, 1.0))
        self.steer += float(np.clip(target - self.steer, -self.MAX_STEER_STEP, self.MAX_STEER_STEP))

        if self.remaining < 1.0:
            return 0.0, self.steer, 1.0
        # The speed target ramps up at ACCEL rather than jumping to the cruise speed.
        target = min(self.speed, math.sqrt(2.0 * self.STOP_DECEL * self.remaining))
        self.v_ref = min(self.v_ref + self.ACCEL * dt, target)
        err = self.v_ref - v
        if abs(err) < 0.5:  # integrate only near the target, so the run-up can't wind it up
            self.integral = float(np.clip(self.integral + err * dt, -1.0, 1.0))
        u = self.KP * err + self.KI * self.integral
        if v >= self.speed:  # the cap is a cap: never push past it
            u = min(u, 0.0)
        return (min(u, self.MAX_THROTTLE), self.steer, 0.0) if u >= 0 else (0.0, self.steer, min(-u, 1.0))


def vehicle_geometry(vehicle):
    """(wheelbase m, max steer deg) from the vehicle's physics."""
    wheels = vehicle.get_physics_control().wheels
    wb = wheels[0].position.distance(wheels[2].position)
    return (wb / 100.0 if wb > 20.0 else wb), wheels[0].max_steer_angle   # older CARLA: cm


# ---------- bag ----------

def latched_metadata(name, type_name):
    """Topic metadata with transient_local QoS, so /tf_static latches on playback."""
    import rosbag2_py
    qos_cls = getattr(rosbag2_py, 'QoS', None) or rosbag2_py._storage.QoS  # Jazzy has it only here
    qos = qos_cls(1).reliable().transient_local()
    return rosbag2_py.TopicMetadata(id=0, name=name, type=type_name, serialization_format='cdr',
                                    offered_qos_profiles=[qos])


class BagWriter:
    """Builds the ROS messages for one tick and writes them, stamped with sim time."""

    def __init__(self, uri, lidar_cfg, imu_cfg):
        import rosbag2_py
        self.lidar_topic, self.lidar_frame = lidar_cfg['topics']['input'], lidar_cfg['frame_id']
        self.imu_topic, self.imu_frame = imu_cfg['topics']['input'], imu_cfg['frame_id']
        types = {
            '/clock': 'rosgraph_msgs/msg/Clock',
            self.lidar_topic: 'sensor_msgs/msg/PointCloud2',
            self.imu_topic: 'sensor_msgs/msg/Imu',
            ODOM_TOPIC: 'nav_msgs/msg/Odometry',
            '/tf': 'tf2_msgs/msg/TFMessage',
            '/tf_static': 'tf2_msgs/msg/TFMessage',
        }
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                         rosbag2_py.ConverterOptions('cdr', 'cdr'))
        for name, type_name in types.items():
            meta = latched_metadata if name == '/tf_static' else topic_metadata
            self.writer.create_topic(meta(name, type_name))
        self.first_ns = None
        self.prev_q = None

    def _write(self, topic, msg, t_ns):
        from rclpy.serialization import serialize_message
        self.writer.write(topic, serialize_message(msg), t_ns)

    @staticmethod
    def _transform(stamp, parent, child, xyz, q):
        from geometry_msgs.msg import TransformStamped
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = map(float, q)
        return tf

    def write_static(self, t_ns, mounts):
        from tf2_msgs.msg import TFMessage
        stamp = ns_to_stamp(t_ns)
        tfs = [self._transform(stamp, VEHICLE_FRAME, frame, mounts[frame]['xyz'],
                               quat_from_rpy(*np.radians(mounts[frame]['rpy'])))
               for frame in (self.lidar_frame, self.imu_frame)]
        self._write('/tf_static', TFMessage(transforms=tfs), t_ns)

    def write_tick(self, t_ns, ego, lidar, imu, dt):
        """ego: carla.ActorSnapshot; lidar/imu: this tick's CARLA measurements."""
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2, PointField
        from tf2_msgs.msg import TFMessage
        stamp = ns_to_stamp(t_ns)
        self.first_ns = t_ns if self.first_ns is None else self.first_ns
        self._write('/clock', Clock(clock=stamp), t_ns)

        # Ground truth: carla_map -> carla_base_link, twist in the vehicle frame.
        xyz, q = ros_pose(ego.get_transform())
        self._write('/tf', TFMessage(transforms=[
            self._transform(stamp, MAP_FRAME, VEHICLE_FRAME, xyz, q)]), t_ns)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, MAP_FRAME, VEHICLE_FRAME
        p, o = odom.pose.pose.position, odom.pose.pose.orientation
        p.x, p.y, p.z = map(float, xyz)
        o.x, o.y, o.z, o.w = map(float, q)
        vel = ego.get_velocity()
        lin = rot_matrix(q).T @ np.array([vel.x, -vel.y, vel.z])
        ang = body_rate(self.prev_q, q, dt) if self.prev_q is not None else np.zeros(3)
        self.prev_q = q
        tw = odom.twist.twist
        tw.linear.x, tw.linear.y, tw.linear.z = map(float, lin)
        tw.angular.x, tw.angular.y, tw.angular.z = map(float, ang)
        self._write(ODOM_TOPIC, odom, t_ns)

        # Every lidar slice, in the sensor frame, y flipped to ROS.
        pts = np.frombuffer(lidar.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        pts[:, 1] *= -1.0
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, self.lidar_frame
        cloud.height, cloud.width = 1, len(pts)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.is_bigendian, cloud.point_step, cloud.row_step = False, 16, 16 * len(pts)
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        self._write(self.lidar_topic, cloud, t_ns)

        # IMU in the sensor frame; orientation is the sensor's true world pose.
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = stamp, self.imu_frame
        a, g = imu.accelerometer, imu.gyroscope
        _, q_imu = ros_pose(imu.transform)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a.x, -a.y, a.z
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = -g.x, g.y, -g.z
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q_imu)
        # covariances stay 0: exact, and orientation_covariance[0] != -1 marks it present
        self._write(self.imu_topic, msg, t_ns)

    def close(self):
        self.writer = None  # rosbag2 finalizes the bag when the writer is destroyed


# ---------- main ----------

def bag_base(map_name, lidar_path, lidar_cfg, imu_path, imu_cfg, short=False):
    def tag(path, cfg):
        stem = os.path.splitext(os.path.basename(path))[0]
        return f'{stem.replace("_", "")}-v{cfg.get("version", 0)}'
    return (f'{date.today():%Y-%m-%d}_{map_name.lower()}_'
            f'{tag(lidar_path, lidar_cfg)}_{tag(imu_path, imu_cfg)}' + '_short' * short)


def get_frame(q, frame, what):
    """The item for this sim frame from a CARLA stream's queue."""
    while True:
        try:
            data = q.get(timeout=SENSOR_TIMEOUT_S)
        except queue.Empty:
            raise RuntimeError(f'no {what} data for {SENSOR_TIMEOUT_S:g} s (CARLA also needs '
                               'port+1 reachable for sensor data)') from None
        if data.frame == frame:
            return data
        if data.frame > frame:
            raise RuntimeError(f'{what} skipped frame {frame}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', default=os.environ.get('CARLA_HOST', 'localhost'),
                   help='CARLA host (default: $CARLA_HOST, else localhost)')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--map', default='Town01', help='CARLA map (default: Town01, the v0.0 map)')
    p.add_argument('--short', action='store_true',
                   help='drive only the smallest closed loop through the spawn point instead of '
                        'covering the map')
    p.add_argument('--spawn-point', type=int, default=0, help='index into the map\'s spawn points')
    p.add_argument('--lidar', default=DEFAULT_LIDARS[0], help='lidar config (default: the kart\'s)')
    p.add_argument('--imu', default=DEFAULT_IMUS[0], help='IMU config (default: the kart\'s)')
    p.add_argument('--mounts', default=DEFAULT_MOUNTS,
                   help='sensor mount poses by frame_id (default: the kart\'s)')
    p.add_argument('--out', help='raw bag path (default: bags/<date>_<map>_<lidar>_<imu>[_short]_raw)')
    p.add_argument('--filter', action='store_true', help='also run filter_bag.py -> _filtered')
    args = p.parse_args()

    # Everything local first, so a bad config fails before the map loads.
    lidar_cfg, imu_cfg, mounts = load_configs(args.lidar, args.imu, args.mounts)
    lidar_attrs = lidar_settings(lidar_cfg)
    raw = args.out or os.path.join(
        'bags', bag_base(args.map, args.lidar, lidar_cfg, args.imu, imu_cfg, args.short) + '_raw')
    filtered = (raw[:-4] if raw.endswith('_raw') else raw) + '_filtered'
    for path in [raw] + [filtered] * args.filter:
        if os.path.exists(path):
            raise SystemExit(f'{path} already exists; delete it or pass --out')
    try:
        import rosbag2_py  # noqa: F401
        import carla
    except ImportError as e:
        raise SystemExit(f'{e}. Source ROS 2 (source /opt/ros/jazzy/setup.bash) and install the '
                         'CARLA client matching the server (pip install carla==<version>)') from None

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    original = world.get_settings()
    original.synchronous_mode = False   # left in sync mode, the server would freeze
    world.apply_settings(original)
    if world.get_map().name.rsplit('/', 1)[-1].lower() != args.map.lower():
        world = client.load_world(args.map)   # slow, hence the long timeout
    else:
        # Reloading the map it's already on can wedge the server, so keep it and just
        # remove what an earlier run left behind (a killed run never cleans up).
        cars = [c for c in world.get_actors().filter('vehicle.*')
                if c.attributes.get('role_name') == 'hero']
        stale = cars + [s for s in world.get_actors().filter('sensor.*')
                        if s.parent is not None and s.parent.id in {c.id for c in cars}]
        client.apply_batch([carla.command.DestroyActor(a.id) for a in stale])
    client.set_timeout(10.0)   # from here a stalled server fails in 10 s, not 60
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(settings)

    actors, writer, tick_cb = [], None, None
    ok, interrupted, max_speed, last_report = False, False, 0.0, -10.0
    try:
        carla_map = world.get_map()
        spawns = carla_map.get_spawn_points()
        if not 0 <= args.spawn_point < len(spawns):
            raise SystemExit(f'--spawn-point must be 0-{len(spawns) - 1} on {args.map}')
        spawn = spawns[args.spawn_point]

        edges = map_edges(carla_map)
        wp = carla_map.get_waypoint(spawn.location)
        e0, i0 = nearest_edge(edges, spawn.location.x, spawn.location.y, wp.road_id, wp.lane_id)
        speed = SPEED_KMH / 3.6
        route, n_cov, n_roads = plan_route(edges, e0, i0, args.short)
        print(f'Route: {length(route) / 1000:.2f} km covering {n_cov}/{n_roads} roads, '
              f'about {(length(route) / speed + 2 * STILL_S) / 60:.1f} min of sim time')

        bp_lib = world.get_blueprint_library()
        vbp = bp_lib.find('vehicle.tesla.model3')
        vbp.set_attribute('role_name', 'hero')
        ego = world.spawn_actor(vbp, spawn)
        actors.append(ego)
        sensors = []
        for bp, cfg in ((make_blueprint(bp_lib, 'sensor.lidar.ray_cast', lidar_attrs), lidar_cfg),
                        (make_blueprint(bp_lib, 'sensor.other.imu', {'sensor_tick': 0.0}), imu_cfg)):
            m = mounts[cfg['frame_id']]
            sensors.append(world.spawn_actor(bp, carla_mount(m['xyz'], m['rpy']), attach_to=ego))
        actors += sensors
        snap_q, lidar_q, imu_q = queue.Queue(), queue.Queue(), queue.Queue()
        tick_cb = world.on_tick(snap_q.put)
        sensors[0].listen(lidar_q.put)
        sensors[1].listen(imu_q.put)

        spectator = world.get_spectator()   # the server window's camera; follows the car
        follower = RouteFollower(route, speed, *vehicle_geometry(ego))
        os.makedirs(os.path.dirname(os.path.abspath(raw)), exist_ok=True)
        settle_ticks, still_ticks = round(SETTLE_S / FIXED_DELTA_S), round(STILL_S / FIXED_DELTA_S)
        phase, phase_ticks, slow_ticks = 'settle', 0, 0

        print(f'Recording {raw}  (Ctrl+C stops early and keeps what was recorded)')
        while phase != 'done':
            frame = world.tick()
            snap = get_frame(snap_q, frame, 'world tick')
            lidar, imu = get_frame(lidar_q, frame, 'lidar'), get_frame(imu_q, frame, 'IMU')
            t_ns = int(round(snap.timestamp.elapsed_seconds * 1e9))
            ego_snap = snap.find(ego.id)
            tf, vel = ego_snap.get_transform(), ego_snap.get_velocity()
            if frame % 10 == 0:   # 10 Hz is smooth enough and spares the RPC link
                spectator.set_transform(carla.Transform(
                    carla.Location(tf.location.x, tf.location.y, tf.location.z + 40.0),
                    carla.Rotation(pitch=-90.0)))
            v = math.hypot(vel.x, vel.y)
            phase_ticks += 1

            if phase == 'drive':
                throttle, steer, brake = follower.step(tf.location.x, tf.location.y,
                                                       tf.rotation.yaw, v, FIXED_DELTA_S)
                max_speed = max(max_speed, v)
                slow_ticks = slow_ticks + 1 if v < 0.2 else 0
                if slow_ticks * FIXED_DELTA_S > STUCK_S:
                    raise RuntimeError(f'stuck for {STUCK_S:g} s at ({tf.location.x:.1f}, '
                                       f'{tf.location.y:.1f})')
                if follower.remaining < 1.0 and v < 0.05:
                    phase, phase_ticks = 'end', 0
            else:
                throttle, steer, brake = 0.0, 0.0, 1.0
            ego.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake,
                                                   hand_brake=phase != 'drive'))

            if phase == 'settle':
                if phase_ticks >= settle_ticks:
                    writer = BagWriter(raw, lidar_cfg, imu_cfg)
                    writer.write_static(t_ns, mounts)
                    phase, phase_ticks = 'start', 0
                continue
            writer.write_tick(t_ns, ego_snap, lidar, imu, FIXED_DELTA_S)
            if phase == 'start' and phase_ticks >= still_ticks:
                phase, phase_ticks = 'drive', 0
            elif phase == 'end' and phase_ticks >= still_ticks:
                phase = 'done'

            sim_s = (t_ns - writer.first_ns) * 1e-9
            if sim_s - last_report >= 10.0:
                last_report = sim_s
                print(f'  {sim_s:6.0f} s  {100 * follower.s[follower.i] / follower.s[-1]:5.1f}% of '
                      f'route  {3.6 * v:4.1f} km/h', flush=True)
        ok = True
    except KeyboardInterrupt:
        interrupted = True
    except RuntimeError as e:
        print(f'error: {e}', file=sys.stderr)
    finally:
        if writer is not None:
            writer.close()
        client.set_timeout(5.0)  # never hang on a frozen server during cleanup
        try:
            if tick_cb is not None:
                world.remove_on_tick(tick_cb)
            for a in actors[1:]:
                a.stop()
            world.apply_settings(original)
            client.apply_batch([carla.command.DestroyActor(a.id) for a in actors])
        except RuntimeError as e:
            print(f'warning: CARLA cleanup failed ({e}); restart CARLA if it is stuck in '
                  'synchronous mode')

    print(f'max speed {3.6 * max_speed:.1f} km/h (cap {SPEED_KMH:g})')
    if not ok:
        raise SystemExit(f'{raw} is partial ({"interrupted" if interrupted else "drive failed"}); '
                         'it does not return to the start')
    print(f'Done: {raw}')

    if args.filter:
        cmd = [sys.executable, os.path.join(HERE, 'filter_bag.py'), raw, filtered,
               '--lidar', args.lidar, '--imu', args.imu]
        print('Running', ' '.join(cmd))
        sys.exit(subprocess.run(cmd).returncode)


if __name__ == '__main__':
    main()
