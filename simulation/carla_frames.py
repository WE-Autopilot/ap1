"""
CARLA <-> ROS conversions, and the helpers record_bag.py and filter_bag.py share.

Defines the frame and topic names the raw bag uses, so both scripts agree on
them. CARLA and ROS are imported only inside the functions that need them, so
this module works with just numpy.
"""
import math

import numpy as np

MAP_FRAME = 'carla_map'
VEHICLE_FRAME = 'carla_base_link'
ODOM_TOPIC = '/carla/hero/odometry'


# ---------- quaternions (x, y, z, w) ----------

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


# ---------- ROS bag helpers ----------

def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def ns_to_stamp(ns):
    from builtin_interfaces.msg import Time
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def topic_metadata(name, type_name):
    import rosbag2_py
    try:
        return rosbag2_py.TopicMetadata(id=0, name=name, type=type_name, serialization_format='cdr')
    except TypeError:  # rosbag2 before Jazzy has no id
        return rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format='cdr')


# ---------- CARLA client helpers ----------

def versions_match(client_version, server_version):
    """Compare releases only: a source-built server reports e.g. 0.9.16-12-gabc123."""
    return client_version.split('-')[0] == server_version.split('-')[0]


def connect(host, port):
    """A carla.Client for a server of the same version; exits with a clear message if not."""
    import carla
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    try:
        server = client.get_server_version()
    except RuntimeError as e:
        raise SystemExit(f'can\'t reach a CARLA server at {host}:{port} ({e}). Is it running? '
                         'See "Recording a raw bag" in simulation/README.md') from None
    local = client.get_client_version()
    if not versions_match(local, server):
        raise SystemExit(
            f'CARLA client {local} doesn\'t match the server at {host}:{port}, which is {server}. '
            f'Install the matching client: python3 -m pip install carla=={server.split("-")[0]} '
            '--break-system-packages')
    return client


def open_world(client, map_name, tick_s):
    """
    The world on map_name in synchronous mode at tick_s. Returns (world, settings
    to restore when done, which are asynchronous so the server keeps running).
    """
    import carla
    client.set_timeout(60.0)    # loading a map is slow
    world = client.get_world()
    original = world.get_settings()
    original.synchronous_mode = False   # left in sync mode, the server would freeze
    world.apply_settings(original)
    if world.get_map().name.rsplit('/', 1)[-1].lower() != map_name.lower():
        world = client.load_world(map_name)
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
    settings.fixed_delta_seconds = tick_s
    world.apply_settings(settings)
    return world, original
