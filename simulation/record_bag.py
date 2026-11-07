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
import math
import os
import queue
import subprocess
import sys

from bag_writer import BagWriter, bag_base
from carla_frames import open_world
from carla_sensors import (DEFAULT_MOUNTS, IMU_ATTRIBUTES, attach_sensor, lidar_attributes,
                           load_configs)
from route import RouteFollower, length, map_edges, nearest_edge, plan_route, vehicle_geometry
from sensor_config import DEFAULT_IMUS, DEFAULT_LIDARS

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
    lidar_cfg, imu_cfg, mounts = load_configs(args.lidar, args.imu, args.mounts, FIXED_DELTA_S)
    lidar_attrs = lidar_attributes(lidar_cfg, LIDAR_OVERSAMPLE, LIDAR_RANGE_M)
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
    world, original = open_world(client, args.map, FIXED_DELTA_S)

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
        lidar_sensor = attach_sensor(world, ego, 'sensor.lidar.ray_cast', lidar_attrs,
                                     mounts[lidar_cfg['frame_id']])
        actors.append(lidar_sensor)
        imu_sensor = attach_sensor(world, ego, 'sensor.other.imu', IMU_ATTRIBUTES,
                                   mounts[imu_cfg['frame_id']])
        actors.append(imu_sensor)
        snap_q, lidar_q, imu_q = queue.Queue(), queue.Queue(), queue.Queue()
        tick_cb = world.on_tick(snap_q.put)
        lidar_sensor.listen(lidar_q.put)
        imu_sensor.listen(imu_q.put)

        spectator = world.get_spectator()   # the server window's camera; follows the car
        follower = RouteFollower(route, speed, *vehicle_geometry(ego), LOOKAHEAD_M, MAX_OFF_ROUTE_M)
        os.makedirs(os.path.dirname(os.path.abspath(raw)), exist_ok=True)
        settle_ticks, still_ticks = round(SETTLE_S / FIXED_DELTA_S), round(STILL_S / FIXED_DELTA_S)
        phase, phase_ticks, slow_ticks = 'settle', 0, 0

        print(f'Recording {raw}  (Ctrl+C stops early and keeps what was recorded)')
        # Driving, sensors and recording share this one loop, which keeps every
        # LiDAR slice in step with the IMU.
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
