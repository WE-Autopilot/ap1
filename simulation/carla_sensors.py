"""
Builds CARLA sensors from the sensor configs and attaches them at the poses in
the mounts config. To add another sensor, such as a camera:

    mounts = load_mounts(DEFAULT_MOUNTS)
    camera = attach_sensor(world, vehicle, 'sensor.camera.rgb',
                           {'image_size_x': 1920, 'image_size_y': 1080, 'sensor_tick': 0.0},
                           mount_pose(mounts, 'camera_link'))
    camera.listen(on_image)
"""
import os

import numpy as np
import yaml

from carla_frames import carla_mount
from sensor_config import (CONFIG_DIR, ConfigError, beam_elevations, check_imu_config,
                           check_lidar_config, load, ring_max_range)

DEFAULT_MOUNTS = os.path.join(CONFIG_DIR, 'mounts.yaml')

IMU_ATTRIBUTES = {'sensor_tick': 0.0}   # every tick, so one sample per sim step


def load_mounts(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except OSError as e:
        raise SystemExit(e) from None


def mount_pose(mounts, frame_id, where='mounts config'):
    """The {xyz, rpy} pose for frame_id; exits if the mounts config doesn't have one."""
    m = mounts.get(frame_id)
    if not (isinstance(m, dict) and len(m.get('xyz') or []) == 3 and len(m.get('rpy') or []) == 3):
        raise SystemExit(f'{where}: needs "{frame_id}" with xyz and rpy (3 numbers each)')
    return m


def load_configs(lidar_path, imu_path, mounts_path, tick_s):
    """The checked (lidar, IMU, mounts) configs for recording at tick_s."""
    try:
        lidar, imu = load(lidar_path), load(imu_path)
    except OSError as e:
        raise SystemExit(e) from None
    mounts = load_mounts(mounts_path)
    for path, cfg, check in ((lidar_path, lidar, check_lidar_config), (imu_path, imu, check_imu_config)):
        try:
            check(cfg)
        except ConfigError as e:
            raise SystemExit(f'{path}: {e}') from None
        mount_pose(mounts, cfg['frame_id'], mounts_path)
    if 1.0 / imu['rate_hz'] < tick_s - 1e-9:
        raise SystemExit(f'{imu_path}: rate_hz {imu["rate_hz"]} is faster than CARLA\'s '
                         f'{1 / tick_s:g} Hz ticks')
    return lidar, imu, mounts


def lidar_attributes(cfg, oversample, range_m):
    """CARLA sensor.lidar.ray_cast attributes that copy the config's beams exactly."""
    elev = np.degrees(beam_elevations(cfg['beams']))
    if len(elev) > 2 and np.ptp(np.diff(np.sort(elev))) > 1e-6:
        raise SystemExit('CARLA\'s lidar only has evenly spaced beams, so it can\'t match '
                         'beams.elevations_deg in this config')
    pps = oversample * len(elev) * 360.0 / cfg['azimuth_resolution_deg'] * cfg['rotation_hz']
    return {
        'channels': len(elev),
        'upper_fov': float(elev.max()),
        'lower_fov': float(elev.min()),
        'rotation_frequency': cfg['rotation_hz'],
        'points_per_second': int(round(pps)),
        'range': max(range_m, float(ring_max_range(cfg['max_range_m'], len(elev)).max())),
        'horizontal_fov': 360.0,
        'sensor_tick': 0.0,            # every tick: one slice per sim step
        # No CARLA noise or drops; the filter adds them from the datasheet.
        'noise_stddev': 0.0,
        'dropoff_general_rate': 0.0,
        'dropoff_intensity_limit': 1.0,    # CARLA also drops low-intensity points otherwise
        'dropoff_zero_intensity': 0.0,
    }


def make_blueprint(bp_lib, name, attributes):
    bp = bp_lib.find(name)
    for key, value in attributes.items():
        bp.set_attribute(key, str(value))
    for attr in bp:  # every noise source off, whatever this CARLA version has
        if attr.id.startswith('noise_') and attr.id != 'noise_seed':
            bp.set_attribute(attr.id, '0')
    return bp


def attach_sensor(world, vehicle, blueprint, attributes, mount):
    """Spawn a sensor on the vehicle at a mount_pose(); the caller must destroy it."""
    bp = make_blueprint(world.get_blueprint_library(), blueprint, attributes)
    return world.spawn_actor(bp, carla_mount(mount['xyz'], mount['rpy']), attach_to=vehicle)
