"""Tests for recording.yaml handling, shared names and the CARLA version check."""
import os

import pytest
import yaml

from carla_frames import ODOM_TOPIC, VEHICLE_FRAME, versions_match
from carla_sensors import lidar_attributes
from record_bag import DEFAULT_RECORDING, parse_args, recording_settings
from sensor_config import DEFAULT_LIDARS, load


def write_config(tmp_path, **changes):
    cfg = {**load(DEFAULT_RECORDING), **changes}
    path = tmp_path / 'recording.yaml'
    path.write_text(yaml.safe_dump(cfg))
    return str(path)


def test_shipped_config_has_the_ticket_values():
    cfg = recording_settings(DEFAULT_RECORDING, {})
    assert cfg['tick_s'] == 0.01
    assert cfg['speed_kmh'] == 10.0
    assert cfg['map'] == 'Town01'
    assert cfg['bag_dir'] == os.path.expanduser('~/WEAP/bags')


def test_flags_override_the_config_and_absent_flags_do_not():
    flags = vars(parse_args(['--map', 'Town02', '--tick-s', '0.005', '--bag-dir', '/data/bags']))
    cfg = recording_settings(DEFAULT_RECORDING, flags)
    assert (cfg['map'], cfg['tick_s'], cfg['bag_dir']) == ('Town02', 0.005, '/data/bags')
    assert cfg['speed_kmh'] == 10.0


def test_spawn_point_zero_on_the_command_line_counts_as_given(tmp_path):
    path = write_config(tmp_path, spawn_point=7)
    assert recording_settings(path, vars(parse_args(['--spawn-point', '0'])))['spawn_point'] == 0


def test_unknown_key_is_rejected(tmp_path):
    with pytest.raises(SystemExit, match='speed_kph'):
        recording_settings(write_config(tmp_path, speed_kph=10), {})


def test_missing_key_is_rejected(tmp_path):
    cfg = load(DEFAULT_RECORDING)
    del cfg['tick_s']
    path = tmp_path / 'recording.yaml'
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(SystemExit, match='tick_s'):
        recording_settings(str(path), {})


def test_flag_values_are_checked_too():
    with pytest.raises(SystemExit, match='tick_s'):
        recording_settings(DEFAULT_RECORDING, {'tick_s': 0.0})


def test_oversample_and_range_reach_the_lidar_attributes():
    lidar = load(DEFAULT_LIDARS[0])
    attrs = lidar_attributes(lidar, 2, 60.0)
    assert attrs['points_per_second'] == 2 * 24 * 600 * 10
    assert attrs['range'] == 60.0
    assert lidar_attributes(lidar, 1, 60.0)['points_per_second'] == 24 * 600 * 10


def test_filter_defaults_use_the_shared_names(monkeypatch):
    import filter_bag
    seen = {}
    monkeypatch.setattr(filter_bag, 'filter_bag', lambda *a: seen.update(frame=a[5], odom=a[7]))
    monkeypatch.setattr('sys.argv', ['filter_bag.py', 'in', 'out'])
    filter_bag.main()
    assert seen == {'frame': VEHICLE_FRAME, 'odom': ODOM_TOPIC}


@pytest.mark.parametrize('client,server,same', [
    ('0.9.16', '0.9.16', True),
    ('0.9.16', '0.9.16-12-gabc123', True),
    ('0.9.15', '0.9.16', False),
    ('0.9.16', '0.9.1', False),
])
def test_version_check(client, server, same):
    assert versions_match(client, server) is same


def slow_imu(tmp_path, rate_hz=10.0):
    """The kart's IMU config at a lower rate, so only the lidar limits the tick."""
    from sensor_config import DEFAULT_IMUS
    path = tmp_path / 'imu.yaml'
    path.write_text(yaml.safe_dump({**load(DEFAULT_IMUS[0]), 'rate_hz': rate_hz}))
    return str(path)


def test_tick_too_long_for_the_lidar_fails_before_recording(tmp_path):
    from carla_sensors import DEFAULT_MOUNTS, load_configs
    imu = slow_imu(tmp_path)
    load_configs(DEFAULT_LIDARS[0], imu, DEFAULT_MOUNTS, 0.05)   # half a 10 Hz rotation is fine
    with pytest.raises(SystemExit, match='tick_s 0.06 is too long'):
        load_configs(DEFAULT_LIDARS[0], imu, DEFAULT_MOUNTS, 0.06)


def test_missing_mount_names_the_mounts_file(tmp_path):
    from carla_sensors import load_configs
    from sensor_config import DEFAULT_IMUS
    mounts = tmp_path / 'mounts.yaml'
    mounts.write_text(yaml.safe_dump({'imu_link': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}}))
    with pytest.raises(SystemExit, match=f'{mounts}: needs "lidar_link"'):
        load_configs(DEFAULT_LIDARS[0], DEFAULT_IMUS[0], str(mounts), 0.01)
