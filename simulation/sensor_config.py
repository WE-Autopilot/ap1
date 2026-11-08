"""
Loads and checks the YAML configs that record_bag.py and filter_bag.py share,
and reads the beam layout and range limits out of a lidar config. Needs only
numpy and PyYAML. simulation/README.md documents the config format.
"""
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
# What each unit measures, so config checking can reject e.g. a time given in
# deg. Random walks are noise densities in other units.
UNIT_KIND = {u: kind for kind, units in {
    'accel': ['m/s^2', 'g', 'mg', 'ug'], 'gyro': ['rad/s', 'deg/s', 'deg/h'],
    'angle': ['rad', 'deg'], 'time': ['s', 'min', 'h'],
    'accel/rtHz': ['m/s/rt-h'], 'gyro/rtHz': ['deg/rt-h', 'rad/rt-h'],
}.items() for u in units}


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


# ---------- config checking ----------

class ConfigError(ValueError):
    pass


def is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def leaf(expected, ok):
    """Checker for a key whose value must pass ok(); `expected` goes in the error."""
    def check(value, where):
        if not ok(value):
            raise ConfigError(f'{where} must be {expected}, got {value!r}')
    return check


def pair(ok):
    return lambda v: isinstance(v, list) and len(v) == 2 and all(map(ok, v))


STR = leaf('a string', lambda v: isinstance(v, str))
INT = leaf('an integer', is_int)
BOOL = leaf('true or false', lambda v: isinstance(v, bool))
NUM = leaf('a number', is_number)
NUMS = leaf('a list of numbers', lambda v: isinstance(v, list) and all(map(is_number, v)))
RANGE = leaf('[start, end] (two numbers)', pair(is_number))
CHANNELS = leaf('[first, last] (two integers)', pair(is_int))
STEPS = leaf('a non-empty list of [up_to_m, stddev_m] pairs',
             lambda v: isinstance(v, list) and v and all(map(pair(is_number), v)))


def qty(kind):
    """Checker for an IMU quantity: a bare SI number, or a string in `kind` units."""
    def check(value, where):
        if not (is_number(value) or isinstance(value, str)):
            raise ConfigError(f'{where} must be a number or "<number> <unit>", got {value!r}')
        try:
            quantity(value)
        except ValueError as e:
            raise ConfigError(f'{where}: {e}') from None
        if isinstance(value, str):
            unit = value.strip().partition(' ')[2].strip()
            got = (UNIT_KIND[unit[:-len('/rtHz')]] + '/rtHz' if unit.endswith('/rtHz')
                   else UNIT_KIND[unit])
            if got != kind:
                raise ConfigError(f'{where}: "{value}" is in {got} units, expected {kind}')
    return check


# Every key a config may have, with a checker for its value. A nested dict is a
# section; [dict] is a list of such sections; (checker, [dict]) is either.
# Unknown keys are rejected, so a typo can't silently fall back to a default.
TOPICS = {'input': STR, 'output': STR}
LIDAR_KEYS = {
    'name': STR, 'version': INT, 'topics': TOPICS,
    'frame_id': STR, 'input_left_handed': BOOL, 'rotation_hz': NUM,
    'beams': {'count': INT, 'lower_deg': NUM, 'upper_deg': NUM,
              'elevations_deg': NUMS, 'tolerance_deg': NUM},
    'min_range_m': NUM, 'max_range_m': (NUM, [{'channels': CHANNELS, 'range_m': NUM}]),
    'horizontal_fov_deg': RANGE, 'azimuth_resolution_deg': NUM,
    'noise': {'range_accuracy': [{'channels': CHANNELS, 'steps': STEPS}], 'dropout_rate': NUM},
    'intensity_scale': NUM, 'seed': INT,
}


def imu_axis(unit):
    return {'noise_density': qty(unit + '/rtHz'), 'white_noise_stddev': qty(unit),
            'turn_on_bias': qty(unit), 'bias_instability': qty(unit),
            'bias_correlation_time': qty('time')}


IMU_KEYS = {
    'name': STR, 'version': INT, 'topics': TOPICS,
    'frame_id': STR, 'input_left_handed': BOOL, 'rate_hz': NUM,
    'accel': imu_axis('accel'), 'gyro': imu_axis('gyro'),
    'orientation': {'roll_pitch_stddev': qty('angle'), 'heading_stddev': qty('angle'),
                    'correlation_time': qty('time')},
    'seed': INT,
}


def check_keys(cfg, schema, where):
    """Reject unknown keys and values of the wrong type."""
    if not isinstance(cfg, dict):
        raise ConfigError(f'{where} must be a mapping, got {cfg!r}')
    unknown = sorted(set(cfg) - set(schema))
    if unknown:
        raise ConfigError(f'{where}: unknown key(s) {unknown}; allowed: {sorted(schema)}')
    for key, sub in schema.items():
        if key not in cfg:
            continue
        value, at = cfg[key], f'{where}.{key}'
        if isinstance(sub, tuple):
            # A plain value, or a list of sections.
            if not isinstance(value, list):
                sub[0](value, at)
                continue
            sub = sub[1]
        if callable(sub):
            sub(value, at)
        elif value is None:
            continue  # empty section, e.g. "noise:" with nothing under it
        elif isinstance(sub, list):
            if not isinstance(value, list):
                raise ConfigError(f'{at} must be a list, got {value!r}')
            for i, item in enumerate(value):
                check_keys(item, sub[0], f'{at}[{i}]')
        else:
            check_keys(value, sub, at)


def require(cfg, keys, where, why=''):
    missing = [k for k in keys if k not in (cfg or {})]
    if missing:
        raise ConfigError(f'{where}: missing {", ".join(missing)}{why}')


def check_lidar_config(cfg):
    check_keys(cfg, LIDAR_KEYS, 'lidar config')
    require(cfg, ['topics', 'frame_id', 'rotation_hz', 'beams', 'max_range_m',
                  'azimuth_resolution_deg'], 'lidar config')
    require(cfg['topics'], ['input', 'output'], 'topics')
    b = cfg['beams'] or {}
    require(b, ['tolerance_deg'], 'beams')
    if 'elevations_deg' not in b:
        require(b, ['count', 'lower_deg', 'upper_deg'], 'beams', ' (or give elevations_deg)')
    if isinstance(cfg['max_range_m'], list):
        for i, zone in enumerate(cfg['max_range_m']):
            require(zone, ['range_m'], f'max_range_m[{i}]')
    for i, zone in enumerate((cfg.get('noise') or {}).get('range_accuracy') or []):
        require(zone, ['steps'], f'noise.range_accuracy[{i}]')


def check_imu_config(cfg):
    check_keys(cfg, IMU_KEYS, 'IMU config')
    require(cfg, ['topics', 'frame_id', 'rate_hz'], 'IMU config')
    require(cfg['topics'], ['input', 'output'], 'topics')
    for axis in ('accel', 'gyro'):
        c = cfg.get(axis) or {}
        if 'bias_instability' in c:
            require(c, ['bias_correlation_time'], axis, ' (needed when bias_instability is set)')
    o = cfg.get('orientation') or {}
    if 'roll_pitch_stddev' in o or 'heading_stddev' in o:
        require(o, ['correlation_time'], 'orientation', ' (needed when a stddev is set)')


# ---------- lidar config values ----------

def beam_elevations(beams):
    """Beam elevations (rad) in ring order: an explicit list, or evenly spaced."""
    if 'elevations_deg' in beams:
        return np.radians(np.asarray(beams['elevations_deg'], dtype=np.float64))
    return np.radians(np.linspace(beams['lower_deg'], beams['upper_deg'], beams['count']))


def channel_mask(zone, ring):
    """Points in a zone's channels: [first, last] (1-based, optional = all)."""
    if 'channels' not in zone:
        return np.ones(len(ring), dtype=bool)
    lo, hi = zone['channels']
    return (ring >= lo - 1) & (ring <= hi - 1)


def ring_max_range(max_range, n_rings):
    """
    Max range (m) per ring, from a number or a list of zones
    {channels: [first, last] (optional = all), range_m}. Later zones win.
    """
    if isinstance(max_range, (int, float)):
        return np.full(n_rings, float(max_range))
    caps = np.full(n_rings, np.nan)
    rings = np.arange(n_rings)
    for zone in max_range:
        caps[channel_mask(zone, rings)] = zone['range_m']
    if np.isnan(caps).any():
        missing = (np.flatnonzero(np.isnan(caps)) + 1).tolist()
        raise ConfigError(f'max_range_m: no zone covers channel(s) {missing}')
    return caps


# ---------- loading ----------

def load(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise SystemExit(f'{path}: expected a YAML mapping of config keys')
    return cfg
