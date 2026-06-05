#!/usr/bin/env python3
"""Summarize accel/gyro streams in a RealSense .bag file."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

import pyrealsense2 as rs


def stream_name(frame: rs.frame) -> str:
    """Return a stable stream name for a RealSense frame."""
    return str(frame.profile.stream_type()).replace('stream.', '')


def summarize_bag(path: Path, timeout_ms: int) -> None:
    """Read a RealSense bag and print stream/sample statistics."""
    config = rs.config()
    rs.config.enable_device_from_file(config, str(path), repeat_playback=False)

    pipe = rs.pipeline()
    profile = pipe.start(config)
    playback = profile.get_device().as_playback()
    playback.set_real_time(False)

    print(f'bag: {path}')
    print(f'device: {profile.get_device().get_info(rs.camera_info.name)}')
    print('streams:')
    for stream_profile in profile.get_streams():
        stream_type = stream_profile.stream_type()
        if stream_type in (rs.stream.color, rs.stream.depth, rs.stream.infrared):
            video_profile = stream_profile.as_video_stream_profile()
            print(
                '  '
                f'{stream_type} {stream_profile.format()} '
                f'{video_profile.width()}x{video_profile.height()} '
                f'@ {stream_profile.fps()} Hz'
            )
        else:
            print(
                '  '
                f'{stream_type} {stream_profile.format()} '
                f'@ {stream_profile.fps()} Hz'
            )

    counts: Counter[str] = Counter()
    first_ts: dict[str, float] = {}
    last_ts: dict[str, float] = {}
    first_values: dict[str, tuple[float, float, float]] = {}
    last_values: dict[str, tuple[float, float, float]] = {}

    try:
        while True:
            frames = pipe.wait_for_frames(timeout_ms)
            for frame in frames:
                name = stream_name(frame)
                counts[name] += 1
                timestamp = frame.get_timestamp()
                first_ts.setdefault(name, timestamp)
                last_ts[name] = timestamp
                if frame.is_motion_frame():
                    motion = frame.as_motion_frame().get_motion_data()
                    value = (motion.x, motion.y, motion.z)
                    first_values.setdefault(name, value)
                    last_values[name] = value
    except RuntimeError as exc:
        print(f'end/read timeout: {exc}')
    finally:
        pipe.stop()

    print('sample summary:')
    for name in sorted(counts):
        duration_sec = (last_ts[name] - first_ts[name]) / 1000.0
        approx_hz = (
            (counts[name] - 1) / duration_sec if duration_sec > 0.0 else 0.0
        )
        print(
            f'  {name}: count={counts[name]} '
            f'duration={duration_sec:.3f}s approx_hz={approx_hz:.2f} '
            f'first={first_values.get(name)} last={last_values.get(name)}'
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Summarize RealSense accel/gyro streams from a .bag file.'
    )
    parser.add_argument('bag', type=Path)
    parser.add_argument('--timeout-ms', type=int, default=5000)
    args = parser.parse_args()

    summarize_bag(args.bag.expanduser().resolve(), args.timeout_ms)


if __name__ == '__main__':
    main()
