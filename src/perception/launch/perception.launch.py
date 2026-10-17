"""Launch the full AP1 perception stack (YOLO + UFLD/ground)."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def _find_venv_site_packages() -> Path | None:
    """Locate the perception uv venv site-packages without hard-coded home paths."""
    override = os.environ.get('AP1_PERCEPTION_VENV')
    candidates = []
    if override:
        override_path = Path(override).expanduser()
        candidates.extend([
            override_path,
            override_path / 'lib',
        ])

    for parent in Path(__file__).resolve().parents:
        candidates.extend([
            parent / '.venv' / 'lib',
            parent / 'src' / 'perception' / '.venv' / 'lib',
            parent / 'src' / 'src' / 'perception' / '.venv' / 'lib',
        ])

    for candidate in candidates:
        if candidate.name == 'site-packages' and candidate.exists():
            return candidate
        if candidate.exists():
            matches = sorted(candidate.glob('python3*/site-packages'))
            if matches:
                return matches[0]

    return None


def generate_launch_description():
    site_packages = _find_venv_site_packages()
    env_actions = (
        [
            SetEnvironmentVariable(
                'PYTHONPATH',
                str(site_packages) + ':' + os.environ.get('PYTHONPATH', ''),
            )
        ]
        if site_packages
        else []
    )

    return LaunchDescription(env_actions + [
        Node(
            package='ap1_perception',
            executable='yolo_node',
            name='ap1_yolo',
            output='screen',
        ),
        Node(
            package='ap1_perception',
            executable='ufld_ground_node',
            name='ap1_ufld_ground',
            output='screen',
        ),
    ])
