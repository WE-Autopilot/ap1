import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_venv_site_packages() -> Path:
    override = os.environ.get('AP1_PERCEPTION_VENV')
    # 'none' turns the lookup off. The Docker images set it: they install every
    # dependency system-wide, and a host .venv seen through the dev container's
    # source mount must not shadow them.
    if override == 'none':
        raise RuntimeError('venv lookup disabled (AP1_PERCEPTION_VENV=none)')
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

    raise RuntimeError('Could not find .venv — is the workspace set up correctly?')


def generate_launch_description():
    try:
        site_packages = str(_find_venv_site_packages())
    except RuntimeError:
        site_packages = None

    env_actions = (
        [SetEnvironmentVariable(
            'PYTHONPATH',
            site_packages + ':' + os.environ.get('PYTHONPATH', ''),
        )]
        if site_packages else []
    )

    default_device = {'1': 'cuda', '0': 'cpu'}.get(os.environ.get('AP1_GPU'), 'auto')

    args = [
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Path to the YOLO .onnx model; empty uses the model bundled in the package'),
        DeclareLaunchArgument(
            'device', default_value=default_device,
            description="'cuda' (fail if no GPU), 'cpu', or 'auto' (GPU when available)"),
    ]

    return LaunchDescription(args + env_actions + [
        Node(
            package='ap1_perception',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
            }],
        ),
    ])
