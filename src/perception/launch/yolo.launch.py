import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_venv_site_packages() -> Path:
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

    raise RuntimeError('Could not find .venv — is the workspace set up correctly?')


def _yolo_node(context):
    params = [LaunchConfiguration('params_file'), {'device': LaunchConfiguration('device')}]
    # Forward model_path only when given, so the node's default (the model
    # installed with the package) applies otherwise
    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path:
        params.append({'model_path': model_path})
    return [Node(
        package='ap1_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=params,
    )]


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

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('ap1_perception'), 'config', 'yolo.yaml'),
            description='YAML file with the YOLO node tuning parameters'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Path to a YOLO .onnx model; empty uses the one installed with the package'),
        DeclareLaunchArgument(
            'device', default_value='auto',
            description="'cuda' (fail if no GPU), 'cpu', or 'auto' (GPU when available)"),
    ]

    return LaunchDescription(args + env_actions + [OpaqueFunction(function=_yolo_node)])
