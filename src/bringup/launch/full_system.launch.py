import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    Shutdown,
)
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

'''
This file kinda works, but because of the logs of the other packages, it screws up the interface itself, not being able to click
'''


def _find_perception_venv_site_packages():
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
            return str(candidate)
        if candidate.exists():
            matches = sorted(candidate.glob('python3*/site-packages'))
            if matches:
                return str(matches[0])

    return None


def generate_launch_description():
    # Helper so that a failure in this node takes down all of AP1 instead of just continuing
    def CriticalNode(**kwargs):
        return Node(on_exit=Shutdown(), **kwargs)

    perception_site_packages = _find_perception_venv_site_packages()
    env_actions = (
        [
            SetEnvironmentVariable(
                'PYTHONPATH',
                perception_site_packages + ':' + os.environ.get('PYTHONPATH', ''),
            )
        ]
        if perception_site_packages else []
    )
    
    # == CONTROL NODE ==
    # needs a path to control cfg.csv
    pkg_share = get_package_share_directory('ap1_control')
    csv_file_path = os.path.join(pkg_share, 'config', 'control_node_cfg.csv')
    control = CriticalNode(
        package='ap1_control',
        executable='control_node',
        name='ap1_control',
        output='log', 
        arguments=[
            csv_file_path
        ],
    )

    # == PLANNER NODE ==
    pkg_share = get_package_share_directory('ap1_planning')
    transitions_file_path = os.path.join(pkg_share, 'config', 'stop_sign_transitions.yaml')
    planner = CriticalNode(
        package='ap1_planning',
        executable='planner_node',
        name='ap1_planning',
        output='log', 
        arguments=[
            transitions_file_path
        ]
    )

    # == PERCEPTION NODES ==
    yolo = CriticalNode(
        package='ap1_perception',
        executable='yolo_node',
        name='ap1_yolo',
        output='log',
    )
    ufld_ground = CriticalNode(
        package='ap1_perception',
        executable='ufld_ground_node',
        name='ap1_ufld_ground',
        output='log',
    )

    # == CONSOLE NODE ==
    console = Node(
        package='ap1_console',
        executable='console',
        name='ap1_console',
        output='screen',
    )

    # == MAPPING ==
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mapping_localization_python'),
                'launch',
                'mapping_pipeline.launch.py'
            )
        )
    )

    return LaunchDescription(env_actions + [
        control,
        planner,
        yolo,
        ufld_ground,
        console,
        mapping
    ])
