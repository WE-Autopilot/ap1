import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

'''
This file kinda works, but because of the logs of the other packages, it screws up the interface itself, not being able to click
'''
def generate_launch_description():
    # Helper so that a failure in this node takes down all of AP1 instead of just continuing
    def CriticalNode(**kwargs):
        return Node(on_exit=Shutdown(), **kwargs)
    
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

    # == PERCEPTION NODE ==
    pkg_share = get_package_share_directory('ap1_perception')
    perception = CriticalNode(
        package='ap1_perception',
        executable='percpetion_node',
        name='ap1_perception',
        output='log',
    )

    # == CONSOLE NODE ==
    console = Node(
        package='ap1_console',
        executable='console',
        name='ap1_console',
        output='screen',
    )

    return LaunchDescription([
        control,
        planner,
        perception,
        console,
    ])
