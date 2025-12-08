from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

'''
Run this file initially with ros2 launch ap1_launch pnc_backend.launch.py 

it will get planning, control and the sim up and running 
'''

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('ap1_control'),
        'config',
        'control_node_cfg.csv',
    )

    control_cfg_arg = DeclareLaunchArgument(
        'control_cfg',
        default_value=TextSubstitution(text=default_cfg),
        description='Path to control_node CSV config file'
    )

    control = Node(
        package='ap1_control',
        executable='control_node',
        name='ap1_control',
        output='log',
        arguments=[LaunchConfiguration('control_cfg')],
    )

    planner = Node(
        package='ap1_planning',
        executable='planner_node',
        name='ap1_planning',
        output='log',
    )

    return LaunchDescription([
        control_cfg_arg,
        control,
        TimerAction(period=2.0, actions=[planner]),
    ])
