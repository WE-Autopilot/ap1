import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

'''
Official P&C Launch File

Example usage:

  ros2 launch ap1_launch ap1.launch.py \
    control_cfg:=/path/to/control_node_cfg.csv \
    map_file_path:=/path/to/MyMap.osm \
    use_sim:=true
    t
By default, control_cfg points to the installed:
  share/ap1_control/config/control_node_cfg.csv

pnc_sim is optional via the use_sim argument.

'''


def generate_launch_description():

    # this is gonna go crazy and find the csv file within the control directory
    default_cfg = os.path.join(
        get_package_share_directory('ap1_control'),
        'config',
        'control_node_cfg.csv',
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file_path',
        default_value='',
        description='Path to Lanelet2 map file (OSM). Leave empty for mock road.'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Whether to start ap1_pnc_sim (true/false)'
    )

    control_cfg_arg = DeclareLaunchArgument(
        'control_cfg',
        default_value=TextSubstitution(text=default_cfg),
        description='Path to control_node CSV config file'
    )

    # ===== Nodes =====
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
        parameters=[{
            'map_file_path': LaunchConfiguration('map_file_path')
        }],
    )

    sim = Node(
        package='ap1_pnc_sim',
        executable='pnc_sim_node',
        name='sim_node',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    return LaunchDescription([
        control_cfg_arg,
        map_file_arg,
        use_sim_arg,

        control,
        TimerAction(period=2.0, actions=[planner]),
        TimerAction(period=4.0, actions=[sim]),
    ])
