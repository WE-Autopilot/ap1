import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

'''
Run this file initially with ros2 launch ap1_bringup pnc_backend.launch.py 

it will get planning, control and the sim up and running 
'''

def generate_launch_description():
    control = Node(
        package='ap1_control',
        executable='control_node',
        name='ap1_control',
        output='screen',
        arguments=[
            os.path.join(get_package_share_directory('ap1_control'), 'config', 'control_node_cfg.csv'),
        ],
    )

    planner = Node(
        package='ap1_planning',
        executable='planner_node',
        name='ap1_planning',
        output='screen',
        arguments=[
            os.path.join(get_package_share_directory('ap1_planning'), 'config', 'stop_sign_transitions.yaml'),
        ],
    )

    sim = Node(
        package='ap1_pnc_sim',
        executable='pnc_sim_node',
        name='sim_node',
        output='screen',
    )

    return LaunchDescription([
        control,
        TimerAction(period=2.0, actions=[planner]),
        TimerAction(period=4.0, actions=[sim]),
    ])
