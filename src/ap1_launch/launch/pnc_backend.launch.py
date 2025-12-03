from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

'''
Run this file initially with ros2 launch ap1_launch pnc_backend.launch.py 

it will get planning, control and the sim up and running 
'''

def generate_launch_description():
    control = Node(
        package='ap1_control',
        executable='control_node',
        name='ap1_control',
        output='screen', 
        arguments=[
            # should be removed down the line, using a filler for now 
            '/home/obaidmm/Repo/ap1/src/planning_and_control/control/control_node_cfg.csv',
        ],
    )

    planner = Node(
        package='ap1_planning',
        executable='planner_node',
        name='ap1_planning',
        output='screen',
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
