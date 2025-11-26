from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

'''
This file kinda works, but because of the logs of the other packages, it screws up the interface itself, not being able to click
'''
def generate_launch_description():
    control = Node(
        package='ap1_control',
        executable='control_node',
        name='ap1_control',
        output='log', 
        # need to change this but this works for the time being
        arguments=[
            '/home/obaidmm/Repo/ap1/src/planning_and_control/control/control_node_cfg.csv',
        ],
    )

    planner = Node(
        package='ap1_planning',
        executable='planner_node',
        name='ap1_planning',
        output='log', 
    )

    sim = Node(
        package='ap1_pnc_sim',
        executable='pnc_sim_node',
        name='sim_node',
        output='log', 
    )

    ui = Node(
        package='ap1_control_interface',
        executable='system_interface',
        name='ap1_control_interface',
        output='screen',  
    )

    return LaunchDescription([
        control,
        TimerAction(period=2.0, actions=[planner]),
        TimerAction(period=4.0, actions=[sim]),
        TimerAction(period=6.0, actions=[ui]),
    ])
