from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    control = Node(
        package='ap1_control',
        executable='control_node',
        name='ap1_control',
        output='screen',
        arguments=['/home/obaidmm/Repo/ap1/src/planning_and_control/control/control_node_cfg.csv'],  # required by your main()
    )

    ui = Node(
        package='ap1_control_interface',
        executable='system_interface',
        name='ap1_control_interface',
        output='screen',
    )

    sim = Node(
        package='ap1_pnc_sim',
        executable='pnc_sim_node',
        name='sim_node',
        output='screen',
    )

    planner = Node(
        package='ap1_planning',
        executable='planner_node',
        name='ap1_planning',
        output='screen',
    )

    return LaunchDescription([
        control,
        TimerAction(period=2.0, actions=[ui]),
        TimerAction(period=4.0, actions=[sim]),
        TimerAction(period=5.0, actions=[planner]),
    ])
