import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

'''
This file kinda works, but because of the logs of the other packages, it screws up the interface itself, not being able to click
'''
def generate_launch_description():
    use_synthetic_actuation_feedback = LaunchConfiguration(
        'use_synthetic_actuation_feedback'
    )
    use_synthetic_odometry = LaunchConfiguration('use_synthetic_odometry')

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
    synthetic_actuation_feedback = Node(
        package='ap1_control',
        executable='synthetic_actuation_feedback_node',
        name='synthetic_actuation_feedback',
        output='screen',
        condition=IfCondition(use_synthetic_actuation_feedback),
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
        ),
        launch_arguments={
            'use_synthetic_odometry': use_synthetic_odometry,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_synthetic_actuation_feedback',
            default_value='false',
            description=(
                'Publish fake /ap1/actuation feedback for smoke tests.'
            ),
        ),
        DeclareLaunchArgument(
            'use_synthetic_odometry',
            default_value='false',
            description='Forward synthetic odometry flag to mapping launch.',
        ),
        control,
        synthetic_actuation_feedback,
        planner,
        yolo,
        ufld_ground,
        console,
        mapping
    ])
