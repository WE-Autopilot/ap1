"""LIO-SAM on filtered CARLA bags (or the kart).

Same nodes as LIO-SAM's run.launch.py minus robot_state_publisher: that one
publishes LIO-SAM's own URDF (identity base_link -> lidar_link / imu_link),
which would fight the bag's /tf_static from simulation/config/mounts.yaml.

    ros2 launch ap1_mapping lio_sam_carla.launch.py
    ros2 launch ap1_mapping lio_sam_carla.launch.py params_file:=/path/to.yaml rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lio_share = get_package_share_directory('lio_sam')
    ap1_share = get_package_share_directory('ap1_mapping')

    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    sim_time = {'use_sim_time': True}

    # on_exit=Shutdown(): if any LIO-SAM node dies, the whole launch exits, so
    # scripts (bag replay, CI) see the launch end instead of a half-running pipeline.
    def lio_node(name):
        return Node(package='lio_sam', executable=f'lio_sam_{name}', name=f'lio_sam_{name}',
                    parameters=[params_file], output='screen', on_exit=Shutdown())

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(ap1_share, 'config', 'lio_sam_carla.yaml'),
            description='LIO-SAM parameters file'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # map and odom coincide, as in LIO-SAM's own launch
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[sim_time], output='screen'),

        lio_node('imuPreintegration'),
        lio_node('imageProjection'),
        lio_node('featureExtraction'),
        lio_node('mapOptimization'),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(lio_share, 'config', 'rviz2.rviz')],
             parameters=[sim_time], output='screen', condition=IfCondition(rviz)),
    ])