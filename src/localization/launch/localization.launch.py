
"""Publish static sensor transforms using simulation/config/mounts.yaml.

Publishes:
    base_link -> lidar_link
    base_link -> imu_link

Sensor positions are defined in metres and orientations in degrees.
"""

import os
import math
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def static_sensor_transform(sensor_frame, mount):
    """Create a static TF publisher for one sensor."""

    xyz = mount['xyz']

    # Convert rotation values from degrees to radians.
    roll, pitch, yaw = [
        math.radians(float(angle)) for angle in mount['rpy']
    ]

    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{sensor_frame}_static_transform',
        arguments=[
            '--x', str(xyz[0]),
            '--y', str(xyz[1]),
            '--z', str(xyz[2]),
            '--roll', str(roll),
            '--pitch', str(pitch),
            '--yaw', str(yaw),
            '--frame-id', 'base_link',
            '--child-frame-id', sensor_frame
        ],
        output='screen'
    )


def generate_launch_description():

    # Find the installed localization package configuration.
    localization_share = get_package_share_directory('ap1_localization')

    mounts_file = os.path.join(
        localization_share,
        'config',
        'mounts.yaml'
    )

    # Load sensor mounting positions and orientations.
    with open(mounts_file, 'r') as file:
        mounts = yaml.safe_load(file)

    return LaunchDescription([
        static_sensor_transform('lidar_link', mounts['lidar_link']),
        static_sensor_transform('imu_link', mounts['imu_link']),
    ])
