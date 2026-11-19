"""Loads the localization map: Autoware's point cloud map loader + map projection loader.

The .pcd lives outside the repo (too big for git). By default it's the team map at
~/WEAP/maps/ap1_map/pointcloud_map.pcd (see SETUP.md), but any .pcd works, e.g. a
map_carla.sh output. The projector info (Local, no GNSS) is in config/.

    ros2 launch ap1_localization map_loader.launch.py
    ros2 launch ap1_localization map_loader.launch.py pointcloud_map:=<run>/results/map/GlobalMap.pcd
    ros2 launch ap1_localization map_loader.launch.py use_sim_time:=false

Outputs (all in the `map` frame):
    /map/pointcloud_map                    whole map, latched (viewing in Foxglove/RViz only)
    /map/get_differential_pointcloud_map   service NDT gets its map from
    /map/get_partial_pointcloud_map        service, map cells inside a circle (testing)
    /map/map_projector_info                projector type (Local), latched
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    share = get_package_share_directory('ap1_localization')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # Autoware's default param files read these via $(var ...); normally a parent
    # Autoware launch sets them, so we set them here.
    set_vars = [
        SetLaunchConfiguration('pcd_paths_or_directory', LaunchConfiguration('pointcloud_map')),
        # 'none' (a path that doesn't exist) is fine for a single .pcd: the loader reads
        # the bounds from the cloud. Only a tiled map needs a metadata yaml.
        SetLaunchConfiguration('pcd_metadata_path', LaunchConfiguration('pointcloud_map_metadata')),
        SetLaunchConfiguration('map_projector_info_path', LaunchConfiguration('map_projector_info')),
        # Only read when map_projector_info.yaml is missing (MGRS fallback); no lanelet2 map yet.
        SetLaunchConfiguration('lanelet2_map_path', 'none'),
    ]

    pointcloud_map_loader = Node(
        package='autoware_map_loader',
        executable='autoware_pointcloud_map_loader',
        name='pointcloud_map_loader',
        parameters=[
            ParameterFile(os.path.join(get_package_share_directory('autoware_map_loader'),
                                       'config', 'pointcloud_map_loader.param.yaml'),
                          allow_substs=True),
            sim_time,
        ],
        remappings=[
            ('output/pointcloud_map', '/map/pointcloud_map'),
            ('output/pointcloud_map_metadata', '/map/pointcloud_map_metadata'),
            ('service/get_partial_pcd_map', '/map/get_partial_pointcloud_map'),
            ('service/get_differential_pcd_map', '/map/get_differential_pointcloud_map'),
            ('service/get_selected_pcd_map', '/map/get_selected_pointcloud_map'),
        ],
        output='screen',
    )

    map_projection_loader = Node(
        package='autoware_map_projection_loader',
        executable='autoware_map_projection_loader_node',
        name='map_projection_loader',
        parameters=[
            ParameterFile(os.path.join(get_package_share_directory('autoware_map_projection_loader'),
                                       'config', 'map_projection_loader.param.yaml'),
                          allow_substs=True),
            sim_time,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'pointcloud_map',
            default_value=os.path.expanduser('~/WEAP/maps/ap1_map/pointcloud_map.pcd'),
            description='Point cloud map (.pcd)'),
        DeclareLaunchArgument(
            'pointcloud_map_metadata', default_value='none',
            description='Metadata yaml, only for a tiled map'),
        DeclareLaunchArgument(
            'map_projector_info',
            default_value=os.path.join(share, 'config', 'map_projector_info.yaml'),
            description='Map projector info (Local)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *set_vars,
        pointcloud_map_loader,
        map_projection_loader,
    ])
