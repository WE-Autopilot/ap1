from launch import LaunchDescription
from launch_ros.actions import Node

''' 
honestly unnecessary because you can just use a ros command to run the interface now using this launch command:
         ros2 launch ap1_bringup ui_setup.launch.py
    if you want to run through regularly just run in another terminal:
        ros2 run ap1_console console
'''
def generate_launch_description():
    console = Node(
        package='ap1_console',
        executable='console',
        name='ap1_console',
        output='screen',
    )

    return LaunchDescription([console])
