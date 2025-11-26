from launch import LaunchDescription
from launch_ros.actions import Node

''' 
honestly unnecessary because you can just use a ros command to run the interface now using this launch command:
         ros2 launch ap1_launch ui_only.launch.py 
    if you want to run through regularly just run in another terminal:
        ros2 run ap1_control_interface system_interface
'''
def generate_launch_description():
    ui = Node(
        package='ap1_control_interface',
        executable='system_interface',
        name='ap1_control_interface',
        output='screen',
    )

    return LaunchDescription([ui])
