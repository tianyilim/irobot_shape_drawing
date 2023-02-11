from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='irobot_command',
            namespace='',
            executable='undock_client',
            name='undock_client'
        ),
        Node(
            package='irobot_command',
            namespace='',
            executable='trace_seven',
            name='trace_seven'
        ),
        Node(
            package='path_viz',
            namespace='',
            executable='path_viz',
            name='path_viz'
        ),
    ])