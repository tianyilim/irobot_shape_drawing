from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
from numpy import pi

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch', 'create3_gazebo.launch.py')
            ),
            launch_arguments={'use_gazebo_gui':'false'}.items()
        ),
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
        Node(
            package='irobot_command',
            namespace='',
            executable='trace_s',
            name='trace_s',
            parameters=[
                {
                    "WAYPOINT_TOL": 0.05,
                    "V_K": 1.5,
                    "V_MAX": 1.5,
                    "W_MAX": pi/2,
                }
            ]
        ),
    ])