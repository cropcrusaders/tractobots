#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("tractobots_description")
    nav_share  = get_package_share_directory("tractobots_robot_localization")
    rosbridge_share = get_package_share_directory('rosbridge_server')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rosbridge',
            default_value='true',
            description='Whether to launch the rosbridge server'
        ),

        # robot TF tree (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_share, 'launch', 'tractobots_tf_tree.launch.py')
            )
        ),

        # helpers
        Node(package='tractobots_robot_localization', executable='gps_parser',
             name='gps_parser',  output='screen'),
        Node(package='tractobots_robot_localization', executable='imu_publisher',
             name='imu_publisher', output='screen'),

        # EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'ekf.launch.py')
            )
        ),

        # navsat transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'navsat.launch.py')
            )
        ),

        # rosbridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                 os.path.join(rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')
            ),
            condition=IfCondition(LaunchConfiguration('use_rosbridge'))
        ),
    ])
