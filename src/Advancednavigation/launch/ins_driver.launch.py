#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver = Node(
        package='ros2-driver',
        executable='adnav_driver',
        name='advanced_navigation_driver',
        output='screen'
    )
    return LaunchDescription([driver])
