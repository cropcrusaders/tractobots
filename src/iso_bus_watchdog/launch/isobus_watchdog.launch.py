#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    watchdog = Node(
        package='iso_bus_watchdog',
        executable='iso_bus_watchdog_node',
        name='iso_bus_watchdog',
        output='screen'
    )
    return LaunchDescription([watchdog])
