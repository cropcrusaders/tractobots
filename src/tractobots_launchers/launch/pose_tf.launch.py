#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="tractobots_robot_localization",
             executable="pose_transformer",
             name="pose_transformer",
             parameters=[{
                 "from_frame": "base_link",
                 "to_frame":   "utm"
             }],
             output="screen")
    ])
