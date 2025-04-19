#!/usr/bin/env python3
"""
Launch the robot_localization EKF node with the Tractobots‑specific
parameter file.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tractobots_robot_localization")
    ekf_yaml = Path(pkg_share, "params", "ekf_localization.yaml")

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml],
        remappings=[  # typical EKF remaps; adjust if your topic names differ
            ("/imu/data", "/imu"),           # IMU topic after republisher
            ("/gps/odom", "/gps/odom")       # navsat_transform output
        ],
    )

    return LaunchDescription([ekf])
