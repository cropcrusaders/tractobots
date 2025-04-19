#!/usr/bin/env python3
"""
Launch navsat_transform_node plus the helper nodes that feed it
(gps_parser, imu_publisher, pose_transformer).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tractobots_robot_localization")
    navsat_yaml = Path(pkg_share, "params", "navsat_transform.yaml")

    # ── Upstream robot_localization node ───────────────────────────
    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[navsat_yaml],
        remappings=[
            # Uses the messages published by our helper nodes
            ("/imu/data", "/imu"),
            ("/gps/fix", "/gps/fix"),
            ("/gps/vel", "/gps/vel"),
            ("/odometry/filtered", "/odometry/filtered"),
        ],
    )

    # ── Helper nodes (all Python, ported earlier) ─────────────────
    gps_parser = Node(
        package="tractobots_robot_localization",
        executable="gps_parser",
        name="gps_parser",
        output="screen",
    )

    imu_publisher = Node(
        package="tractobots_robot_localization",
        executable="imu_publisher",
        name="imu_publisher",
        output="screen",
    )

    pose_transformer = Node(
        package="tractobots_robot_localization",
        executable="pose_transformer",
        name="pose_transformer",
        output="screen",
        parameters=[{"from_frame": "base_link", "to_frame": "utm"}],
    )

    return LaunchDescription([
        gps_parser,
        imu_publisher,
        pose_transformer,
        navsat_transform,
    ])
