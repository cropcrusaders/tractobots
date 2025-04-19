#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    desc_share = get_package_share_directory("tractobots_description")
    nav_share  = get_package_share_directory("tractobots_robot_localization")

    return LaunchDescription([
        # robot TF tree (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                Path(desc_share, "launch", "tractobots_tf_tree.launch.py")
            )
        ),

        # helpers
        Node(package="tractobots_robot_localization", executable="gps_parser",
             name="gps_parser",  output="screen"),
        Node(package="tractobots_robot_localization", executable="imu_publisher",
             name="imu_publisher", output="screen"),

        # EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                Path(nav_share, "launch", "ekf.launch.py")
            )
        ),

        # navsat transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                Path(nav_share, "launch", "navsat.launch.py")
            )
        ),
    ])
