#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='tractor1')
    robot_name = LaunchConfiguration('robot_name')

    # Robot description and TF tree
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('tractobots_description'), 'launch', 'tractobots_tf_tree.launch.py'])
        )
    )

    # GPS parser and localization helpers
    gps_parser = Node(
        package='tractobots_robot_localization',
        executable='gps_parser',
        name='gps_parser',
        output='screen'
    )

    imu_publisher = Node(
        package='tractobots_robot_localization',
        executable='imu_publisher', 
        name='imu_publisher',
        output='screen'
    )

    # EKF for sensor fusion
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('tractobots_robot_localization'), 'launch', 'ekf.launch.py'])
        )
    )

    # NavSat transform for GPS
    navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('tractobots_robot_localization'), 'launch', 'navsat.launch.py'])
        )
    )

    return LaunchDescription([
        robot_name_arg,
        robot_description_launch,
        gps_parser,
        imu_publisher,
        ekf_launch,
        navsat_launch,
    ])
