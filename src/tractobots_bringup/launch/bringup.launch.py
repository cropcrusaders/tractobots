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

    ins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros2-driver'), 'launch', 'ins_driver.launch.py'])
        )
    )

    ekf_params = PathJoinSubstitution([
        FindPackageShare('tractobots_bringup'), 'params', 'ekf.yaml'
    ])
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[ekf_params]
    )

    watchdog_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('iso_bus_watchdog'), 'launch', 'isobus_watchdog.launch.py'])
        )
    )

    group = GroupAction([
        PushRosNamespace(robot_name),
        ins_launch,
        ekf_node,
        watchdog_launch,
    ])

    return LaunchDescription([robot_name_arg, group])
