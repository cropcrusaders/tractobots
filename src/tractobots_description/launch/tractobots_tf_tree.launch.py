#!/usr/bin/env python3
"""
Launch file for Tractobots robot description and TF tree
Converts the XML launch file to ROS2 Python format
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('tractobots_description'),
            'urdf',
            'tractobots_library.urdf.xacro'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Joint State Publisher (for simulation/testing)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'rate': 5.0}],  # 5 Hz publish rate
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
    ])
