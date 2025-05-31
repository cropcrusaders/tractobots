#!/usr/bin/env python3
"""Launch the Nav2 stack with Tractobots-specific parameters."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('tractobots_nav2')
    params_file = Path(pkg_share, 'params', 'nav2.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                str(get_package_share_directory('nav2_bringup') + '/launch/navigation_launch.py')
            ]
        ),
        launch_arguments={'use_sim_time': 'false',
                          'autostart': 'true',
                          'params_file': str(params_file)}.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
