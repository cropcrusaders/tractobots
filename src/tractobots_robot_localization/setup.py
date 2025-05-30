#!/usr/bin/env python3
from setuptools import setup

package_name = 'tractobots_robot_localization'

setup(
    name=package_name,
    version='0.0.1',
    # Directory containing gps_parser, imu_publisher and pose_transformer modules
    packages=[package_name],
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # install the ROS package manifest
        ('share/' + package_name, ['package.xml']),
        # install launch & param folders so users can `ros2 launch` directly
        ('share/' + package_name + '/launch', ['launch/ekf.launch.py',
                                               'launch/navsat.launch.py']),
        ('share/' + package_name + '/params', ['params/ekf_localization.yaml',
                                               'params/navsat_transform.yaml']),
    ],
    install_requires=[
        'setuptools',
        'tf-transformations'      # needed by pose_transformer
    ],
    zip_safe=True,
    maintainer='Kyler Laird',
    maintainer_email='KylerLaird@todo.com',
    description='Helper nodes (GPS parser, IMU republisher, pose transformer) '
                'that wrap the robot_localization stack for Tractobots.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_parser = tractobots_robot_localization.gps_parser:main',
            'imu_publisher = tractobots_robot_localization.imu_publisher:main',
            'pose_transformer = tractobots_robot_localization.pose_transformer:main',
        ],
    },
)
