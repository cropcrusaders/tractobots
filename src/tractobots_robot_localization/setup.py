#!/usr/bin/env python3
# ★ Codex-edit
from glob import glob
from setuptools import setup, find_packages

package_name = 'tractobots_robot_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'tf-transformations',
    ],
    zip_safe=True,
    maintainer='Nicholas Bass',
    maintainer_email='nckbass218@gmail.com',
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
