from glob import glob
from setuptools import setup

package_name = 'tractobots_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicholas Bass',
    maintainer_email='nicholasbass@crop-crusaders.com',
    description='Nav2 launch files and configuration for Tractobots.',
    license='GPLv3',
)
