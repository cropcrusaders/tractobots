from setuptools import setup

package_name = 'tractobots_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/params', ['params/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyler Laird',
    maintainer_email='KylerLaird@todo.com',
    description='Launch files for bringing up Tractobots sensors and safety nodes',
    license='GPLv3',
)
