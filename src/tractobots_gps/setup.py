from setuptools import setup

package_name = 'tractobots_gps'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],           # your Python module directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyler Laird',
    maintainer_email='KylerLaird@todo.com',
    description='ROS2 interface for the NV08C GPS receiver',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # adjust the module path if your code lives elsewhere
            'nv08c_node = tractobots_gps.NV08C_node:main',
        ],
    },
)
