from setuptools import setup

package_name = 'tractobots_mission_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicholas Bass',
    maintainer_email='nckbass218@gmail.com',
    description='Web and GUI interface to start/stop missions',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'mission_ui_node = tractobots_mission_ui.mission_ui_node:main',
            'mission_gui_node = tractobots_mission_ui.mission_gui_node:main',
        ],
    },
)
