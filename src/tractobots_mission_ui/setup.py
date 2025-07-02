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
    install_requires=[
        'setuptools',
        'flask',
        'flask-socketio',
        'numpy',
        'geopandas',
        'shapely',
        'pyproj',
        'fiona',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='Nicholas Bass',
    maintainer_email='nckbass218@gmail.com',
    description='Modern GUI interfaces for Tractobots mission control',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'mission_ui_node = tractobots_mission_ui.mission_ui_node:main',
            'mission_gui_node = tractobots_mission_ui.mission_gui_node:main',
            'enhanced_gui = tractobots_mission_ui.enhanced_gui:main',
            'web_dashboard = tractobots_mission_ui.web_dashboard:main',
            'qt_gui = tractobots_mission_ui.qt_gui:main',
            'gui_launcher = tractobots_mission_ui.gui_launcher:main',
            'shapefile_manager = tractobots_mission_ui.shapefile_manager:main',
            'shapefile_gui = tractobots_mission_ui.shapefile_gui:main',
            'field_boundary_service = tractobots_mission_ui.field_boundary_service:main',
        ],
    },
)
