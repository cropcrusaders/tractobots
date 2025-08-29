from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='tractobots_core', executable='e_stop_node'),
        Node(package='tractobots_vehicle', plugin='VehicleInterface'),
        Node(package='tractobots_planning', plugin='PlannerNode'),
        Node(package='tractobots_control', plugin='ControllerNode'),
        Node(package='tractobots_bridges', plugin='AgOpenGPSBridge'),
        Node(package='tractobots_bridges', plugin='NMEABridge'),
    ])
