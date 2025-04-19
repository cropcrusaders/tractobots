#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="swri_transform",
             arguments=["0","0","0","0","0","0","map","odom","100"]),

        Node(package="swri_transform_util", executable="initialize_origin.py",
             name="initialize_origin",
             parameters=[{
               "local_xy_frame": "/map",
               "local_xy_origin": "tractofarm",
               "local_xy_origins": [
                   { "name":"tractofarm",
                     "latitude": 40.8882561488,
                     "longitude": -87.1996192638,
                     "altitude": 0.0,
                     "heading": 0.0 }
               ]
             }]),

        Node(package="mapviz", executable="mapviz", name="mapviz", output="screen"),
    ])
