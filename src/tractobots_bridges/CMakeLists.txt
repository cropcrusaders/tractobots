cmake_minimum_required(VERSION 3.8)
project(tractobots_bridges)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(agopengps_bridge src/agopengps_bridge.cpp)
ament_target_dependencies(agopengps_bridge rclcpp geometry_msgs)

add_library(nmea_bridge src/nmea_bridge.cpp)
ament_target_dependencies(nmea_bridge rclcpp sensor_msgs geometry_msgs)

install(TARGETS agopengps_bridge nmea_bridge
destination lib/${PROJECT_NAME})

ament_package()
