cmake_minimum_required(VERSION 3.8)
project(tractobots_vehicle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(vehicle_interface src/vehicle_interface.cpp)
ament_target_dependencies(vehicle_interface rclcpp)

install(TARGETS vehicle_interface
destination lib/${PROJECT_NAME})

ament_package()
