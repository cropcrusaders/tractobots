cmake_minimum_required(VERSION 3.8)
project(tractobots_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp nav_msgs geometry_msgs)

install(TARGETS controller_node
destination lib/${PROJECT_NAME})

ament_package()
