cmake_minimum_required(VERSION 3.8)
project(tractobots_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node rclcpp nav_msgs)

install(TARGETS planner_node
destination lib/${PROJECT_NAME})

ament_package()
