cmake_minimum_required(VERSION 3.8)
project(tractobots_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(e_stop_node src/e_stop_node.cpp)
ament_target_dependencies(e_stop_node rclcpp diagnostic_updater std_msgs)

install(TARGETS e_stop_node
destination lib/${PROJECT_NAME})

ament_package()
