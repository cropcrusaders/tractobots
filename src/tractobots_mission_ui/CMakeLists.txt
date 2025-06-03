cmake_minimum_required(VERSION 3.22)
project(tractobots_mission_ui)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
