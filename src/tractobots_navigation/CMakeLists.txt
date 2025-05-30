cmake_minimum_required(VERSION 3.5)
project(tractobots_navigation)

find_package(ament_cmake_python REQUIRED)  # ★ Codex-edit
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})



ament_package()
