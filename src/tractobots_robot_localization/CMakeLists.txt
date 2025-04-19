cmake_minimum_required(VERSION 3.5)
project(tractobots_robot_localization)

# ── Find ament & rclpy ────────────────────────────────────────────────────────
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ── Install the Python package ────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# Install the three executable scripts
install(
  PROGRAMS
    scripts/gps_parser.py
    scripts/imu_publisher.py
    scripts/pose_transformer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch & parameter files
install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
