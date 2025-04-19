cmake_minimum_required(VERSION 3.5)
project(tractobots_navigation)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/driver.py        # will be ported below
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
