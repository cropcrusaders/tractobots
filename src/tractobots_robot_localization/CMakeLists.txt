cmake_minimum_required(VERSION 3.10)
project(tractobots_robot_localization)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
