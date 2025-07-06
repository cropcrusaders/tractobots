cmake_minimum_required(VERSION 3.10)
project(tractobots_robot_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
