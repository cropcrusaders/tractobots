cmake_minimum_required(VERSION 3.5)
project(tractobots_launchers)

# No code to compile – just install Python launch scripts
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
