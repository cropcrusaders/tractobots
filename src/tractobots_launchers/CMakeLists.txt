cmake_minimum_required(VERSION 3.5)
project(tractobots_launchers)

# No code to compile – just install Python launch scripts
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
