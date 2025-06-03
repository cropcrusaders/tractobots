cmake_minimum_required(VERSION 3.22)
project(tractobots_gps)

# find ament and rclpy
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# install the Python package
ament_python_install_package(${PROJECT_NAME})

# install any standalone scripts (if you prefer not to use entry_points)
# install(
#   PROGRAMS
#     scripts/NV08C_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
