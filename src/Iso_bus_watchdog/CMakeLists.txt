cmake_minimum_required(VERSION 3.16)
project(iso_bus_watchdog)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(isobus REQUIRED)
find_package(Threads REQUIRED)

add_executable(iso_bus_watchdog_node
  src/iso_bus_watchdog_node.cpp
)

ament_target_dependencies(iso_bus_watchdog_node
  rclcpp
  std_msgs
  isobus
)

# link pthreads for AgIsoStack++
target_link_libraries(iso_bus_watchdog_node
  Threads::Threads
)

install(TARGETS iso_bus_watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
