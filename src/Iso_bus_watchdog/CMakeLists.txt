cmake_minimum_required(VERSION 3.16)
project(iso_bus_watchdog)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(../AgIsoStackPlusPlus ${CMAKE_CURRENT_BINARY_DIR}/AgIsoStackPlusPlus)

add_executable(iso_bus_watchdog_node
  src/iso_bus_watchdog_node.cpp
)

ament_target_dependencies(iso_bus_watchdog_node
  rclcpp
  std_msgs
)

# link pthreads for AgIsoStack++
target_link_libraries(iso_bus_watchdog_node
  Threads::Threads
  isobus
)

install(TARGETS iso_bus_watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
) # ★ Codex-edit

ament_package()
