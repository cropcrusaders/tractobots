cmake_minimum_required(VERSION 3.22)
project(iso_bus_watchdog LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(AgIsoStackPlusPlus REQUIRED)


add_executable(iso_bus_watchdog_node
  src/iso_bus_watchdog_node.cpp
)

target_link_libraries(iso_bus_watchdog_node isobus) # ★ Codex-edit

ament_target_dependencies(iso_bus_watchdog_node
  rclcpp
  std_msgs
) # ★ Codex-edit

# link pthreads for AgIsoStack++
target_link_libraries(iso_bus_watchdog_node
  Threads::Threads
  isobus
)

install(TARGETS iso_bus_watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
) # ★ Codex-edit

ament_package()
