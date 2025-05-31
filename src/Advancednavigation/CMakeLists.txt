cmake_minimum_required(VERSION 3.22)
project(ros2-driver LANGUAGES CXX C)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED) # ★ Codex-edit
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB DRIVER_CPP_SOURCES
  "Src/*.cpp"
) # ★ Codex-edit

add_executable(adnav_driver
  ${DRIVER_CPP_SOURCES}
  Src/Rs232/rs232.c
  Src/an_packet_protocol.c
  Src/spatial_packets.c
  Src/NTRIP_Client/NTRIP/ntripclient.c
) # ★ Codex-edit
ament_target_dependencies(adnav_driver rclcpp std_msgs sensor_msgs diagnostic_msgs geometry_msgs)

install(TARGETS
adnav_driver
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
) # ★ Codex-edit

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
