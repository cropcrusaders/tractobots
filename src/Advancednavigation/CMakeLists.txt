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
find_package(tf2 REQUIRED) # ★ Codex-edit
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

file(GLOB DRIVER_SOURCES
  "Src/*.c"
  "Src/*.cpp"
  "Src/rs232/*.[cC]*"
  "Src/NTRIP_Client/**/*.c"
) # ★ Codex-edit

add_executable(adnav_driver
  ${DRIVER_SOURCES}
) # ★ Codex-edit
target_include_directories(adnav_driver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/rs232
) # ★ Codex-edit
ament_target_dependencies(adnav_driver
  rclcpp sensor_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  diagnostic_msgs            # ★ Codex-edit (new)
) # ★ Codex-edit

install(TARGETS
adnav_driver
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
) # ★ Codex-edit

if(BUILD_TESTING)

  #
  # ─── LINTING TOGGLE ────────────────────────────────────────────────────────────
  #

  # Turn OFF in CI while code style is being fixed; default ON for normal dev work.
  option(ENABLE_LINTING "Run ament_* linter tests" ON)

  if(ENABLE_LINTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

endif()

ament_package()
