cmake_minimum_required(VERSION 3.5)
project(gcode_parser)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(gcode_reader_node
  Src/gcode_reader_node.cpp
  Src/gcode_parser.cpp
  Src/coordinate_conversion.cpp
)
set_target_properties(gcode_reader_node PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)

target_include_directories(gcode_reader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gcode_reader_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS gcode_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY Include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(gcode_parser_test
    tests/test_gcode_parser.cpp
    Src/gcode_parser.cpp
  )
  set_target_properties(gcode_parser_test PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
  target_include_directories(gcode_parser_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
  )
  target_compile_definitions(gcode_parser_test PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
  )

  ament_add_gtest(coordinate_conversion_test
    tests/test_coordinate_conversion.cpp
    Src/coordinate_conversion.cpp
  )
  set_target_properties(coordinate_conversion_test PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
  target_include_directories(coordinate_conversion_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
  )
endif()

ament_package()
