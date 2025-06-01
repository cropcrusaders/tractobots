cmake_minimum_required(VERSION 3.22)
project(tractobots_description LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

option(ENABLE_LINTING "Run ament linters" ON)

if(ENABLE_LINTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
