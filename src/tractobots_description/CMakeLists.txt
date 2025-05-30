cmake_minimum_required(VERSION 3.22)
project(tractobots_description LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
 # ★ Codex-edit

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
