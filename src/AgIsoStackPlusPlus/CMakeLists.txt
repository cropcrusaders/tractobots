cmake_minimum_required(VERSION 3.22)
set(CMAKE_WARN_DEPRECATED FALSE) # ★ Codex-edit
project(AgIsoStackPlusPlus LANGUAGES CXX C)

find_package(ament_cmake REQUIRED)

add_library(isobus
  src/Isobus/isobus/can_message.cpp
  src/Isobus/isobus/can_message_data.cpp
  src/Isobus/isobus/can_message_frame.cpp
  src/Isobus/isobus/can_network_manager.cpp
  src/Isobus/hardware_integration/socket_can_interface.cpp
  src/Isobus/hardware_integration/can_hardware_interface.cpp
)

# Public headers
target_include_directories(isobus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(isobus PUBLIC pthread)

install(TARGETS isobus
  EXPORT export_isobus
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(isobus)
ament_export_targets(export_isobus HAS_LIBRARY_TARGET)

ament_package()
