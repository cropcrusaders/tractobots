cmake_minimum_required(VERSION 3.10)
project(AgIsoStackPlusPlus)

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
)

target_link_libraries(isobus PUBLIC pthread)

install(TARGETS isobus
  EXPORT isobus-export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)


