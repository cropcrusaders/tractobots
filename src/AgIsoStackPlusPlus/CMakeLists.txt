cmake_minimum_required(VERSION 3.22)
set(CMAKE_WARN_DEPRECATED FALSE) # ★ Codex-edit
project(AgIsoStackPlusPlus LANGUAGES CXX) # ★ Codex-edit

find_package(ament_cmake REQUIRED)

file(GLOB ISOBUS_CORE_SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Isobus/isobus/*.cpp") # ★ Codex-edit
file(GLOB HARDWARE_SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Isobus/hardware_integration/*.cpp")

file(GLOB UTILITY_SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/utility/*.cpp")

add_library(isobus
  ${ISOBUS_CORE_SRC}
  ${HARDWARE_SRC}
  ${UTILITY_SRC}
) # ★ Codex-edit

# Public headers
target_include_directories(isobus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # ★ Codex-edit
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(isobus PUBLIC pthread)

install(TARGETS isobus
  EXPORT export_isobus
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include) # ★ Codex-edit
ament_export_libraries(isobus)
ament_export_targets(export_isobus HAS_LIBRARY_TARGET)

ament_package()
