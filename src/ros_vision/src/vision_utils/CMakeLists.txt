cmake_minimum_required(VERSION 3.10)
project(vision_utils)

add_library(vision_utils INTERFACE)

target_include_directories(vision_utils
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
