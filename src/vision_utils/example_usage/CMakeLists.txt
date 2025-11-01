# Example CMakeLists.txt for using vision_utils rotation utilities

cmake_minimum_required(VERSION 3.10)
project(rotation_example)

find_package(ament_cmake REQUIRED)
find_package(vision_utils REQUIRED)

# Example executable using rotation utilities
add_executable(rotation_example src/main.cpp)

target_include_directories(rotation_example
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(rotation_example
  vision_utils::vision_utils
)

ament_target_dependencies(rotation_example
  vision_utils
)

install(
  TARGETS rotation_example
  DESTINATION bin
)

ament_package()
