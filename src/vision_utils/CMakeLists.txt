cmake_minimum_required(VERSION 3.10)
project(vision_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

set(VISION_DEPS_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vision_deps)
message(STATUS "VISION_DEPS_INSTALL_DIR is ${VISION_DEPS_INSTALL_DIR}")

add_library(vision_utils
  src/camera_utils.cpp
)

target_include_directories(vision_utils
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISION_DEPS_INSTALL_DIR}/json-install/include/
)

target_link_libraries(vision_utils
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
)

install(
  TARGETS vision_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(vision_utils)

# Format target for clang-format
file(GLOB_RECURSE ALL_SOURCE_FILES src/*.cpp src/*.h)
add_custom_target(format
    COMMAND clang-format -i -style=Google ${ALL_SOURCE_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_package()
