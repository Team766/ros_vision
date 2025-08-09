cmake_minimum_required(VERSION 3.10)
project(vision_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_deps REQUIRED)
find_package(OpenCV REQUIRED)

set(VISION_DEPS_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vision_deps)

# Create a proper library instead of interface-only
add_library(vision_utils 
  src/config_loader.cpp
  src/rotation_utils.cpp
  src/process_scheduler.cpp
)

# Ensure position-independent code for shared library linking
set_target_properties(vision_utils PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(vision_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${VISION_DEPS_INSTALL_DIR}/json-install/include/
)

target_link_libraries(vision_utils ${OpenCV_LIBRARIES})
ament_target_dependencies(vision_utils ament_index_cpp rclcpp vision_deps OpenCV)

# Demo executable
add_executable(rotation_demo
  src/rotation_demo.cpp
)

target_include_directories(rotation_demo
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(rotation_demo
  vision_utils
)

ament_target_dependencies(rotation_demo
  ament_index_cpp
  vision_deps
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS vision_utils
  EXPORT vision_utils-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install demo executable separately (not part of exported targets)
install(
  TARGETS rotation_demo
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS timing_report.py
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(vision_utils-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp rclcpp vision_deps)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_config_loader
    test/test_config_loader.cpp
  )
  
  target_include_directories(test_config_loader
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${VISION_DEPS_INSTALL_DIR}/json-install/include/
  )
  
  target_link_libraries(test_config_loader
    vision_utils
  )
  
  ament_target_dependencies(test_config_loader
    ament_index_cpp
    vision_deps
  )
  
  ament_add_gtest(test_config_loader_utilities
    test/test_config_loader_utilities.cpp
  )
  
  target_include_directories(test_config_loader_utilities
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${VISION_DEPS_INSTALL_DIR}/json-install/include/
  )
  
  target_link_libraries(test_config_loader_utilities
    vision_utils
  )
  
  ament_target_dependencies(test_config_loader_utilities
    ament_index_cpp
    vision_deps
  )
  
  ament_add_gtest(test_rotation_utils
    test/test_rotation_utils.cpp
  )
  
  target_include_directories(test_rotation_utils
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  
  target_link_libraries(test_rotation_utils
    vision_utils
  )
  
  ament_target_dependencies(test_rotation_utils
    ament_index_cpp
    vision_deps
  )
endif()

# Clang-format targets
file(GLOB_RECURSE VISION_UTILS_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*/*.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test/*.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example_usage/src/*.cpp
)

add_custom_target(format_all
    COMMAND clang-format -i -style=Google ${VISION_UTILS_SOURCE_FILES}
    COMMENT "Running clang-format on vision_utils source files"
    VERBATIM
)

add_custom_target(format_check
    COMMAND clang-format --dry-run -Werror -style=Google ${VISION_UTILS_SOURCE_FILES}
    COMMENT "Checking clang-format compliance on vision_utils source files"
    VERBATIM
)

ament_package()
