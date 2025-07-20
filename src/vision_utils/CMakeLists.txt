cmake_minimum_required(VERSION 3.10)
project(vision_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(vision_deps REQUIRED)

set(VISION_DEPS_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vision_deps)

# Create a proper library instead of interface-only
add_library(vision_utils 
  src/config_loader.cpp
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

target_link_libraries(vision_utils)
ament_target_dependencies(vision_utils ament_index_cpp vision_deps)

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
ament_export_dependencies(ament_index_cpp vision_deps)

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
endif()

ament_package()
