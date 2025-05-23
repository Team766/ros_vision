cmake_minimum_required(VERSION 3.10)
project(vision_utils)

find_package(ament_cmake REQUIRED)

add_library(vision_utils INTERFACE)

target_include_directories(vision_utils
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(
  DIRECTORY include/
  DESTINATION include
)

# add_custom_target(copy_headers ALL
#     COMMAND bash -c "mkdir -p ${CMAKE_INSTALL_PREFIX}/include && cp -r ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_INSTALL_PREFIX}/include"
#     COMMENT "Copying headers to build directory"
# )

# install(CODE "
#     execute_process(COMMAND bash -c \"
#         mkdir -p ${CMAKE_INSTALL_PREFIX}/include && \
#         cp -r ${CMAKE_CURRENT_SOURCE_DIR}/include/* ${CMAKE_INSTALL_PREFIX}/include/
#     \")
# ")

ament_export_include_directories(include)

ament_package()
