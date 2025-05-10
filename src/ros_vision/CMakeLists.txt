cmake_minimum_required(VERSION 3.16)
project(ros_vision)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

include(ExternalProject)
include(ProcessorCount)

ProcessorCount(NUM_PROCESSORS)
if(NUM_PROCESSORS EQUAL 0)
    set(NUM_PROCESSORS 1)
endif()

# External project build (corrected)
ExternalProject_Add(vision_deps_external
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/vision_deps_build
    INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/vision_deps_install
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DCMAKE_BUILD_TYPE=Release
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel ${NUM_PROCESSORS}
    INSTALL_COMMAND ${CMAKE_COMMAND} --install .
)

ExternalProject_Get_Property(vision_deps_external INSTALL_DIR)

# Explicitly pass to usb_camera as cache var
set(VISION_DEPS_INSTALL_DIR ${INSTALL_DIR} CACHE PATH "vision_deps installation path")

include_directories(${VISION_DEPS_INSTALL_DIR}/seasocks-install/include)
link_directories(${VISION_DEPS_INSTALL_DIR}/seasocks-install/lib)

# Pass this clearly to your subdirectory
add_subdirectory(src/usb_camera)
add_subdirectory(src/seasocks_viewer)

ament_package()
