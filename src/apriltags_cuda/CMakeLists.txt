cmake_minimum_required(VERSION 3.16)

set(CMAKE_CUDA_COMPILER /usr/bin/clang++-17 CACHE STRING "CUDA compiler" FORCE)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-17 CACHE STRING "CXX compiler" FORCE)

project(apriltags_cuda LANGUAGES C CUDA CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wno-deprecated-anon-enum-enum-conversion)
add_compile_options(-Wno-c99-extensions)
add_compile_options(-Wno-unused-parameter)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 -g -gdwarf-4 -O0")
else()
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 -O3")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_utils REQUIRED)
find_package(vision_deps REQUIRED)
find_package(glog REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

#message(STATUS "prefix is ${CMAKE_INSTALL_PREFIX}")
set(VISION_DEPS_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vision_deps)
message(STATUS "VISION_DEPS_INSTALL_DIR is ${VISION_DEPS_INSTALL_DIR}")


# Use the propagated install directory from parent (ros_vision)
include_directories(
  ${VISION_DEPS_INSTALL_DIR}/CCCL-install/include/cub
  ${VISION_DEPS_INSTALL_DIR}/CCCL-install/include/libcudacxx/include
  ${VISION_DEPS_INSTALL_DIR}/CCCL-install/include/thrust
  ${VISION_DEPS_INSTALL_DIR}/opencv-install/include/opencv4
  ${VISION_DEPS_INSTALL_DIR}/wpilib-install/include/ntcore
  ${VISION_DEPS_INSTALL_DIR}/wpilib-install/include/wpiutil
  ${VISION_DEPS_INSTALL_DIR}/json-install/include/
  ${VISION_DEPS_INSTALL_DIR}/apriltag-install/include/apriltag
  ${VISION_DEPS_INSTALL_DIR}/apriltag-install/include/apriltag/common
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TagDetection.msg"
  "msg/TagDetectionArray.msg"
)

set(CUDA_LIB_SOURCES 
    src/apriltag_detect.cu 
    src/apriltag_gpu.cu
    src/cuda_frc971.cu
    src/labeling_allegretti_2019_BKE.cu
    src/line_fit_filter.cu
    src/points.cu
    src/threshold.cu
    src/apriltag_utils.cu
    src/apriltags_cuda_detector.cu
    src/DoubleArraySender.cpp
    src/NetworkTablesUtil.cpp)

# Add a library with the above source files
add_library(apriltags_cuda_core SHARED ${CUDA_LIB_SOURCES})

# Add dependencies for the core library since it now includes ROS-dependent code
ament_target_dependencies(apriltags_cuda_core rclcpp sensor_msgs cv_bridge image_transport vision_deps vision_utils rosidl_default_runtime)

#add_dependencies(apriltags_cuda_core vision_deps_external vision_utils)

add_executable(apriltags_cuda_node src/apriltags_cuda_node.cu)
target_include_directories(apriltags_cuda_node PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_dependencies(apriltags_cuda_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

ament_target_dependencies(apriltags_cuda_node rclcpp sensor_msgs cv_bridge image_transport vision_deps vision_utils rosidl_default_runtime)
target_link_libraries(apriltags_cuda_node
    apriltags_cuda_core
    apriltags_cuda__rosidl_typesupport_cpp
    ${VISION_DEPS_INSTALL_DIR}/apriltag-install/lib/libapriltag.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_core.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgproc.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_highgui.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_videoio.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgcodecs.so
    ${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib/libntcore.so
    ${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib/libwpiutil.so
    glog::glog)

set_target_properties(apriltags_cuda_node PROPERTIES
    BUILD_RPATH "${VISION_DEPS_INSTALL_DIR}/apriltag-install/lib;${VISION_DEPS_INSTALL_DIR}/opencv-install/lib;${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib"
    INSTALL_RPATH "${VISION_DEPS_INSTALL_DIR}/apriltag-install/lib;${VISION_DEPS_INSTALL_DIR}/opencv-install/lib;${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS apriltags_cuda_node
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS apriltags_cuda_core
    LIBRARY DESTINATION lib)

# Install test data
install(DIRECTORY test/data/
    DESTINATION share/${PROJECT_NAME}/test/data)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(gpu_detector_test test/gpu_detector_test.cu)
  target_include_directories(gpu_detector_test PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISION_DEPS_INSTALL_DIR}/CCCL-install/include/cub
    ${VISION_DEPS_INSTALL_DIR}/CCCL-install/include/libcudacxx/include
    ${VISION_DEPS_INSTALL_DIR}/CCCL-install/include/thrust
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/include/opencv4
    ${VISION_DEPS_INSTALL_DIR}/wpilib-install/include/ntcore
    ${VISION_DEPS_INSTALL_DIR}/wpilib-install/include/wpiutil
    ${VISION_DEPS_INSTALL_DIR}/json-install/include/
    ${VISION_DEPS_INSTALL_DIR}/apriltag-install/include/apriltag
    ${VISION_DEPS_INSTALL_DIR}/apriltag-install/include/apriltag/common
  )
  ament_target_dependencies(gpu_detector_test 
    rclcpp 
    sensor_msgs 
    cv_bridge 
    image_transport 
    vision_deps 
    vision_utils 
    rosidl_default_runtime
    ament_index_cpp
  )
  target_link_libraries(gpu_detector_test
    apriltags_cuda_core
    apriltags_cuda__rosidl_typesupport_cpp
    ${VISION_DEPS_INSTALL_DIR}/apriltag-install/lib/libapriltag.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_core.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgproc.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_highgui.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_videoio.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgcodecs.so
    ${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib/libntcore.so
    ${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib/libwpiutil.so
    glog::glog
  )
  set_target_properties(gpu_detector_test PROPERTIES
    BUILD_RPATH "${VISION_DEPS_INSTALL_DIR}/apriltag-install/lib;${VISION_DEPS_INSTALL_DIR}/opencv-install/lib;${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib"
    INSTALL_RPATH "${VISION_DEPS_INSTALL_DIR}/apriltag-install/lib;${VISION_DEPS_INSTALL_DIR}/opencv-install/lib;${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )
  
  # Set working directory for test to find data files
  set_tests_properties(gpu_detector_test PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
