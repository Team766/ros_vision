cmake_minimum_required(VERSION 3.8)
project(game_piece_detection LANGUAGES CUDA CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(CUDAToolkit REQUIRED)

# On aarch64 (L4T/JetPack), libnvinfer transitively depends on DLA libraries
# (libnvdla_compiler.so, libcudla.so) which require Jetson-specific runtime libs
# (libnvos.so, libnvcudla.so) that only exist on real hardware. Allow these
# unresolved symbols at link time; they resolve at runtime on the Jetson.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")
endif()

set(VISION_DEPS_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vision_deps)
message(STATUS "VISION_DEPS_INSTALL_DIR is ${VISION_DEPS_INSTALL_DIR}")

include_directories(
  ${VISION_DEPS_INSTALL_DIR}/opencv-install/include/opencv4
  ${VISION_DEPS_INSTALL_DIR}/wpilib-install/include/ntcore
  ${VISION_DEPS_INSTALL_DIR}/wpilib-install/include/wpiutil
  ${VISION_DEPS_INSTALL_DIR}/json-install/include/
  ${CUDAToolkit_INCLUDE_DIRS}
  include
)

add_executable(gamepiece_detection_node src/game_piece_detection_node.cu)
ament_target_dependencies(gamepiece_detection_node rclcpp sensor_msgs cv_bridge image_transport vision_deps vision_utils)
target_link_libraries(gamepiece_detection_node
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_core.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgproc.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_highgui.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_videoio.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgcodecs.so
    ${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib/libntcore.so
    ${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib/libwpiutil.so
    nvinfer
    CUDA::cudart)
set_target_properties(gamepiece_detection_node PROPERTIES
    BUILD_RPATH "${VISION_DEPS_INSTALL_DIR}/opencv-install/lib;${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib/"
    INSTALL_RPATH "${VISION_DEPS_INSTALL_DIR}/opencv-install/lib;${VISION_DEPS_INSTALL_DIR}/wpilib-install/lib/"
    INSTALL_RPATH_USE_LINK_PATH TRUE)

# Detection test executable (standalone, no ROS dependencies)
add_executable(detection_test src/detection_test.cpp)
target_link_libraries(detection_test
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_core.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgproc.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_highgui.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgcodecs.so
    nvinfer
    CUDA::cudart)
set_target_properties(detection_test PROPERTIES
    BUILD_RPATH "${VISION_DEPS_INSTALL_DIR}/opencv-install/lib"
    INSTALL_RPATH "${VISION_DEPS_INSTALL_DIR}/opencv-install/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE)

# Inference benchmark executable
add_executable(inference_benchmark src/inference_benchmark.cpp)
target_link_libraries(inference_benchmark
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_core.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgproc.so
    ${VISION_DEPS_INSTALL_DIR}/opencv-install/lib/libopencv_imgcodecs.so
    nvinfer
    CUDA::cudart)
set_target_properties(inference_benchmark PROPERTIES
    BUILD_RPATH "${VISION_DEPS_INSTALL_DIR}/opencv-install/lib"
    INSTALL_RPATH "${VISION_DEPS_INSTALL_DIR}/opencv-install/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS gamepiece_detection_node detection_test inference_benchmark DESTINATION lib/${PROJECT_NAME})

# Install Python utils directory
install(DIRECTORY utils/ DESTINATION share/${PROJECT_NAME}/utils
  USE_SOURCE_PERMISSIONS
  PATTERN ".venv" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.egg-info" EXCLUDE
)

# Install data directory
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

# Install downloaded models (exclude scripts, venv, etc.)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models
  PATTERN ".venv" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.sh" EXCLUDE
  PATTERN "*.txt" EXCLUDE
  PATTERN "*.md" EXCLUDE
  PATTERN ".gitignore" EXCLUDE
)

ament_package()
