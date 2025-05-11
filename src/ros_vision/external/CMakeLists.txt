include(ExternalProject)
include(ProcessorCount)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(vision_deps LANGUAGES CUDA CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(CUDA REQUIRED)
find_package(glog REQUIRED)
find_package(GTest REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

ProcessorCount(NUM_PROCESSORS)
if(NUM_PROCESSORS EQUAL 0)
    set(NUM_PROCESSORS 1)
endif()

set(SEASOCKS_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/seasocks-install)

ExternalProject_Add(
    seasocks
    PREFIX ${CMAKE_BINARY_DIR}/seasocks
    GIT_REPOSITORY https://github.com/mattgodbolt/seasocks.git
    GIT_TAG v1.4.6
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${SEASOCKS_INSTALL_DIR}
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel ${NUM_PROCESSORS}
    INSTALL_COMMAND ${CMAKE_COMMAND} --install .
)

set(OPENCV_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/opencv-install)
ExternalProject_Add(
    OpenCV
    PREFIX ${CMAKE_BINARY_DIR}/OpenCV
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.9.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_DIR}
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel ${NUM_PROCESSORS}
    INSTALL_COMMAND ${CMAKE_COMMAND} --install .
)

# Add wpilib package
set(WPILIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/opencv-install)
ExternalProject_Add(
    wpilib
    PREFIX ${CMAKE_BINARY_DIR}/wpilib
    GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
    GIT_TAG v2024.3.2
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${WPILIB_INSTALL_DIR} -DWITH_JAVA=OFF -DWITH_GUI=OFF -DOpenCV_DIR=${OPENCV_INSTALL_DIR}/lib/cmake/opencv4
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel ${NUM_PROCESSORS}
    INSTALL_COMMAND ${CMAKE_COMMAND} --install .
)

# Add CCCL package
ExternalProject_Add(
    CCCL 
    PREFIX ${CMAKE_BINARY_DIR}/cccl
    GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
    GIT_TAG v2.3.2
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)

add_dependencies(wpilib OpenCV)

# clang-format target (optional)
add_custom_target(format_all
    COMMAND clang-format -i -style=Google
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    COMMENT "Running clang-format"
)
