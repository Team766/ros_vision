include(ExternalProject)
include(ProcessorCount)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(vision_deps LANGUAGES CUDA CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(VISION_DEPS_INSTALL_DIR ${CMAKE_INSTALL_PREFIX} CACHE INTERNAL "")

find_package(ament_cmake REQUIRED)
find_package(CUDA REQUIRED)
find_package(glog REQUIRED)
find_package(GTest REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

ProcessorCount(NUM_PROCESSORS)
if(NUM_PROCESSORS EQUAL 0)
    set(NUM_PROCESSORS 1)
endif()

# Add opencv
set(OPENCV_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/opencv-install)
ExternalProject_Add(
    OpenCV
    PREFIX ${CMAKE_BINARY_DIR}/OpenCV
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.9.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_DIR} -DBUILD_opencv_python3=1
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel ${NUM_PROCESSORS}
    INSTALL_COMMAND ${CMAKE_COMMAND} --install .
)

# Add wpilib package
set(WPILIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/wpilib-install)
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
set(CCCL_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/CCCL-install)
ExternalProject_Add(
    CCCL 
    PREFIX ${CMAKE_BINARY_DIR}/cccl
    GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
    GIT_TAG v2.3.2
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND bash -c "mkdir -p ${CCCL_INSTALL_DIR}/include && cp -r <SOURCE_DIR>/* ${CCCL_INSTALL_DIR}/include/"
)

# Add json package
set(JSON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/json-install)
ExternalProject_Add(
    json
    PREFIX ${CMAKE_BINARY_DIR}/json
    GIT_REPOSITORY https://github.com/nlohmann/json
    GIT_TAG v3.11.3
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${JSON_INSTALL_DIR} -DJSON_BuildTests=OFF
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel ${NUM_PROCESSORS}
    INSTALL_COMMAND ${CMAKE_COMMAND} --install .
)

# add seasocks package
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

# Add 971 modified umich apriltag package
set(APRILTAG_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/apriltag-install)
ExternalProject_Add(
    apriltag
    PREFIX ${CMAKE_BINARY_DIR}/apriltag
    GIT_REPOSITORY https://github.com/cgpadwick/apriltag.git
    GIT_TAG 3.3.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${APRILTAG_INSTALL_DIR} -DOpenCV_DIR=${OPENCV_INSTALL_DIR}
    BUILD_COMMAND make -j${NUM_PROCESSORS}
    INSTALL_COMMAND make install
)

# Enfoce a build order.  The only one that is explicitly necessary is
# that OpenCV gets built before wpilib.  I just like to have things
# behave deterministically.
add_dependencies(wpilib OpenCV)
add_dependencies(CCCL wpilib)
add_dependencies(json CCCL)
add_dependencies(seasocks json)
add_dependencies(apriltag seasocks)

ament_package()

