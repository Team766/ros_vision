include(ExternalProject)
include(ProcessorCount)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(vision_deps LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# clang-format target (optional)
add_custom_target(format_all
    COMMAND clang-format -i -style=Google
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    COMMENT "Running clang-format"
)
