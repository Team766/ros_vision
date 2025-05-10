cmake_minimum_required(VERSION 3.16)
project(seasocks_viewer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

# Use the propagated install directory from parent (ros_vision)
include_directories(${VISION_DEPS_INSTALL_DIR}/seasocks-install/include)
link_directories(${VISION_DEPS_INSTALL_DIR}/seasocks-install/lib)

add_executable(seasocks_viewer_node src/seasocks_viewer_node.cpp)

ament_target_dependencies(seasocks_viewer_node
  rclcpp sensor_msgs cv_bridge OpenCV Threads ZLIB
)

target_link_libraries(seasocks_viewer_node 
  seasocks
  Threads::Threads
  ZLIB::ZLIB)

install(TARGETS seasocks_viewer_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES src/index.html
  DESTINATION share/${PROJECT_NAME}/web)

ament_package()
