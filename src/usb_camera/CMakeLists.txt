cmake_minimum_required(VERSION 3.8)
project(usb_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_utils)
find_package(vision_deps)

add_executable(usb_camera_node src/usb_camera_node.cpp)
ament_target_dependencies(usb_camera_node rclcpp sensor_msgs cv_bridge image_transport OpenCV vision_utils vision_deps)

add_executable(image_processor_node src/image_processor_node.cpp)
ament_target_dependencies(image_processor_node rclcpp sensor_msgs cv_bridge OpenCV vision_deps)

install(TARGETS
  usb_camera_node
  image_processor_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
