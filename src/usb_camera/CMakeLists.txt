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

# Add library for camera implementations
add_library(usb_camera_lib 
  src/camera_publisher.cpp 
  src/opencv_camera.cpp
)
target_include_directories(usb_camera_lib PUBLIC include)
ament_target_dependencies(usb_camera_lib rclcpp sensor_msgs cv_bridge image_transport OpenCV vision_utils vision_deps)

add_executable(usb_camera_node src/usb_camera_node.cpp)
target_link_libraries(usb_camera_node usb_camera_lib)
ament_target_dependencies(usb_camera_node rclcpp)

add_executable(image_processor_node src/image_processor_node.cpp)
ament_target_dependencies(image_processor_node rclcpp sensor_msgs cv_bridge OpenCV vision_deps)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Disable all linting tests to focus on functionality
  set(ament_cmake_copyright_FOUND FALSE)
  set(ament_cmake_cpplint_FOUND FALSE)
  set(ament_cmake_uncrustify_FOUND FALSE)
  set(ament_cmake_lint_cmake_FOUND FALSE)
  set(ament_cmake_xmllint_FOUND FALSE)
  set(ament_cmake_cppcheck_FOUND FALSE)
  
  # Unit tests only
  ament_add_gtest(test_camera_publisher 
    test/test_camera_publisher.cpp
    test/mock_camera.cpp
  )
  target_include_directories(test_camera_publisher PRIVATE include test)
  target_link_libraries(test_camera_publisher usb_camera_lib)
  ament_target_dependencies(test_camera_publisher rclcpp sensor_msgs cv_bridge OpenCV)
endif()

install(TARGETS
  usb_camera_node
  image_processor_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  usb_camera_lib
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include/)


ament_package()
