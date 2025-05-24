cmake_minimum_required(VERSION 3.16)
project(ros_vision_launch)
find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
