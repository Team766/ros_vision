#include <rclcpp/rclcpp.hpp>
#include "usb_camera/camera_publisher.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}