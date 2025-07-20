#include "apriltags_cuda/apriltags_cuda_detector.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApriltagsDetector>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}