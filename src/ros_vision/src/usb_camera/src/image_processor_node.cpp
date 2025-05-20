#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "vision_utils/publisher_queue.hpp"

class ImageProcessor : public rclcpp::Node {
public:
  ImageProcessor() : Node("image_processor") {
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto start = std::chrono::high_resolution_clock::now();

    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Scalar mean_scalar = cv::mean(img);
    double mean_intensity =
        (mean_scalar[0] + mean_scalar[1] + mean_scalar[2]) / 3.0;

    auto end = std::chrono::high_resolution_clock::now();
    auto processing_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();

    RCLCPP_INFO(this->get_logger(),
                "Mean Intensity: %.2f, Processing Time: %ld ms", mean_intensity,
                processing_time);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}