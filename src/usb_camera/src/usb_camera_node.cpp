#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <fstream>
#include <image_transport/image_transport.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "vision_utils/camera_utils.hpp"
#include "vision_utils/publisher_queue.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

class CameraPublisher : public rclcpp::Node {
 public:
  CameraPublisher() : Node("camera_publisher"), cap_() {
    // Decare parameters
    this->declare_parameter<int>("camera_idx", 0);
    int camera_idx = this->get_parameter("camera_idx").as_int();

    this->declare_parameter<std::string>("camera_serial", "N/A");
    camera_serial_ = this->get_parameter("camera_serial").as_string();

    this->declare_parameter<std::string>("topic_name", "camera/image_raw");
    topic_name_ = this->get_parameter("topic_name").as_string();

    setup_camera(camera_idx);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(16),  // ~60fps
        std::bind(&CameraPublisher::timerCallback, this));
  }

  ~CameraPublisher() { image_pub_queue_->stop(); }

  void init() {
    // This can't be in the constructor because of the call to shared_from_this.
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    publisher_ = it_->advertise(topic_name_, 10);
    image_pub_queue_ = std::make_shared<
        PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>(
        publisher_, 2);
  }

 private:
  void timerCallback() {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Captured empty frame");
      return;
    }

    auto msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_frame";  // <-- add this line

    image_pub_queue_->enqueue(msg);
  }

  void setup_camera(int camera_idx) {
    try {
      auto camera_data = vision_utils::get_camera_data(camera_serial_);
      width_ = camera_data.width;
      height_ = camera_data.height;
      frame_rate_ = camera_data.frame_rate;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Camera config error: %s", e.what());
      throw;
    }

    RCLCPP_INFO(this->get_logger(), "Opening camera on idx: '%d'", camera_idx);
    cap_.open(camera_idx, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open camera");
      throw std::runtime_error("Camera not opened");
    }

    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!cap_.set(cv::CAP_PROP_FOURCC, fourcc)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set FOURCC");
      throw std::runtime_error("Failed to set FOURCC");
    }
    if (!cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set FRAME_WIDTH");
      throw std::runtime_error("Failed to set FRAME_WIDTH");
    }
    if (!cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set FRAME_HEIGHT");
      throw std::runtime_error("Failed to set FRAME_HEIGHT");
    }
    if (!cap_.set(cv::CAP_PROP_FPS, frame_rate_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set FPS");
      throw std::runtime_error("Failed to set FPS");
    }
    if (!cap_.set(cv::CAP_PROP_CONVERT_RGB, true)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set CONVERT_RGB");
      throw std::runtime_error("Failed to set CONVERT_RGB");
    }

    RCLCPP_INFO(this->get_logger(), "Width: '%d'",
                static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)));
    RCLCPP_INFO(this->get_logger(), "Height: '%d'",
                static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));
    RCLCPP_INFO(this->get_logger(), "FPS: '%d'",
                static_cast<int>(cap_.get(cv::CAP_PROP_FPS)));
    RCLCPP_INFO(this->get_logger(), "Pubishing on Topic: '%s'",
                topic_name_.c_str());
  }

  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<
      PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>
      image_pub_queue_;
  image_transport::Publisher publisher_;

  std::string topic_name_;
  std::string camera_serial_;
  int width_;
  int height_;
  int frame_rate_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}