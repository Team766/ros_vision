#pragma once

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

class CameraPublisher : public rclcpp::Node {
 public:
  CameraPublisher();
  ~CameraPublisher();
  void init();

 private:
  void timerCallback();
  void setup_camera(int camera_idx);

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
