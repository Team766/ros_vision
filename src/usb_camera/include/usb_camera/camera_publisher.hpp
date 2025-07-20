#ifndef USB_CAMERA__CAMERA_PUBLISHER_HPP_
#define USB_CAMERA__CAMERA_PUBLISHER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "vision_utils/publisher_queue.hpp"

class CameraPublisher : public rclcpp::Node {
 public:
  CameraPublisher();
  ~CameraPublisher();
  
  void init();

 private:
  void timerCallback();

  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<
      PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>
      image_pub_queue_;
  image_transport::Publisher publisher_;

  std::string topic_name_;
  std::string camera_serial_;

  // Performance monitoring
  int frame_count_;
  rclcpp::Time last_fps_time_;
};

#endif  // USB_CAMERA__CAMERA_PUBLISHER_HPP_
