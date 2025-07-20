// Copyright 2025 Team766
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef USB_CAMERA__CAMERA_PUBLISHER_HPP_
#define USB_CAMERA__CAMERA_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "vision_utils/publisher_queue.hpp"
#include "vision_utils/config_loader.hpp"
#include "usb_camera/camera_interface.hpp"

class CameraPublisher : public rclcpp::Node
{
public:
  // Constructor for production use (creates real OpenCV camera)
  CameraPublisher();

  // Constructor for testing (accepts injected camera interface)
  explicit CameraPublisher(std::unique_ptr<CameraInterface> camera);

  ~CameraPublisher();

  void init();

private:
  void timerCallback();
  void initializeCamera(int camera_idx);
  void applyCameraConfig(const vision_utils::CameraConfig& config);

  std::unique_ptr<CameraInterface> camera_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<
    PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>
    image_pub_queue_;
  image_transport::Publisher publisher_;

  std::string topic_name_;
  std::string camera_serial_;
  vision_utils::CameraConfig camera_config_;

  // Performance monitoring
  int frame_count_;
  rclcpp::Time last_fps_time_;
};

#endif  // USB_CAMERA__CAMERA_PUBLISHER_HPP_
