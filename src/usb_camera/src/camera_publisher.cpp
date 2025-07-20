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

#include "usb_camera/camera_publisher.hpp"
#include "usb_camera/opencv_camera.hpp"

CameraPublisher::CameraPublisher() : Node("camera_publisher") {
  // Create real OpenCV camera for production use
  camera_ = std::make_unique<OpenCVCamera>();
  
  // Declare parameters
  this->declare_parameter<int>("camera_idx", 0);
  int camera_idx = this->get_parameter("camera_idx").as_int();

  this->declare_parameter<std::string>("camera_serial", "N/A");
  camera_serial_ = this->get_parameter("camera_serial").as_string();

  this->declare_parameter<std::string>("topic_name", "camera/image_raw");
  topic_name_ = this->get_parameter("topic_name").as_string();

  initializeCamera(camera_idx);

  // Use higher frequency timer for more responsive capture
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(8),  // ~125fps to stay ahead of camera
      std::bind(&CameraPublisher::timerCallback, this));

  frame_count_ = 0;
  last_fps_time_ = this->now();
}

CameraPublisher::CameraPublisher(std::unique_ptr<CameraInterface> camera) 
    : Node("camera_publisher"), camera_(std::move(camera)) {
  // Declare parameters with defaults for testing
  this->declare_parameter<int>("camera_idx", 0);
  this->declare_parameter<std::string>("camera_serial", "TEST_CAMERA");
  this->declare_parameter<std::string>("topic_name", "camera/image_raw");

  camera_serial_ = this->get_parameter("camera_serial").as_string();
  topic_name_ = this->get_parameter("topic_name").as_string();

  // Check if injected camera is opened - throw if not
  if (!camera_->isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Injected camera is not opened");
    throw std::runtime_error("Camera not opened");
  }

  RCLCPP_INFO(this->get_logger(), "Camera publisher initialized with injected camera");
  RCLCPP_INFO(this->get_logger(), "Publishing on Topic: '%s'", topic_name_.c_str());

  // Use higher frequency timer for more responsive capture
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(8),  // ~125fps to stay ahead of camera
      std::bind(&CameraPublisher::timerCallback, this));

  frame_count_ = 0;
  last_fps_time_ = this->now();
}

CameraPublisher::~CameraPublisher() {
  if (image_pub_queue_) {
    image_pub_queue_->stop();
  }
}

void CameraPublisher::init() {
  // This can't be in the constructor because of the call to shared_from_this.
  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

  // Create publisher with optimized QoS to match subscriber settings
  // Convert rclcpp::QoS to rmw_qos_profile_t for image_transport
  auto qos = rclcpp::QoS(1)              // Queue depth of 1 for low latency
                 .best_effort()          // Use best effort for lower latency
                 .durability_volatile()  // No need to store messages
                 .deadline(std::chrono::milliseconds(
                     50));  // Match subscriber deadline

  publisher_ = it_->advertise(topic_name_, qos.get_rmw_qos_profile());

  // Reduce publisher queue size for lower latency
  image_pub_queue_ = std::make_shared<
      PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>(
      publisher_, 1);  // Reduced from 2 to 1 for lower latency
}

void CameraPublisher::timerCallback() {
  // Capture frame as close to hardware timestamp as possible
  cv::Mat frame;
  if (!camera_->read(frame)) {
    RCLCPP_WARN(this->get_logger(), "Failed to capture frame from camera");
    return;
  }

  // Timestamp immediately after frame capture
  rclcpp::Time capture_time = this->now();

  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Captured empty frame");
    return;
  }

  auto msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  msg->header.stamp = capture_time;
  msg->header.frame_id = "camera_frame";

  // Measure and log capture-to-publish latency periodically
  frame_count_++;
  if (frame_count_ % 100 == 0) {
    auto current_time = this->now();
    auto publish_latency = (current_time - capture_time).seconds() * 1000.0;
    auto fps_interval = (current_time - last_fps_time_).seconds();
    double fps = 100.0 / fps_interval;

    RCLCPP_DEBUG(this->get_logger(),
                "Camera stats - FPS: %.1f, Capture->Publish latency: %.2f ms",
                fps, publish_latency);
    last_fps_time_ = current_time;
  }

  image_pub_queue_->enqueue(msg);
}

void CameraPublisher::initializeCamera(int camera_idx) {
  RCLCPP_INFO(this->get_logger(), "Opening camera on idx: '%d'", camera_idx);
  
  if (!camera_->open(camera_idx, cv::CAP_V4L2)) {
    RCLCPP_ERROR(this->get_logger(), "Could not open camera");
    throw std::runtime_error("Camera not opened");
  }

  // Optimize camera settings for low latency
  int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  camera_->set(cv::CAP_PROP_FOURCC, fourcc);
  camera_->set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  camera_->set(cv::CAP_PROP_FRAME_HEIGHT, 800);
  camera_->set(cv::CAP_PROP_CONVERT_RGB, true);

  // Critical latency optimizations
  camera_->set(cv::CAP_PROP_BUFFERSIZE, 1);  // Minimize V4L2 buffer to 1 frame
  camera_->set(cv::CAP_PROP_FPS, 100);       // Explicitly set target FPS

  RCLCPP_INFO(this->get_logger(), "Width: '%d'",
              static_cast<int>(camera_->get(cv::CAP_PROP_FRAME_WIDTH)));
  RCLCPP_INFO(this->get_logger(), "Height: '%d'",
              static_cast<int>(camera_->get(cv::CAP_PROP_FRAME_HEIGHT)));
  RCLCPP_INFO(this->get_logger(), "FPS: '%d'",
              static_cast<int>(camera_->get(cv::CAP_PROP_FPS)));
  RCLCPP_INFO(this->get_logger(), "Buffer Size: '%d'",
              static_cast<int>(camera_->get(cv::CAP_PROP_BUFFERSIZE)));
  RCLCPP_INFO(this->get_logger(), "Publishing on Topic: '%s'",
              topic_name_.c_str());
}
