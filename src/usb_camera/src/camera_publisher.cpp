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

  // Load camera configuration from config file
  auto config_opt = vision_utils::ConfigLoader::getCameraConfig(camera_serial_);
  if (config_opt.has_value()) {
    camera_config_ = config_opt.value();
    RCLCPP_INFO(this->get_logger(), "Loaded camera config for serial: %s", camera_serial_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "No config found for camera serial: %s", camera_serial_.c_str());
    throw std::runtime_error("Camera configuration not found for serial: " + camera_serial_);
  }

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
  
  // Convert API preference string to OpenCV constant
  int api_preference;
  if (camera_config_.api_preference == "V4L2") {
    api_preference = cv::CAP_V4L2;
  } else if (camera_config_.api_preference == "ANY") {
    api_preference = cv::CAP_ANY;
  } else if (camera_config_.api_preference == "GSTREAMER") {
    api_preference = cv::CAP_GSTREAMER;
  } else if (camera_config_.api_preference == "FFMPEG") {
    api_preference = cv::CAP_FFMPEG;
  } else {
    // Default to V4L2
    api_preference = cv::CAP_V4L2;
  }
  
  if (!camera_->open(camera_idx, api_preference)) {
    RCLCPP_ERROR(this->get_logger(), "Could not open camera");
    throw std::runtime_error("Camera not opened");
  }

  // Apply camera configuration from config file
  applyCameraConfig(camera_config_);

  // Log actual camera settings after configuration
  int actual_width = static_cast<int>(camera_->get(cv::CAP_PROP_FRAME_WIDTH));
  int actual_height = static_cast<int>(camera_->get(cv::CAP_PROP_FRAME_HEIGHT));
  int actual_fps = static_cast<int>(camera_->get(cv::CAP_PROP_FPS));
  int actual_buffer_size = static_cast<int>(camera_->get(cv::CAP_PROP_BUFFERSIZE));

  RCLCPP_INFO(this->get_logger(), "Requested: %dx%d @ %d fps, Format: %s", 
              camera_config_.width, camera_config_.height, camera_config_.frame_rate, camera_config_.format.c_str());
  RCLCPP_INFO(this->get_logger(), "Actual: %dx%d @ %d fps, Buffer Size: %d", 
              actual_width, actual_height, actual_fps, actual_buffer_size);
  
  // Warn if actual settings differ significantly from requested
  if (actual_width != camera_config_.width) {
    RCLCPP_WARN(this->get_logger(), "Camera width differs: requested %d, actual %d", 
                camera_config_.width, actual_width);
  }
  if (actual_height != camera_config_.height) {
    RCLCPP_WARN(this->get_logger(), "Camera height differs: requested %d, actual %d", 
                camera_config_.height, actual_height);
  }
  if (actual_fps != camera_config_.frame_rate) {
    RCLCPP_WARN(this->get_logger(), "Camera FPS differs: requested %d, actual %d", 
                camera_config_.frame_rate, actual_fps);
  }

  RCLCPP_INFO(this->get_logger(), "API Preference: %s", camera_config_.api_preference.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing on Topic: '%s'",
              topic_name_.c_str());
}

void CameraPublisher::applyCameraConfig(const vision_utils::CameraConfig& config) {
  // Convert format string to OpenCV fourcc
  int fourcc;
  if (config.format == "MJPG" || config.format == "MJPEG") {
    fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  } else if (config.format == "YUYV") {
    fourcc = cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');
  } else if (config.format == "H264") {
    fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
  } else if (config.format == "BGR3") {
    fourcc = cv::VideoWriter::fourcc('B', 'G', 'R', '3');
  } else {
    // Default to MJPG
    fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  }
  
  // Set video format with error checking
  if (!camera_->set(cv::CAP_PROP_FOURCC, fourcc)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set camera format to %s (fourcc: %d)", 
                 config.format.c_str(), fourcc);
    throw std::runtime_error("Failed to set camera format: " + config.format);
  }
  
  // Set resolution with error checking
  if (!camera_->set(cv::CAP_PROP_FRAME_WIDTH, config.width)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set camera width to %d", config.width);
    throw std::runtime_error("Failed to set camera width to " + std::to_string(config.width));
  }
  
  if (!camera_->set(cv::CAP_PROP_FRAME_HEIGHT, config.height)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set camera height to %d", config.height);
    throw std::runtime_error("Failed to set camera height to " + std::to_string(config.height));
  }
  
  // Set frame rate with error checking
  if (!camera_->set(cv::CAP_PROP_FPS, config.frame_rate)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set camera frame rate to %d", config.frame_rate);
    throw std::runtime_error("Failed to set camera frame rate to " + std::to_string(config.frame_rate));
  }
  
  // Standard optimizations with error checking
  if (!camera_->set(cv::CAP_PROP_CONVERT_RGB, true)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set camera RGB conversion - continuing anyway");
  }
  
  if (!camera_->set(cv::CAP_PROP_BUFFERSIZE, 1)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set camera buffer size to 1 - continuing anyway");
  }
}
