#include "usb_camera/camera_publisher.hpp"

CameraPublisher::CameraPublisher() : Node("camera_publisher"), cap_() {
  // Declare parameters
  this->declare_parameter<int>("camera_idx", 0);
  int camera_idx = this->get_parameter("camera_idx").as_int();

  this->declare_parameter<std::string>("camera_serial", "N/A");
  camera_serial_ = this->get_parameter("camera_serial").as_string();

  this->declare_parameter<std::string>("topic_name", "camera/image_raw");
  topic_name_ = this->get_parameter("topic_name").as_string();

  RCLCPP_INFO(this->get_logger(), "Opening camera on idx: '%d'", camera_idx);
  cap_.open(camera_idx, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open camera");
    throw std::runtime_error("Camera not opened");
  }

  // Optimize camera settings for low latency
  int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  cap_.set(cv::CAP_PROP_FOURCC, fourcc);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
  cap_.set(cv::CAP_PROP_CONVERT_RGB, true);

  // Critical latency optimizations
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Minimize V4L2 buffer to 1 frame
  cap_.set(cv::CAP_PROP_FPS, 100);       // Explicitly set target FPS

  RCLCPP_INFO(this->get_logger(), "Width: '%d'",
              static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)));
  RCLCPP_INFO(this->get_logger(), "Height: '%d'",
              static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));
  RCLCPP_INFO(this->get_logger(), "FPS: '%d'",
              static_cast<int>(cap_.get(cv::CAP_PROP_FPS)));
  RCLCPP_INFO(this->get_logger(), "Buffer Size: '%d'",
              static_cast<int>(cap_.get(cv::CAP_PROP_BUFFERSIZE)));
  RCLCPP_INFO(this->get_logger(), "Publishing on Topic: '%s'",
              topic_name_.c_str());

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
  cap_ >> frame;

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
