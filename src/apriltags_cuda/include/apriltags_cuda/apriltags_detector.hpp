#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <memory>
#include <nlohmann/json.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "vision_utils/publisher_queue.hpp"
#include "vision_utils/camera_utils.hpp"
#include "apriltag_gpu.h"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/zarray.h"
}

class ApriltagsDetector : public rclcpp::Node {
public:
  ApriltagsDetector();
  void init();
  ~ApriltagsDetector();
private:
  void get_extrinsic_params();
  void get_camera_calibration_data(frc971::apriltag::CameraMatrix* cam,
                                   frc971::apriltag::DistCoeffs* dist);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>> image_pub_queue_;
  image_transport::Publisher publisher_;
  std::string publish_to_topic_;
  std::string camera_serial_;
  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;
  apriltag_detection_info_t info_;
  frc971::apriltag::GpuDetector *detector_;
  const char *tag_family_name_ = "tag36h11";
};
