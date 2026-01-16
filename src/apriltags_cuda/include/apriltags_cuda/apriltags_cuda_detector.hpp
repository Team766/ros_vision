#pragma once

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <image_transport/image_transport.hpp>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "apriltags_cuda/DoubleArraySender.h"
#include "apriltags_cuda/apriltag_gpu.h"
#include "apriltags_cuda/apriltag_utils.h"
#include "apriltags_cuda/msg/tag_detection.hpp"
#include "apriltags_cuda/msg/tag_detection_array.hpp"
#include "apriltags_cuda/tag_detection_msg_helpers.hpp"
#include "vision_utils/config_loader.hpp"
#include "vision_utils/process_scheduler.hpp"
#include "vision_utils/publisher_queue.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/zarray.h"
}

#define TAGSIZE 0.1651  // tag size, in meters

namespace fs = std::filesystem;
using json = nlohmann::json;

// Forward declaration for test classes
class TestableApriltagsDetectorSorting;

/**
 * @brief Main node for AprilTag detection using GPU acceleration.
 *
 * This node subscribes to camera images, detects AprilTags using a
 * GPU-accelerated detector, publishes tag detections to a custom ROS 2 message,
 * and sends pose data to network tables.
 *
 * Topics:
 *   - Subscribes: sensor_msgs::msg::Image (camera images)
 *   - Publishes:  apriltags_cuda::msg::TagDetectionArray (tag detections)
 *   - Publishes:  sensor_msgs::msg::Image (debug images)
 *
 * Parameters:
 *   - topic_name: Input image topic
 *   - camera_serial: Camera serial number
 *   - publish_images_to_topic: Output topic for debug images
 *   - publish_pose_to_topic: Output topic for tag detections
 */
class ApriltagsDetector : public rclcpp::Node {
  // Allow test classes to access private members
  friend class TestableApriltagsDetectorSorting;

 public:
  /**
   * @brief Construct the ApriltagsDetector node and initialize topics and
   * AprilTag detector.
   */
  ApriltagsDetector();

  /**
   * @brief Initialize publishers and image transport after construction.
   *
   * This must be called after the node is fully constructed.
   */
  virtual void init();

  /**
   * @brief Destructor. Cleans up detector and stops publisher queue.
   */
  ~ApriltagsDetector();

  /**
   * @brief Structure to hold detection data for sorting by distance.
   */
  struct DetectionData {
    apriltag_detection_t *det;  ///< Pointer to the raw detection
    cv::Vec3d camera_position;  ///< Position in camera coordinate frame
    cv::Mat robot_position;     ///< Position in robot coordinate frame
    double distance;            ///< Euclidean distance from camera origin
    double pose_error;          ///< Pose estimation error
  };

 protected:
  /**
   * @brief Protected constructor for testing that can bypass initialization
   */
  ApriltagsDetector(bool bypass_init);
  // Extrinsic parameters (protected for testing)
  cv::Mat extrinsic_rotation_;  ///< 3x3 rotation matrix from extrinsics
  cv::Mat extrinsic_offset_;    ///< 3x1 offset vector from extrinsics

  /**
   * @brief Transform a 3D point from camera frame to robot frame using
   * extrinsic parameters.
   *
   * This method applies the extrinsic rotation and translation to transform
   * coordinates from the camera's coordinate system to the robot's coordinate
   * system.
   *
   * @param camera_point 3D point in camera coordinate system as cv::Vec3d
   * @return 3D point in robot coordinate system as cv::Mat (3x1)
   */
  cv::Mat transformCameraToRobot(const cv::Vec3d &camera_point) const;

 private:
  /**
   * @brief Declare and initialize all ROS 2 topics and parameters.
   */
  void setup_topics();

  /**
   * @brief Set up the AprilTag detector, camera calibration, and network
   * tables.
   *
   * This method initializes the tag family, detector, loads camera calibration,
   * extrinsics, and network tables configuration, and creates the GPU detector.
   */
  void setup_apriltags();

  /**
   * @brief Load extrinsic parameters (rotation and offset) for this camera from
   * the system config file.
   *
   * Reads the rotation (3x3) and offset (3x1) from the extrinsics record for
   * the camera position, and stores them in OpenCV cv::Mat (rotation) and
   * cv::Vec3d (offset).
   */
  void get_extrinsic_params();

  /**
   * @brief Load network tables configuration from system config file.
   */
  void get_network_tables_params();

  /**
   * @brief Load camera calibration data from configuration files.
   *
   * @param cam Pointer to camera matrix structure to fill
   * @param dist Pointer to distortion coefficients structure to fill
   */
  void get_camera_calibration_data(frc971::apriltag::CameraMatrix *cam,
                                   frc971::apriltag::DistCoeffs *dist);

  /**
   * @brief Callback function for incoming image messages.
   *
   * This function is called when a new image message is received. It converts
   * the image to the YUYV format, runs the AprilTag detector, publishes the
   * detections, and sends pose data to the network tables.
   *
   * @param msg The incoming image message.
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Setup measurement and timing parameters.
   */
  void setup_measurement_params();

  /**
   * @brief Apply CPU pinning and real-time scheduling using vision_utils
   * ProcessScheduler.
   */
  void applyCpuPinningAndScheduling();

  // ROS 2 communication
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<
      PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>
      image_pub_queue_;
  image_transport::Publisher image_publisher_;
  rclcpp::Publisher<apriltags_cuda::msg::TagDetectionArray>::SharedPtr
      tag_detection_pub_;
  rclcpp::Publisher<apriltags_cuda::msg::TagDetectionArray>::SharedPtr
      tag_detection_camera_pub_;

  // Configuration parameters
  std::string publish_images_to_topic_;
  std::string publish_pose_to_topic_;
  std::string camera_serial_;
  std::string table_address_;
  std::string table_name_;

  // CPU pinning and scheduling parameters
  int pin_to_core_;
  int priority_;

  // Network tables
  std::shared_ptr<DoubleArraySender> tag_sender_;

  // AprilTag detection
  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;
  apriltag_detection_info_t info_;
  frc971::apriltag::GpuDetector *detector_;
  const char *tag_family_name_ = "tag36h11";

  // Measurement mode
  bool measurement_mode_ = false;
  std::string csv_path_;
  std::ofstream csv_file_;
};
