#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <image_transport/image_transport.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "DoubleArraySender.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "apriltags_cuda/msg/tag_detection.hpp"
#include "apriltags_cuda/msg/tag_detection_array.hpp"
#include "apriltags_cuda/tag_detection_msg_helpers.hpp"
#include "vision_utils/publisher_queue.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/zarray.h"
}

#define TAGSIZE 0.1651  // tag size, in meters

namespace fs = std::filesystem;
using json = nlohmann::json;

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
 public:
  /**
   * @brief Construct the ApriltagsDetector node and initialize topics and
   * AprilTag detector.
   */
  ApriltagsDetector()
      : Node("apriltags_detector"),
        tag_family_(nullptr),
        tag_detector_(nullptr) {
    setup_topics();
    get_network_tables_params();
    setup_apriltags();
  }

  /**
   * @brief Initialize publishers and image transport after construction.
   *
   * This must be called after the node is fully constructed.
   */
  void init() {
    // The object needs to be constructed before using shared_from_this, thus
    // it is broken off into another method.
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_publisher_ = it_->advertise(publish_images_to_topic_, 10);
    image_pub_queue_ = std::make_shared<
        PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>(
        image_publisher_, 2);
    tag_detection_pub_ =
        this->create_publisher<apriltags_cuda::msg::TagDetectionArray>(
            publish_pose_to_topic_, 10);
    // Add publisher for camera-frame poses
    tag_detection_camera_pub_ =
        this->create_publisher<apriltags_cuda::msg::TagDetectionArray>(
            publish_pose_to_topic_ + "_camera", 10);

    RCLCPP_INFO(this->get_logger(), "Publishing images on topic: %s",
                publish_images_to_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing pose on topic: %s",
                publish_pose_to_topic_.c_str());
  }

  /**
   * @brief Destructor. Cleans up detector and stops publisher queue.
   */
  ~ApriltagsDetector() {
    apriltag_detector_destroy(tag_detector_);
    teardown_tag_family(&tag_family_, tag_family_name_);
    delete detector_;
    image_pub_queue_->stop();
  }

 private:
  /**
   * @brief Declare and initialize all ROS 2 topics and parameters.
   */
  void setup_topics() {
    this->declare_parameter<std::string>("topic_name", "camera/image_raw");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    this->declare_parameter<std::string>("camera_serial", "N/A");
    camera_serial_ = this->get_parameter("camera_serial").as_string();

    this->declare_parameter<std::string>("publish_images_to_topic",
                                         "apriltags/images");
    publish_images_to_topic_ =
        this->get_parameter("publish_images_to_topic").as_string();

    this->declare_parameter<std::string>("publish_pose_to_topic",
                                         "camera/pose");
    publish_pose_to_topic_ =
        this->get_parameter("publish_pose_to_topic").as_string();

    // Create subscription with optimized QoS for low latency
    auto qos = rclcpp::QoS(1)      // Reduce queue depth for lower latency
                   .best_effort()  // Use best effort for lower latency
                   .durability_volatile()  // No need to store messages
                   .deadline(std::chrono::milliseconds(
                       50));  // Expect messages within 50ms

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, qos,
        std::bind(&ApriltagsDetector::imageCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to topic: %s with optimized QoS",
                topic_name.c_str());
  }

  /**
   * @brief Set up the AprilTag detector, camera calibration, and network
   * tables.
   *
   * This method initializes the tag family, detector, loads camera calibration,
   * extrinsics, and network tables configuration, and creates the GPU detector.
   */
  void setup_apriltags() {
    setup_tag_family(&tag_family_, tag_family_name_);
    tag_detector_ = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector_, tag_family_);

    tag_detector_->quad_decimate = 2.0;
    tag_detector_->quad_sigma = 0.0;
    tag_detector_->nthreads = 1;
    tag_detector_->debug = false;
    tag_detector_->refine_edges = true;
    tag_detector_->wp = workerpool_create(4);

    // Camera and detector setup
    frc971::apriltag::CameraMatrix cam;
    frc971::apriltag::DistCoeffs dist;
    get_camera_calibration_data(&cam, &dist);
    get_extrinsic_params();

    tag_sender_ = std::make_shared<DoubleArraySender>(
        camera_serial_, table_address_, table_name_);

    int frame_width = 1280;
    int frame_height = 800;

    auto start = std::chrono::high_resolution_clock::now();
    detector_ = new frc971::apriltag::GpuDetector(frame_width, frame_height,
                                                  tag_detector_, cam, dist);
    auto end = std::chrono::high_resolution_clock::now();
    auto processing_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();

    info_.tagsize = TAGSIZE;
    info_.fx = cam.fx;
    info_.fy = cam.fy;
    info_.cx = cam.cx;
    info_.cy = cam.cy;

    RCLCPP_INFO(this->get_logger(),
                "GPU Apriltag Detector created, took %ld ms", processing_time);
  }

  /**
   * @brief Load extrinsic parameters (rotation and offset) for this camera from
   * the system config file.
   *
   * Reads the rotation (3x3) and offset (3x1) from the extrinsics record for
   * the camera position, and stores them in OpenCV cv::Mat (rotation) and
   * cv::Vec3d (offset).
   */
  void get_extrinsic_params() {
    // Locate the system config file.
    fs::path config_path =
        ament_index_cpp::get_package_share_directory("vision_config_data");
    fs::path config_file = config_path / "data" / "system_config.json";

    if (!std::filesystem::exists(config_file)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find system config file at path: %s",
                   config_file.c_str());
      return;
    }

    // Load the parameters from the file.
    std::ifstream f(config_file);
    json data = json::parse(f);

    if (!data.contains("camera_mounted_positions")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find key \"camera_mounted_positions\" in system "
                   "config file");
      return;
    }
    if (!data.contains("extrinsics")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find key \"extrinsics\" in system config file");
      return;
    }

    std::string camera_position = "N/A";
    if (data["camera_mounted_positions"].contains(camera_serial_)) {
      camera_position =
          data["camera_mounted_positions"][camera_serial_].get<std::string>();
    } else {
      RCLCPP_ERROR(
          this->get_logger(),
          "Unable to find camera serial %s in camera_mounted_positions in "
          "system config file",
          camera_serial_.c_str());
      return;
    }
    if (!data["extrinsics"].contains(camera_position)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find camera position %s in extrinsics in system "
                   "config file",
                   camera_position.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Found camera position %s for camera serial %s",
                camera_position.c_str(), camera_serial_.c_str());

    // Read rotation and offset
    const auto &extr = data["extrinsics"][camera_position];
    if (!extr.contains("rotation") || !extr.contains("offset")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Extrinsics for %s missing 'rotation' or 'offset' field",
                   camera_position.c_str());
      return;
    }
    // Read rotation (should be 3x3 array)
    cv::Mat rotation(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rotation.at<double>(i, j) = extr["rotation"][i][j];
      }
    }
    // Read offset (should be 3x1 array)
    cv::Vec3d offset;
    for (int i = 0; i < 3; ++i) {
      offset[i] = extr["offset"][i];
    }
    extrinsic_rotation_ = rotation;
    extrinsic_offset_ = cv::Mat(offset);

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded extrinsics for %s: "
        "rotation=[[%f,%f,%f],[%f,%f,%f],[%f,%f,%f]], offset=[%f,%f,%f]",
        camera_position.c_str(), rotation.at<double>(0, 0),
        rotation.at<double>(0, 1), rotation.at<double>(0, 2),
        rotation.at<double>(1, 0), rotation.at<double>(1, 1),
        rotation.at<double>(1, 2), rotation.at<double>(2, 0),
        rotation.at<double>(2, 1), rotation.at<double>(2, 2), offset[0],
        offset[1], offset[2]);
  }

  void get_network_tables_params() {
    // Locate the system config file.
    fs::path config_path =
        ament_index_cpp::get_package_share_directory("vision_config_data");
    fs::path config_file = config_path / "data" / "system_config.json";

    if (!std::filesystem::exists(config_file)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find system config file at path: %s",
                   config_file.c_str());
    }

    // Load the parameters from the file.
    std::ifstream f(config_file);
    json data = json::parse(f);

    if (!data.contains("network_tables_config")) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Unable to find key \"network_tables_config\" in system config file");
    }
    if (!data["network_tables_config"].contains("table_address")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find key \"table_address\" in "
                   "\"network_tables_config\" record in system config file");
    }
    if (!data["network_tables_config"].contains("table_name")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find key \"table_name\" in "
                   "\"network_tables_config\" record in system config file");
    }

    data["network_tables_config"].at("table_address").get_to(table_address_);
    data["network_tables_config"].at("table_name").get_to(table_name_);

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded network tables parameters table_address: %s, table_name: %s",
        table_address_.c_str(), table_name_.c_str());

    return;
  }

  void get_camera_calibration_data(frc971::apriltag::CameraMatrix *cam,
                                   frc971::apriltag::DistCoeffs *dist) {
    // Locate the calibration file for the camera serial id that we are
    // processing.
    fs::path config_path =
        ament_index_cpp::get_package_share_directory("vision_config_data");
    std::ostringstream oss;
    oss << "calibrationmatrix_" << camera_serial_ << ".json";
    fs::path cal_file = config_path / "data" / "calibration" / oss.str();

    if (!std::filesystem::exists(cal_file)) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Unable to find calibration parameters for camera %s at path: %s",
          camera_serial_.c_str(), cal_file.c_str());
    }

    // Load the parameters from the file.
    std::ifstream f(cal_file);
    json data = json::parse(f);

    if (!data.contains("matrix")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find key \"matrix\" in calibration file");
    }
    if (!data.contains("disto")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find key \"disto\" in calibration file");
    }

    RCLCPP_INFO(this->get_logger(),
                "Loading camera calibration params from: %s", cal_file.c_str());

    // Setup Camera Matrix
    // Intrinsic Matrices are explained here:
    // https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
    cam->fx = data["matrix"][0][0];
    cam->fy = data["matrix"][1][1];
    cam->cx = data["matrix"][0][2];
    cam->cy = data["matrix"][1][2];

    // Setup Distortion Coefficients
    // OpenCV writes them out in the order specified here:
    // https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
    dist->k1 = data["disto"][0][0];
    dist->k2 = data["disto"][0][1];
    dist->p1 = data["disto"][0][2];
    dist->p2 = data["disto"][0][3];
    dist->k3 = data["disto"][0][4];

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded camera matrix parameters: fx: %f, fy: %f, cx: %f, cy: %f",
        cam->fx, cam->fy, cam->cx, cam->cy);
    RCLCPP_INFO(
        this->get_logger(),
        "Loaded distortion coeffs k1: %f, k2: %f, p1: %f, p2: %f, k3: %f",
        dist->k1, dist->k2, dist->p1, dist->p2, dist->k3);
  }

  /**
   * @brief Callback function for incoming image messages.
   *
   * This function is called when a new image message is received. It converts
   * the image to the YUYV format, runs the AprilTag detector, publishes the
   * detections, and sends pose data to the network tables.
   *
   * @param msg The incoming image message.
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat yuyv_img;

    rclcpp::Time image_capture_time = msg->header.stamp;

    // Measure latency between image capture and receipt by apriltags node
    rclcpp::Time current_time = this->now();
    auto latency_duration = current_time - image_capture_time;
    double latency_ms = latency_duration.seconds() * 1000.0;

    RCLCPP_DEBUG(this->get_logger(),
                "Image latency: %.2f ms (captured at %.6f, received at %.6f)",
                latency_ms, image_capture_time.seconds(),
                current_time.seconds());

    auto start = std::chrono::high_resolution_clock::now();
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");  // Use smart pointer
    cv::Mat bgr_img = cv_ptr->image;

    cv::cvtColor(bgr_img, yuyv_img, cv::COLOR_BGR2YUV_YUYV);

    auto detstart = std::chrono::high_resolution_clock::now();
    detector_->Detect(yuyv_img.data);
    auto detend = std::chrono::high_resolution_clock::now();
    auto det_processing_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(detend - detstart)
            .count();

    const zarray_t *detections = detector_->Detections();
    draw_detection_outlines(bgr_img, const_cast<zarray_t *>(detections));

    std::vector<double> networktables_pose_data = {};
    apriltags_cuda::msg::TagDetectionArray tag_detection_array_msg;
    apriltags_cuda::msg::TagDetectionArray tag_detection_camera_array_msg;
    tag_detection_array_msg.detections.clear();
    tag_detection_camera_array_msg.detections.clear();
    if (zarray_size(detections) > 0) {
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(const_cast<zarray_t *>(detections), i, &det);

        // Setup the detection info struct for use down below.
        info_.det = det;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info_, &pose);
        cv::Vec3d aprilTagInCameraFrame(pose.t->data[0], pose.t->data[1],
                                        pose.t->data[2]);

        RCLCPP_DEBUG(
            this->get_logger(),
            "Tag id: %d, x: %.6f, y: %.6f, z: %.6f, err: %.6f in camera frame",
            det->id, pose.t->data[0], pose.t->data[1], pose.t->data[2], err);

        cv::Mat aprilTagInCameraFrameAsMat = cv::Mat(aprilTagInCameraFrame);
        cv::Mat aprilTagInRobotFrame =
            extrinsic_rotation_ * aprilTagInCameraFrameAsMat +
            extrinsic_offset_;

        RCLCPP_DEBUG(this->get_logger(),
                     "Tag id: %d, x: %.6f, y: %.6f, z: %.6f in robot frame",
                     det->id, aprilTagInRobotFrame.at<double>(0),
                     aprilTagInRobotFrame.at<double>(1),
                     aprilTagInRobotFrame.at<double>(2));

        networktables_pose_data.push_back(image_capture_time.seconds());
        networktables_pose_data.push_back(det->id * 1.0);
        networktables_pose_data.push_back(aprilTagInRobotFrame.at<double>(0));
        networktables_pose_data.push_back(aprilTagInRobotFrame.at<double>(1));
        networktables_pose_data.push_back(aprilTagInRobotFrame.at<double>(2));

        // Publish both camera frame and robot frame
        apriltags_cuda::add_tag_detection(
            tag_detection_camera_array_msg, det->id, aprilTagInCameraFrame[0],
            aprilTagInCameraFrame[1], aprilTagInCameraFrame[2]);
        apriltags_cuda::add_tag_detection(tag_detection_array_msg, det->id,
                                          aprilTagInRobotFrame.at<double>(0),
                                          aprilTagInRobotFrame.at<double>(1),
                                          aprilTagInRobotFrame.at<double>(2));
      }
      detector_->ReinitializeDetections();
    }

    // Send the pose data to the network tables.
    tag_sender_->sendValue(networktables_pose_data);

    // Publish the marker array to the ROS topic
    tag_detection_pub_->publish(tag_detection_array_msg);
    tag_detection_camera_pub_->publish(tag_detection_camera_array_msg);

    // Publish the image message
    auto outgoing_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_img)
            .toImageMsg();
    outgoing_msg->header.stamp = image_capture_time;
    outgoing_msg->header.frame_id = "apriltag_detections";
    image_pub_queue_->enqueue(outgoing_msg);

    auto end = std::chrono::high_resolution_clock::now();
    auto processing_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();

    RCLCPP_DEBUG(this->get_logger(), "Total Time: %ld ms, Det Time: %ld ms",
                processing_time, det_processing_time);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

  std::shared_ptr<image_transport::ImageTransport> it_;

  std::shared_ptr<
      PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>
      image_pub_queue_;
  image_transport::Publisher image_publisher_;
  std::string publish_images_to_topic_;
  std::string publish_pose_to_topic_;
  std::string camera_serial_;
  std::string table_address_;
  std::string table_name_;

  std::shared_ptr<DoubleArraySender> tag_sender_;

  rclcpp::Publisher<apriltags_cuda::msg::TagDetectionArray>::SharedPtr
      tag_detection_pub_;
  rclcpp::Publisher<apriltags_cuda::msg::TagDetectionArray>::SharedPtr
      tag_detection_camera_pub_;

  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;
  apriltag_detection_info_t info_;
  frc971::apriltag::GpuDetector *detector_;
  const char *tag_family_name_ = "tag36h11";

  // Extrinsic parameters
  cv::Mat extrinsic_rotation_;  ///< 3x3 rotation matrix from extrinsics
  cv::Mat extrinsic_offset_;    ///< 3x1 offset vector from extrinsics
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApriltagsDetector>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}