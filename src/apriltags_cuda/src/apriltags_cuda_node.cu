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

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 1,
        std::bind(&ApriltagsDetector::imageCallback, this,
                  std::placeholders::_1));
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

  void get_extrinsic_params() {
    // TODO: Implement a method to parse the extrinsics parameters from the
    //  config files.
    RCLCPP_INFO(this->get_logger(),
                "Extrinsics parameters not yet implemented!");

    return;
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
    tag_detection_array_msg.detections.clear();
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

        RCLCPP_DEBUG(this->get_logger(),
                     "Tag id: %d, x: %.6f, y: %.6f, z: %.6f, err: %.6f",
                     det->id, pose.t->data[0], pose.t->data[1], pose.t->data[2],
                     err);
        networktables_pose_data.push_back(image_capture_time.seconds());
        networktables_pose_data.push_back(det->id * 1.0);
        networktables_pose_data.push_back(aprilTagInCameraFrame[0]);
        networktables_pose_data.push_back(aprilTagInCameraFrame[1]);
        networktables_pose_data.push_back(aprilTagInCameraFrame[2]);

        apriltags_cuda::add_tag_detection(
            tag_detection_array_msg, det->id, aprilTagInCameraFrame[0],
            aprilTagInCameraFrame[1], aprilTagInCameraFrame[2]);
      }
      detector_->ReinitializeDetections();
    }

    // Send the pose data to the network tables.
    tag_sender_->sendValue(networktables_pose_data);

    // Publish the marker array to the ROS topic
    tag_detection_pub_->publish(tag_detection_array_msg);

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

  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;
  apriltag_detection_info_t info_;
  frc971::apriltag::GpuDetector *detector_;
  const char *tag_family_name_ = "tag36h11";
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApriltagsDetector>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}