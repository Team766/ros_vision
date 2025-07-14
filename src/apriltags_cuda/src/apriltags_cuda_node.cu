#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"

#include "vision_utils/publisher_queue.hpp"
#include "vision_utils/camera_utils.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/zarray.h"
}

#define TAGSIZE 0.1651 // tag size, in meters

namespace fs = std::filesystem;
using json = nlohmann::json;

class ApriltagsDetector : public rclcpp::Node {
public:
  ApriltagsDetector()
      : Node("apriltags_detector"), tag_family_(nullptr),
        tag_detector_(nullptr) {

    // Decare parameters
    this->declare_parameter<std::string>("topic_name", "camera/image_raw");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    this->declare_parameter<std::string>("camera_serial", "N/A");
    camera_serial_ = this->get_parameter("camera_serial").as_string();

    this->declare_parameter<std::string>("publish_to_topic",
                                         "apriltags/images");
    publish_to_topic_ = this->get_parameter("publish_to_topic").as_string();

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 1,
        std::bind(&ApriltagsDetector::imageCallback, this,
                  std::placeholders::_1));

    // Apriltag detector setup
    setup_tag_family(&tag_family_, tag_family_name_);
    tag_detector_ = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector_, tag_family_);

    tag_detector_->quad_decimate = 2.0;
    tag_detector_->quad_sigma = 0.0;
    tag_detector_->nthreads = 1;
    tag_detector_->debug = false;
    tag_detector_->refine_edges = true;
    tag_detector_->wp = workerpool_create(4);

    // TODO: read these from a file or a topic.
    frc971::apriltag::CameraMatrix cam;
    frc971::apriltag::DistCoeffs dist;

    get_camera_calibration_data(&cam, &dist);
    get_extrinsic_params();

    // Get camera width and height from vision_utils
    auto camera_data = vision_utils::get_camera_data(camera_serial_);
    int frame_width = camera_data.width;
    int frame_height = camera_data.height;

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

  void init() {
    // The object needs to be constructed before using shared_from_this, thus
    // it is broken off into another method.
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    publisher_ = it_->advertise(publish_to_topic_, 10);
    image_pub_queue_ = std::make_shared<
        PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>(
        publisher_, 2);

    RCLCPP_INFO(this->get_logger(), "Publishing on topic: %s",
                publish_to_topic_.c_str());
  }

  ~ApriltagsDetector() {
    apriltag_detector_destroy(tag_detector_);
    teardown_tag_family(&tag_family_, tag_family_name_);
    delete detector_;
    image_pub_queue_->stop();
  }

private:

  void get_extrinsic_params() {

    //TODO: Implement a method to parse the extrinsics parameters from the
    // config files.
    RCLCPP_INFO(this->get_logger(), "Extrinsics parameters not yet implemented!");

    return;

  }

  void get_camera_calibration_data(frc971::apriltag::CameraMatrix* cam,
                                   frc971::apriltag::DistCoeffs* dist) {

    // Locate the calibration file for the camera serial id that we are processing.
    fs::path config_path = ament_index_cpp::get_package_share_directory("vision_config_data");
    std::ostringstream oss;
    oss << "calibrationmatrix_" << camera_serial_ << ".json";
    fs::path cal_file = config_path / "data" / "calibration" / oss.str();

    if (!std::filesystem::exists(cal_file)) {
      RCLCPP_ERROR(this->get_logger(),
        "Unable to find calibration parameters for camera %s at path: %s", camera_serial_.c_str(), cal_file.c_str());

    }

    //Load the parameters from the file.
    std::ifstream f(cal_file);
    json data = json::parse(f);

    if (!data.contains("matrix")) {
      RCLCPP_ERROR(this->get_logger(), "Unable to find key \"matrix\" in calibration file");
    }
    if (!data.contains("disto")) {
      RCLCPP_ERROR(this->get_logger(), "Unable to find key \"disto\" in calibration file");
    }

    RCLCPP_INFO(this->get_logger(), "Loading camera calibration params from: %s",
                cal_file.c_str());

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

    RCLCPP_INFO(this->get_logger(), "Loaded camera matrix parameters: fx: %f, fy: %f, cx: %f, cy: %f",
      cam->fx, cam->fy, cam->cx, cam->cy);
    RCLCPP_INFO(this->get_logger(), "Loaded distortion coeffs k1: %f, k2: %f, p1: %f, p2: %f, k3: %f",
      dist->k1, dist->k2, dist->p1, dist->p2, dist->k3);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

    cv::Mat yuyv_img;

    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat bgr_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    cv::cvtColor(bgr_img, yuyv_img, cv::COLOR_BGR2YUV_YUYV);

    auto detstart = std::chrono::high_resolution_clock::now();
    detector_->Detect(yuyv_img.data);
    auto detend = std::chrono::high_resolution_clock::now();
    auto det_processing_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(detend - detstart)
            .count();

    const zarray_t *detections = detector_->Detections();
    draw_detection_outlines(bgr_img, const_cast<zarray_t *>(detections));

    if (zarray_size(detections) > 0) {
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(const_cast<zarray_t *>(detections), i, &det);

        // Setup the detection info struct for use down below.
        info_.det = det;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info_, &pose);

        RCLCPP_DEBUG(this->get_logger(),
                    "Tag id: %d, x: %.6f, y: %.6f, z: %.6f, err: %.6f", det->id,
                    pose.t->data[0], pose.t->data[1], pose.t->data[2], err);
      }
      detector_->ReinitializeDetections();
    }

    // Publish the message to the viewer
    auto outgoing_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_img)
            .toImageMsg();
    outgoing_msg->header.stamp = this->now();
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
  image_transport::Publisher publisher_;
  std::string publish_to_topic_;
  std::string camera_serial_;

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