#include "apriltags_cuda/apriltags_cuda_detector.hpp"

/**
 * @brief Construct the ApriltagsDetector node and initialize topics and
 * AprilTag detector.
 */
ApriltagsDetector::ApriltagsDetector()
    : Node("apriltags_detector"),
      tag_sender_(nullptr),
      proto_tag_sender_(nullptr),
      tag_family_(nullptr),
      tag_detector_(nullptr),
      detector_(nullptr) {
  setup_topics();
  get_network_tables_params();
  setup_apriltags();
  setup_measurement_params();
  applyCpuPinningAndScheduling();
}

/**
 * @brief Protected constructor for testing that bypasses full initialization
 */
ApriltagsDetector::ApriltagsDetector(bool bypass_init)
    : Node("apriltags_detector"),
      tag_sender_(nullptr),
      proto_tag_sender_(nullptr),
      tag_family_(nullptr),
      tag_detector_(nullptr),
      detector_(nullptr) {
  if (!bypass_init) {
    setup_topics();
    get_network_tables_params();
    setup_apriltags();
    setup_measurement_params();
  }
  // For testing, just initialize extrinsics to identity
  extrinsic_rotation_ = cv::Mat::eye(3, 3, CV_64F);
  extrinsic_offset_ = cv::Mat::zeros(3, 1, CV_64F);
}

/**
 * @brief Initialize publishers and image transport after construction.
 *
 * This must be called after the node is fully constructed.
 */
void ApriltagsDetector::init() {
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
ApriltagsDetector::~ApriltagsDetector() {
  if (tag_detector_) {
    apriltag_detector_destroy(tag_detector_);
  }
  if (tag_family_) {
    teardown_tag_family(&tag_family_, tag_family_name_);
  }
  if (detector_) {
    delete detector_;
  }
  if (image_pub_queue_) {
    image_pub_queue_->stop();
  }
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }
}

/**
 * @brief Declare and initialize all ROS 2 topics and parameters.
 */
void ApriltagsDetector::setup_topics() {
  this->declare_parameter<std::string>("topic_name", "camera/image_raw");
  std::string topic_name = this->get_parameter("topic_name").as_string();

  this->declare_parameter<std::string>("camera_serial", "N/A");
  camera_serial_ = this->get_parameter("camera_serial").as_string();

  this->declare_parameter<std::string>("publish_images_to_topic",
                                       "apriltags/images");
  publish_images_to_topic_ =
      this->get_parameter("publish_images_to_topic").as_string();

  this->declare_parameter<std::string>("publish_pose_to_topic", "camera/pose");
  publish_pose_to_topic_ =
      this->get_parameter("publish_pose_to_topic").as_string();

  this->declare_parameter<int>("pin_to_core", -1);
  pin_to_core_ = this->get_parameter("pin_to_core").as_int();

  this->declare_parameter<int>("priority", 80);
  priority_ = this->get_parameter("priority").as_int();

  // Create subscription with optimized QoS for low latency
  auto qos = rclcpp::QoS(1)              // Reduce queue depth for lower latency
                 .best_effort()          // Use best effort for lower latency
                 .durability_volatile()  // No need to store messages
                 .deadline(std::chrono::milliseconds(
                     50));  // Expect messages within 50ms

  subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, qos,
      std::bind(&ApriltagsDetector::imageCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s with optimized QoS",
              topic_name.c_str());
}

/**
 * @brief Set up the AprilTag detector, camera calibration, and network
 * tables.
 *
 * This method initializes the tag family, detector, loads camera calibration,
 * extrinsics, and network tables configuration, and creates the GPU detector.
 */
void ApriltagsDetector::setup_apriltags() {
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

  inst_ = nt::NetworkTableInstance::GetDefault();
  inst_.SetServer(table_address_.c_str());
  inst_.StartClient4(table_address_.c_str());
  nt_table_ = inst_.GetTable(table_name_);

  tag_sender_ = std::make_shared<DoubleArraySender>(camera_serial_, nt_table_);
  proto_tag_sender_ =
      std::make_shared<AprilTagProtobufSender>(camera_serial_, nt_table_);

  // Load camera configuration to get frame dimensions
  auto camera_config =
      vision_utils::ConfigLoader::getCameraConfig(camera_serial_);

  if (!camera_config.has_value()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not load camera config for serial %s - configuration "
                 "is required",
                 camera_serial_.c_str());
    throw std::runtime_error(
        "Failed to load camera configuration for serial: " + camera_serial_);
  }

  int frame_width = camera_config->width;
  int frame_height = camera_config->height;
  RCLCPP_INFO(this->get_logger(),
              "Using camera config dimensions: %dx%d for serial %s",
              frame_width, frame_height, camera_serial_.c_str());

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

  RCLCPP_INFO(this->get_logger(), "GPU Apriltag Detector created, took %ld ms",
              processing_time);
}

/**
 * @brief Load extrinsic parameters (rotation and offset) for this camera from
 * the system config file.
 *
 * Reads the rotation (3x3) and offset (3x1) from the extrinsics record for
 * the camera position, and stores them in OpenCV cv::Mat (rotation) and
 * cv::Vec3d (offset).
 */
void ApriltagsDetector::get_extrinsic_params() {
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
    auto camera_config = data["camera_mounted_positions"][camera_serial_];
    if (camera_config.is_string()) {
      // Legacy format: camera config is just a location string
      camera_position = camera_config.get<std::string>();
    } else if (camera_config.is_object() &&
               camera_config.contains("location")) {
      // New format: camera config is an object with location field
      camera_position = camera_config["location"].get<std::string>();
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid camera config format for serial %s: expected "
                   "string or object with 'location' field",
                   camera_serial_.c_str());
      return;
    }
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

  RCLCPP_INFO(this->get_logger(),
              "Loaded extrinsics for %s: "
              "rotation=[[%f,%f,%f],[%f,%f,%f],[%f,%f,%f]], offset=[%f,%f,%f]",
              camera_position.c_str(), rotation.at<double>(0, 0),
              rotation.at<double>(0, 1), rotation.at<double>(0, 2),
              rotation.at<double>(1, 0), rotation.at<double>(1, 1),
              rotation.at<double>(1, 2), rotation.at<double>(2, 0),
              rotation.at<double>(2, 1), rotation.at<double>(2, 2), offset[0],
              offset[1], offset[2]);
}

void ApriltagsDetector::get_network_tables_params() {
  auto network_config = vision_utils::ConfigLoader::getNetworkTablesConfig();

  table_address_ = network_config.table_address;
  table_name_ = network_config.table_name;

  RCLCPP_INFO(
      this->get_logger(),
      "Loaded network tables parameters table_address: %s, table_name: %s",
      table_address_.c_str(), table_name_.c_str());
}

void ApriltagsDetector::get_camera_calibration_data(
    frc971::apriltag::CameraMatrix *cam, frc971::apriltag::DistCoeffs *dist) {
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

  RCLCPP_INFO(this->get_logger(),
              "Loaded camera matrix parameters: fx: %f, fy: %f, cx: %f, cy: %f",
              cam->fx, cam->fy, cam->cx, cam->cy);
  RCLCPP_INFO(this->get_logger(),
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
void ApriltagsDetector::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
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
  auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
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
  com::team766::vision::ApriltagListProto tag_detection_proto_list;
  tag_detection_array_msg.detections.clear();
  tag_detection_camera_array_msg.detections.clear();

  if (zarray_size(detections) > 0) {
    // First pass: collect all detections and calculate distances
    std::vector<DetectionData> detection_data;
    detection_data.reserve(zarray_size(detections));

    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(const_cast<zarray_t *>(detections), i, &det);

      // Setup the detection info struct for pose estimation
      info_.det = det;

      apriltag_pose_t pose;
      double err = estimate_tag_pose(&info_, &pose);
      cv::Vec3d aprilTagInCameraFrame(pose.t->data[0], pose.t->data[1],
                                      pose.t->data[2]);

      cv::Mat aprilTagInRobotFrame =
          transformCameraToRobot(aprilTagInCameraFrame);

      // Calculate Euclidean distance from camera origin (use camera frame for
      // consistency)
      double distance =
          std::sqrt(aprilTagInCameraFrame[0] * aprilTagInCameraFrame[0] +
                    aprilTagInCameraFrame[1] * aprilTagInCameraFrame[1] +
                    aprilTagInCameraFrame[2] * aprilTagInCameraFrame[2]);

      // Store detection data
      detection_data.push_back({det, aprilTagInCameraFrame,
                                aprilTagInRobotFrame.clone(), distance, err});

      RCLCPP_DEBUG(this->get_logger(),
                   "Tag id: %d, distance: %.6f, x: %.6f, y: %.6f, z: %.6f, "
                   "err: %.6f in camera frame",
                   det->id, distance, pose.t->data[0], pose.t->data[1],
                   pose.t->data[2], err);
    }

    // Sort detections by distance (closest first)
    std::sort(detection_data.begin(), detection_data.end(),
              [](const DetectionData &a, const DetectionData &b) {
                return a.distance < b.distance;
              });

    // Second pass: process sorted detections
    for (const auto &data : detection_data) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Processing sorted tag id: %d, distance: %.6f, x: %.6f, y: %.6f, z: "
          "%.6f in robot frame",
          data.det->id, data.distance, data.robot_position.at<double>(0),
          data.robot_position.at<double>(1), data.robot_position.at<double>(2));

      // Add to network tables data (sorted by distance)
      networktables_pose_data.push_back(image_capture_time.seconds());
      networktables_pose_data.push_back(data.det->id * 1.0);
      networktables_pose_data.push_back(data.robot_position.at<double>(0));
      networktables_pose_data.push_back(data.robot_position.at<double>(1));
      networktables_pose_data.push_back(data.robot_position.at<double>(2));

      // Add to protobuf list
      auto* tag_proto = tag_detection_proto_list.add_tags();
      tag_proto->set_collect_time(image_capture_time.seconds());
      tag_proto->set_tag_id(data.det->id);
      tag_proto->set_x(data.robot_position.at<double>(0));
      tag_proto->set_y(data.robot_position.at<double>(1));
      tag_proto->set_z(data.robot_position.at<double>(2));

      // Publish both camera frame and robot frame (sorted by distance)
      apriltags_cuda::add_tag_detection(
          tag_detection_camera_array_msg, data.det->id, data.camera_position[0],
          data.camera_position[1], data.camera_position[2]);
      apriltags_cuda::add_tag_detection(tag_detection_array_msg, data.det->id,
                                        data.robot_position.at<double>(0),
                                        data.robot_position.at<double>(1),
                                        data.robot_position.at<double>(2));
    }
    detector_->ReinitializeDetections();
  }

  auto nt_start = std::chrono::high_resolution_clock::now();
  tag_sender_->sendValue(networktables_pose_data);
  proto_tag_sender_->sendProtobuf(tag_detection_proto_list);
  inst_.Flush();
  auto nt_end = std::chrono::high_resolution_clock::now();

  auto pub_pose_start = std::chrono::high_resolution_clock::now();
  tag_detection_pub_->publish(tag_detection_array_msg);
  auto pub_pose_end = std::chrono::high_resolution_clock::now();

  auto pub_camera_pose_start = std::chrono::high_resolution_clock::now();
  tag_detection_camera_pub_->publish(tag_detection_camera_array_msg);
  auto pub_camera_pose_end = std::chrono::high_resolution_clock::now();

  auto pub_image_start = std::chrono::high_resolution_clock::now();
  auto outgoing_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_img).toImageMsg();
  outgoing_msg->header.stamp = image_capture_time;
  outgoing_msg->header.frame_id = "apriltag_detections";
  image_pub_queue_->enqueue(outgoing_msg);
  auto pub_image_end = std::chrono::high_resolution_clock::now();

  auto end = std::chrono::high_resolution_clock::now();
  auto processing_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  if (measurement_mode_ && csv_file_.is_open()) {
    csv_file_ << latency_duration.nanoseconds() / 1000 << ","
              << std::chrono::duration_cast<std::chrono::microseconds>(detend -
                                                                       detstart)
                     .count()
              << ","
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     pub_pose_end - pub_pose_start)
                     .count()
              << ","
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     pub_camera_pose_end - pub_camera_pose_start)
                     .count()
              << ","
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     pub_image_end - pub_image_start)
                     .count()
              << ","
              << std::chrono::duration_cast<std::chrono::microseconds>(nt_end -
                                                                       nt_start)
                     .count()
              << ","
              << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                       start)
                     .count()
              << "\n";
    csv_file_.flush();
  }

  RCLCPP_DEBUG(this->get_logger(), "Total Time: %ld ms, Det Time: %ld ms",
               processing_time, det_processing_time);
}

void ApriltagsDetector::setup_measurement_params() {
  this->declare_parameter<bool>("measurement_mode", false);
  // Generate default filename: apriltags_timing_YYYYMMDD_hhmmss.csv
  this->declare_parameter<std::string>("timing_csv_path", "");
  measurement_mode_ = this->get_parameter("measurement_mode").as_bool();
  csv_path_ = this->get_parameter("timing_csv_path").as_string();
  if (csv_path_.empty()) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&now_c, &tm_now);
    std::ostringstream oss;
    oss << "apriltags_timing_" << std::put_time(&tm_now, "%Y%m%d_%H%M%S")
        << ".csv";
    csv_path_ = oss.str();
  }
  RCLCPP_INFO(this->get_logger(), "Measurement mode: %s",
              measurement_mode_ ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(this->get_logger(), "Timing CSV path: '%s'", csv_path_.c_str());
  if (measurement_mode_ && !csv_path_.empty()) {
    csv_file_.open(csv_path_, std::ios::out);
    if (csv_file_.is_open()) {
      RCLCPP_INFO(this->get_logger(),
                  "Timing CSV file successfully opened: '%s'",
                  csv_path_.c_str());
      csv_file_
          << "latency_us,det_time_us,publish_pose_us,publish_camera_pose_us,"
             "publish_image_us,networktables_us,processing_time_us\n";
      csv_file_.flush();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open timing CSV file: '%s'",
                   csv_path_.c_str());
    }
  }
}

cv::Mat ApriltagsDetector::transformCameraToRobot(
    const cv::Vec3d &camera_point) const {
  cv::Mat camera_point_mat = cv::Mat(camera_point);
  return extrinsic_rotation_ * camera_point_mat + extrinsic_offset_;
}

void ApriltagsDetector::applyCpuPinningAndScheduling() {
  auto logger = this->get_logger();
  vision_utils::ProcessScheduler::applyCpuPinningAndScheduling(
      pin_to_core_, priority_, &logger);
}
