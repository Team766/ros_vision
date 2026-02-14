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

#include "game_piece_detection/ModelInference.h"
#include "game_piece_detection/yolo_detection.h"

#include "vision_utils/publisher_queue.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

constexpr float CONF_THRESHOLD = 0.25f;
constexpr float IOU_THRESHOLD = 0.45f;

class GamePieceDetector : public rclcpp::Node {
public:
GamePieceDetector()
      : Node("game_piece_detector") {

    // Decare parameters
    this->declare_parameter<std::string>("topic_name", "camera/image_raw");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    this->declare_parameter<std::string>("camera_serial", "N/A");
    camera_serial_ = this->get_parameter("camera_serial").as_string();

    this->declare_parameter<std::string>("engine_file", "N/A");
    engine_file_ = this->get_parameter("engine_file").as_string();

    this->declare_parameter<std::string>("publish_to_topic",
                                         "game_piece_detector/images");
    publish_to_topic_ = this->get_parameter("publish_to_topic").as_string();

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 1,
        std::bind(&GamePieceDetector::imageCallback, this,
                  std::placeholders::_1));

    // Game piece detector setup
    get_extrinsic_params();

    // TODO: Initialize ModelInference object with engine file.
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
    json config_data = json::parse(f);


    std::string engine_path = config_data.value("engine_file", "");

    if (engine_path.empty()) {
      RCLCPP_ERROR(this->get_logger(),
          "Error: No engine file specified in config or command line");
    }

    if (!fs::exists(engine_path)) {
      RCLCPP_ERROR(this->get_logger(),
          "Error: Engine path not found: %s",
          engine_path.c_str());
    }

    auto start_load = std::chrono::high_resolution_clock::now();
    ModelInference model(engine_path);
    auto end_load = std::chrono::high_resolution_clock::now();
    auto load_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_load - start_load).count();
    std::cout << "Loaded ModelInference Engine in " << load_time << " ms" << std::endl;
    
    input_channels_ = model.getInputChannels();
    int input_height = model.getInputHeight();
    int input_width = model.getInputWidth();
    num_classes_ = model.getNumClasses();
    num_predictions_ = model.getNumPredictions();

    input_channels_ = config_data.value("input_channels", 3);

    input_size_ = input_channels_ * input_width * input_height;
    output_size_ = model.getOutputSize() / sizeof(float);

    if (input_channels != config_input_channels) {
      std::cerr << "Warning: Engine input channels (" << input_channels
              << ") differs from config (" << config_input_channels << ")" << std::endl;
    }

    if (config_data.contains("class_names")) {
      for (const auto& name : config_data["class_names"]) {
        class_names_.push_back(name.get<std::string>());
      }
    }


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

  ~GamePieceDetector() {
    image_pub_queue_->stop();
  }

private:

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

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat bgr_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    std::vector<float> input_buffer(input_size);
    std::vector<float> output_buffer(output_size);

    int orig_width = bgr_img.cols;
    int orig_height = bgr_img.rows;

    preprocess_image(bgr_img, input_buffer.data(), input_width, input_height, input_channels);


    auto detections = parse_yolov11_output(
      output_buffer.data(), num_predictions, num_classes, class_names,
      CONF_THRESHOLD, IOU_THRESHOLD);

    auto scaled_detections = scale_detections(detections, input_width, input_height,
                                               orig_width, orig_height);

    std::cout << "  Size: " << orig_width << "x" << orig_height
            << ", Inference: " << std::fixed << std::setprecision(2) << infer_time << " ms"
            << ", Detections: " << scaled_detections.size() << std::endl;
    // TODO: Publish the inference and detections out
    
    (void)bgr_img;  // Suppress unused warning until inference is implemented
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

  std::shared_ptr<image_transport::ImageTransport> it_;

  std::shared_ptr<
      PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>
      image_pub_queue_;
  image_transport::Publisher publisher_;
  std::string publish_to_topic_;
  std::string camera_serial_;
  std::string engine_file_;

  // Extrinsic calibration
  cv::Mat extrinsic_rotation_;
  cv::Mat extrinsic_offset_;

  // NetworkTables config
  std::string table_address_;
  std::string table_name_;

  size_t input_size_;
  size_t output_size_;

  int input_channels_;

  std:vector<std::string> class_names_;

  int num_predictions_;

  int num_classes_;
  
};

/**
 * Preprocess an image for YOLOv11 inference.
 * - Resize to model input size
 * - Convert BGR to RGB (if 3 channels) or grayscale
 * - Normalize to [0, 1]
 * - Convert to NCHW format (channels first)
 *
 * @param img Input BGR image (any size)
 * @param output Pre-allocated buffer for output
 * @param input_width Model input width
 * @param input_height Model input height
 * @param input_channels Number of input channels (1 or 3)
 */
void preprocess_image(const cv::Mat& img, float* output,
                      int input_width, int input_height, int input_channels) {
  // Resize to model input size
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(input_width, input_height));

  cv::Mat processed;
  if (input_channels == 3) {
    // Convert BGR to RGB
    cv::cvtColor(resized, processed, cv::COLOR_BGR2RGB);
  } else if (input_channels == 1) {
    // Convert to grayscale
    cv::cvtColor(resized, processed, cv::COLOR_BGR2GRAY);
  } else {
    processed = resized;
  }

  // Convert to float and normalize to [0, 1]
  cv::Mat float_img;
  int cv_type = (input_channels == 1) ? CV_32FC1 : CV_32FC3;
  processed.convertTo(float_img, cv_type, 1.0 / 255.0);

  // Convert from HWC to CHW (NCHW with batch=1)
  size_t channel_size = input_width * input_height;
  if (input_channels == 1) {
    memcpy(output, float_img.data, channel_size * sizeof(float));
  } else {
    std::vector<cv::Mat> channels(input_channels);
    cv::split(float_img, channels);
    for (int c = 0; c < input_channels; ++c) {
      memcpy(output + c * channel_size, channels[c].data, channel_size * sizeof(float));
    }
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamePieceDetector>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}