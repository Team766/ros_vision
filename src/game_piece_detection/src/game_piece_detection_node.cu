#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <nlohmann/json.hpp>

#include "game_piece_detection/ModelInference.h"
#include "game_piece_detection/yolo_detection.h"
#include "game_piece_detection/detection_utils.h"

#include "vision_utils/publisher_queue.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

class GamePieceDetector : public rclcpp::Node {
public:
GamePieceDetector()
      : Node("game_piece_detector") {

    // Declare parameters
    this->declare_parameter<std::string>("topic_name", "camera/image_raw");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    this->declare_parameter<std::string>("camera_serial", "N/A");
    camera_serial_ = this->get_parameter("camera_serial").as_string();

    this->declare_parameter<std::string>("publish_to_topic",
                                         "game_piece_detector/images");
    publish_to_topic_ = this->get_parameter("publish_to_topic").as_string();

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 1,
        std::bind(&GamePieceDetector::imageCallback, this,
                  std::placeholders::_1));

    // Load extrinsic parameters for camera-to-robot transformation
    get_extrinsic_params();

    // Load game piece detection configuration (throws on error)
    get_game_piece_detection_config();

    // Verify engine file exists
    if (!std::filesystem::exists(engine_file_path_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Engine file not found: %s", engine_file_path_.c_str());
      throw std::runtime_error("Engine file not found: " + engine_file_path_);
    }

    // Initialize TensorRT model
    RCLCPP_INFO(this->get_logger(),
                "Loading TensorRT engine from: %s", engine_file_path_.c_str());
    try {
      model_ = std::make_unique<ModelInference>(engine_file_path_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Exception while initializing ModelInference from '%s': %s",
                   engine_file_path_.c_str(), e.what());
      throw;
    }

    // Get model dimensions
    model_input_width_ = model_->getInputWidth();
    model_input_height_ = model_->getInputHeight();
    num_classes_ = model_->getNumClasses();
    num_predictions_ = model_->getNumPredictions();

    // Verify input channels match config
    int engine_channels = model_->getInputChannels();
    if (engine_channels != input_channels_) {
      RCLCPP_WARN(this->get_logger(),
                  "Engine input channels (%d) differs from config (%d), using engine value",
                  engine_channels, input_channels_);
      input_channels_ = engine_channels;
    }

    // Allocate inference buffers
    size_t input_size = input_channels_ * model_input_width_ * model_input_height_;
    size_t output_size = model_->getOutputSize() / sizeof(float);
    input_buffer_.resize(input_size);
    output_buffer_.resize(output_size);

    RCLCPP_INFO(this->get_logger(),
                "Model initialized: input=%dx%dx%d, classes=%d, predictions=%d",
                model_input_width_, model_input_height_, input_channels_,
                num_classes_, num_predictions_);
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

  /**
   * @brief Load game piece detection configuration from system config file.
   *
   * Reads the engine file path, input channels, and class names from the
   * game_piece_detection section. Throws std::runtime_error on failure.
   */
  void get_game_piece_detection_config() {
    // Locate the system config file
    fs::path config_path =
        ament_index_cpp::get_package_share_directory("vision_config_data");
    fs::path config_file = config_path / "data" / "system_config.json";

    if (!std::filesystem::exists(config_file)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find system config file at path: %s",
                   config_file.c_str());
      throw std::runtime_error("System config file not found: " + config_file.string());
    }

    // Load the parameters from the file
    std::ifstream f(config_file);
    json data = json::parse(f);

    if (!data.contains("game_piece_detection")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to find key \"game_piece_detection\" in system config file");
      throw std::runtime_error("Missing 'game_piece_detection' section in config");
    }

    const auto& gpd = data["game_piece_detection"];

    // Read engine file path (required)
    if (!gpd.contains("engine_file")) {
    if (!f.is_open()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open system config file at path: %s",
                   config_file.c_str());
      throw std::runtime_error("Failed to open system config file: " + config_file.string());
    }

    json data;
    try {
      data = json::parse(f);

      if (!data.contains("game_piece_detection")) {
        RCLCPP_ERROR(this->get_logger(),
                     "Unable to find key \"game_piece_detection\" in system config file");
        throw std::runtime_error("Missing 'game_piece_detection' section in config");
      }

      const auto& gpd = data["game_piece_detection"];

      // Read engine file path (required)
      if (!gpd.contains("engine_file")) {
        RCLCPP_ERROR(this->get_logger(),
                     "Missing \"engine_file\" in game_piece_detection config");
        throw std::runtime_error("Missing 'engine_file' in game_piece_detection config");
      }
      engine_file_path_ = gpd["engine_file"].get<std::string>();

      // Read input channels (optional, defaults to 3)
      if (gpd.contains("input_channels")) {
        input_channels_ = gpd["input_channels"].get<int>();
      }

      // Read class names (optional but recommended)
      if (gpd.contains("class_names")) {
        for (const auto& name : gpd["class_names"]) {
          class_names_.push_back(name.get<std::string>());
        }
      }

      RCLCPP_INFO(this->get_logger(),
                  "Loaded game piece detection config: engine=%s, channels=%d, classes=%zu",
                  engine_file_path_.c_str(), input_channels_, class_names_.size());
    } catch (const nlohmann::json::parse_error& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to parse JSON from system config file '%s': %s",
                   config_file.c_str(), e.what());
      throw std::runtime_error(std::string("Failed to parse system config JSON: ") + e.what());
    } catch (const nlohmann::json::type_error& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Type error while reading game_piece_detection config from '%s': %s",
                   config_file.c_str(), e.what());
      throw std::runtime_error(std::string("Invalid type in system config JSON: ") + e.what());
    } catch (const nlohmann::json::out_of_range& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Out-of-range error while reading game_piece_detection config from '%s': %s",
                   config_file.c_str(), e.what());
      throw std::runtime_error(std::string("Out-of-range error in system config JSON: ") + e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unexpected error while reading system config file '%s': %s",
                   config_file.c_str(), e.what());
      throw std::runtime_error(std::string("Unexpected error reading system config: ") + e.what());
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Model must be initialized - this is a fatal condition
    if (!model_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Model not initialized - cannot process images");
      throw std::runtime_error("Model not initialized in imageCallback");
    }

    // Input is always BGR from cv_bridge, regardless of whether camera is RGB or mono.
    // The preprocess_image() function handles conversion to what the model expects.
    cv::Mat bgr_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    int orig_width = bgr_img.cols;
    int orig_height = bgr_img.rows;

    // Step 1: Preprocess image
    // - Resizes to model input dimensions (e.g., 640x640)
    // - Normalizes pixel values to [0, 1]
    // - Converts BGR to RGB (if input_channels_==3) or grayscale (if input_channels_==1)
    // - Converts from HWC to CHW format for GPU inference
    preprocess_image(bgr_img, input_buffer_.data(),
                     model_input_width_, model_input_height_, input_channels_);

    // Step 2: Run inference
    if (!model_->infer(input_buffer_.data(), output_buffer_.data())) {
      RCLCPP_ERROR(this->get_logger(), "Inference failed");
      return;  // Skip this frame but don't crash
    }

    // Step 3: Parse detections from model output
    auto detections = parse_yolov11_output(
        output_buffer_.data(),
        num_predictions_,
        num_classes_,
        class_names_,
        DEFAULT_CONF_THRESHOLD,
        DEFAULT_IOU_THRESHOLD);

    // Step 4: Scale detections from model input size back to original image size
    auto scaled_detections = scale_detections(
        detections,
        model_input_width_, model_input_height_,
        orig_width, orig_height);

    // Step 5: Draw detections on image (always on BGR image for visualization)
    draw_detections(bgr_img, scaled_detections);

    // Step 6: Publish annotated image (following apriltags_cuda pattern)
    auto output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_img).toImageMsg();
    output_msg->header.stamp = msg->header.stamp;  // Preserve original timestamp
    output_msg->header.frame_id = "game_piece_detections";
    image_pub_queue_->enqueue(output_msg);

    // Log detection count (throttled to avoid spam)
    if (!scaled_detections.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Detected %zu game pieces", scaled_detections.size());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

  std::shared_ptr<image_transport::ImageTransport> it_;

  std::shared_ptr<
      PublisherQueue<sensor_msgs::msg::Image, image_transport::Publisher>>
      image_pub_queue_;
  image_transport::Publisher publisher_;
  std::string publish_to_topic_;
  std::string camera_serial_;

  // Model inference
  std::unique_ptr<ModelInference> model_;

  // Model configuration (from system_config.json)
  std::string engine_file_path_;
  int input_channels_ = 3;
  std::vector<std::string> class_names_;

  // Inference buffers (allocated once, reused per frame)
  std::vector<float> input_buffer_;
  std::vector<float> output_buffer_;

  // Model dimensions (populated after model initialization)
  int model_input_width_ = 0;
  int model_input_height_ = 0;
  int num_classes_ = 0;
  int num_predictions_ = 0;

  // Extrinsic calibration
  cv::Mat extrinsic_rotation_;
  cv::Mat extrinsic_offset_;

  // NetworkTables config
  std::string table_address_;
  std::string table_name_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamePieceDetector>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}