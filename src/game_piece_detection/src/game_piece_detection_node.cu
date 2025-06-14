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

    auto start = std::chrono::high_resolution_clock::now();
    // TODO: Initalize ModelInference object with engine file.
    RCLCPP_INFO(this->get_logger(),
                "ModelInference Initialization not implemented yet!");
    
    auto end = std::chrono::high_resolution_clock::now();
    auto processing_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
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

  void get_extrinsic_params() {

    //TODO: Implement a method to parse the extrinsics parameters from the
    // config files.
    RCLCPP_INFO(this->get_logger(), "Extrinsics parameters not yet implemented!");

    return;

  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

    cv::Mat yuyv_img;

    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat bgr_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    //TODO: Run the inference and publish the output to a message.

    
    // Publish the message to the viewer
    // auto outgoing_msg =
    //     cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_img)
    //         .toImageMsg();
    // outgoing_msg->header.stamp = this->now();
    // outgoing_msg->header.frame_id = "apriltag_detections";
    // image_pub_queue_->enqueue(outgoing_msg);

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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamePieceDetector>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}