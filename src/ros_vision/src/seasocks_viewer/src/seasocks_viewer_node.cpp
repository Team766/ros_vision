#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <seasocks/PrintfLogger.h>
#include <seasocks/Server.h>
#include <seasocks/WebSocket.h>
#include <sensor_msgs/msg/image.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace seasocks;

class ImageWebSocketHandler : public WebSocket::Handler {
public:
  void onConnect(WebSocket *socket) override {
    std::lock_guard<std::mutex> lock(mutex_);
    clients_.insert(socket);
  }

  void onDisconnect(WebSocket *socket) override {
    std::lock_guard<std::mutex> lock(mutex_);
    clients_.erase(socket);
  }

  void sendImage(Server &server, const std::vector<uint8_t> &imageData) {

    server.execute([this, imageData]() {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto *client : clients_) {
        // client->send(reinterpret_cast<const char*>(imageData.data()),
        // imageData.size());
        client->send(imageData.data(), imageData.size());
      }
    });
  }

private:
  std::set<WebSocket *> clients_;
  std::mutex mutex_;
};

class SeasocksViewerNode : public rclcpp::Node {
public:
  SeasocksViewerNode() : Node("seasocks_viewer_node") {

    // Declare parameters
    this->declare_parameter<std::string>("topic_name", "camera/image_raw");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    this->declare_parameter<int>("port", 9090);
    int port = this->get_parameter("port").as_int();

    // Seasocks setup
    auto logger = std::make_shared<PrintfLogger>();
    server_ = std::make_shared<Server>(logger);
    image_handler_ = std::make_shared<ImageWebSocketHandler>();
    server_->addWebSocketHandler("/image", image_handler_);

    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("seasocks_viewer");
    std::string web_root = package_share_directory + "/web";

    server_thread_ = std::thread(
        [this, web_root, port] { server_->serve(web_root.c_str(), port); });

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 10,
        std::bind(&SeasocksViewerNode::imageCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Seasocks viewer running at ws://localhost:'%d'/image", port);
    RCLCPP_INFO(get_logger(), "Reading images from topic: '%s'",
                topic_name.c_str());
  }

  ~SeasocksViewerNode() override {
    server_->execute(
        [this]() { server_->terminate(); }); // call terminate on correct thread
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    std::vector<uint8_t> buffer;
    cv::imencode(".jpg", cv_ptr->image, buffer);

    image_handler_->sendImage(*server_, buffer);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::shared_ptr<Server> server_;
  std::shared_ptr<ImageWebSocketHandler> image_handler_;
  std::thread server_thread_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SeasocksViewerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
