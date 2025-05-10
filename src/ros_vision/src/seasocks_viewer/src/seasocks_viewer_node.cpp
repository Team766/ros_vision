#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <seasocks/Server.h>
#include <seasocks/WebSocket.h>
#include <seasocks/PrintfLogger.h>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace seasocks;

class ImageWebSocketHandler : public WebSocket::Handler {
public:
    void onConnect(WebSocket* socket) override {
        std::lock_guard<std::mutex> lock(mutex_);
        clients_.insert(socket);
    }

    void onDisconnect(WebSocket* socket) override {
        std::lock_guard<std::mutex> lock(mutex_);
        clients_.erase(socket);
    }

    void sendImage(const std::vector<uint8_t>& imageData) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto* client : clients_) {
            //client->send(reinterpret_cast<const char*>(imageData.data()), imageData.size());
            client->send(imageData.data(), imageData.size());
        }
    }

private:
    std::set<WebSocket*> clients_;
    std::mutex mutex_;
};

class SeasocksViewerNode : public rclcpp::Node {
public:
    SeasocksViewerNode()
        : Node("seasocks_viewer_node") {
        
        // Seasocks setup
        auto logger = std::make_shared<PrintfLogger>();
        server_ = std::make_shared<Server>(logger);
        image_handler_ = std::make_shared<ImageWebSocketHandler>();
        server_->addWebSocketHandler("/image", image_handler_);

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("seasocks_viewer");
        std::string web_root = package_share_directory + "/web";
        
        server_thread_ = std::thread([this, web_root] {
            server_->serve(web_root.c_str(), 9090);  // serve on port 9090
        });

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&SeasocksViewerNode::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Seasocks viewer running at ws://localhost:9090/image");
    }

    ~SeasocksViewerNode() {
        server_->terminate();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        std::vector<uint8_t> buffer;
        cv::imencode(".jpg", cv_ptr->image, buffer);

        image_handler_->sendImage(buffer);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::shared_ptr<Server> server_;
    std::shared_ptr<ImageWebSocketHandler> image_handler_;
    std::thread server_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SeasocksViewerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
