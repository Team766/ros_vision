#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher()
        : Node("camera_publisher"), cap_(0) {

        RCLCPP_INFO(this->get_logger(), "Opening camera on idx: '%d'", 0);
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera");
            throw std::runtime_error("Camera not opened");
        }

        RCLCPP_INFO(this->get_logger(), "Width: '%d'", static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)));
        RCLCPP_INFO(this->get_logger(), "Height: '%d'", static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));
        RCLCPP_INFO(this->get_logger(), "FPS: '%d'", static_cast<int>(cap_.get(cv::CAP_PROP_FPS)));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // ~100fps
            std::bind(&CameraPublisher::timerCallback, this));
    }

private:
    void timerCallback() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();

        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}