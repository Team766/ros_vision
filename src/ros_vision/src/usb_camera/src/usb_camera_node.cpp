#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher()
        : Node("camera_publisher"), cap_() {

        // Decare parameters
        this->declare_parameter<int>("camera_idx", 0);
        int camera_idx = this->get_parameter("camera_idx").as_int();

        this->declare_parameter<std::string>("topic_name", "camera/image_raw");
        topic_name_ = this->get_parameter("topic_name").as_string();

        RCLCPP_INFO(this->get_logger(), "Opening camera on idx: '%d'", camera_idx);
        cap_.open(camera_idx, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera");
            throw std::runtime_error("Camera not opened");
        }

        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        cap_.set(cv::CAP_PROP_FOURCC, fourcc);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
        cap_.set(cv::CAP_PROP_CONVERT_RGB, true);

        RCLCPP_INFO(this->get_logger(), "Width: '%d'", static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)));
        RCLCPP_INFO(this->get_logger(), "Height: '%d'", static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));
        RCLCPP_INFO(this->get_logger(), "FPS: '%d'", static_cast<int>(cap_.get(cv::CAP_PROP_FPS)));
        RCLCPP_INFO(this->get_logger(), "Pubishing on Topic: '%s'", topic_name_.c_str());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(16),  // ~60fps
            std::bind(&CameraPublisher::timerCallback, this));
    }

    void init() {
        // This can't be in the constructor because of the call to shared_from_this.
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        publisher_ = it_->advertise(topic_name_, 10);

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
        msg->header.frame_id = "camera_frame";  // <-- add this line

        publisher_.publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher publisher_;

    std::string topic_name_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}