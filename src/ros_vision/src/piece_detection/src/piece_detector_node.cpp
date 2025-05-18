#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class PieceDetector : public rclcpp::Node {
public:
    PieceDetector() : Node("piece_detector") {
        
    }
}