#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"

extern "C" {
    #include "apriltag.h"
    #include "apriltag_pose.h"
    #include "common/zarray.h"
}

#define TAGSIZE 0.1651 // tag size, in meters

class ApriltagsDetector : public rclcpp::Node {
public:
    ApriltagsDetector()
        : Node("apriltags_detector"), tag_family_(nullptr), tag_detector_(nullptr) {

        // Decare parameters
        this->declare_parameter<std::string>("topic_name", "camera/image_raw");
        std::string topic_name = this->get_parameter("topic_name").as_string();

        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name, 1,
            std::bind(&ApriltagsDetector::imageCallback, this, std::placeholders::_1));

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

        //TODO: read these from a file or a topic.
        frc971::apriltag::CameraMatrix cam;
        frc971::apriltag::DistCoeffs dist;

        cam.fx = 902.5192299862339,
        cam.fy = 899.3430316868563;
        cam.cx = 598.0485673849513;
        cam.cy = 398.91164891101846;

        // Setup Distortion Coefficients
        // OpenCV writes them out in the order specified here:
        // https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
        dist.k1 = 0.06574151949799477;
        dist.k2 = -0.12872189270318862;
        dist.p1 = -0.0002926773194706;
        dist.p2 = 0.0025074759286610325;
        dist.k3 = 0.06490397393668323;

        int frame_width = 1280;
        int frame_height = 800;

        auto start = std::chrono::high_resolution_clock::now();
        detector_ = new frc971::apriltag::GpuDetector(frame_width, frame_height, tag_detector_, cam, dist);
        auto end = std::chrono::high_resolution_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        info_.tagsize = TAGSIZE;
        info_.fx = cam.fx;
        info_.fy = cam.fy;
        info_.cx = cam.cx;
        info_.cy = cam.cy;

        RCLCPP_INFO(this->get_logger(), "GPU Apriltag Detector created, took %ld ms", processing_time);
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("apriltags/images", 10);
    }

    ~ApriltagsDetector() {
        apriltag_detector_destroy(tag_detector_);
        teardown_tag_family(&tag_family_, tag_family_name_);
        delete detector_;
    }

private:

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

        cv::Mat yuyv_img;

        auto start = std::chrono::high_resolution_clock::now();
        cv::Mat bgr_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

        cv::cvtColor(bgr_img, yuyv_img, cv::COLOR_BGR2YUV_YUYV);

        auto detstart = std::chrono::high_resolution_clock::now();
        detector_->Detect(yuyv_img.data);
        auto detend = std::chrono::high_resolution_clock::now();
        auto det_processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(detend - detstart).count();

        const zarray_t* detections = detector_->Detections();
        draw_detection_outlines(bgr_img, const_cast<zarray_t*>(detections));

        if (zarray_size(detections) > 0) {
            for (int i = 0; i < zarray_size(detections); i++) {
              apriltag_detection_t* det;
              zarray_get(const_cast<zarray_t*>(detections), i, &det);
  
              // Setup the detection info struct for use down below.
              info_.det = det;
  
              apriltag_pose_t pose;
              double err = estimate_tag_pose(&info_, &pose);

              RCLCPP_INFO(this->get_logger(), "Tag id: %d, x: %.6f, y: %.6f, z: %.6f, err: %.6f",
                det->id, pose.t->data[0], pose.t->data[1], pose.t->data[2], err);
            }
            detector_->ReinitializeDetections();
        }

        // Publish the message to the viewer
        auto outgoing_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_img).toImageMsg();
        outgoing_msg->header.stamp = this->now();
        outgoing_msg->header.frame_id = "apriltag_detections";
        publisher_->publish(*outgoing_msg);

        auto end = std::chrono::high_resolution_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        RCLCPP_INFO(this->get_logger(), "Total Time: %ld ms, Det Time: %ld ms", processing_time, det_processing_time);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    apriltag_family_t* tag_family_;
    apriltag_detector_t* tag_detector_;
    apriltag_detection_info_t info_;
    frc971::apriltag::GpuDetector* detector_;
    const char* tag_family_name_ = "tag36h11";
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagsDetector>());
    rclcpp::shutdown();
    return 0;
}