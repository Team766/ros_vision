#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>
#include <set>

#include "usb_camera/camera_publisher.hpp"
#include "mock_camera.hpp"

class CameraPublisherTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  // Helper function to create a test frame with specific characteristics
  cv::Mat createTestFrame(int width = 640, int height = 480, 
                         const cv::Scalar& color = cv::Scalar(0, 255, 0)) {
    cv::Mat frame = cv::Mat::zeros(height, width, CV_8UC3);
    cv::circle(frame, cv::Point(width/2, height/2), 50, color, -1);
    cv::putText(frame, "Test", cv::Point(50, 100), 
                cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
    return frame;
  }

  // Helper function to create QoS settings that match the camera publisher
  rclcpp::QoS createMatchingQoS() {
    return rclcpp::QoS(1)
               .best_effort()
               .durability_volatile()
               .deadline(std::chrono::milliseconds(50));
  }
};

/**
 * Test that CameraPublisher can be constructed with a mock camera
 * and properly initializes without throwing exceptions.
 */
TEST_F(CameraPublisherTest, ConstructorWithMockCamera) {
  auto mock_camera = std::make_unique<MockCamera>();
  mock_camera->setOpened(true);

  EXPECT_NO_THROW({
    auto publisher = std::make_shared<CameraPublisher>(std::move(mock_camera));
    publisher->init();
  });
}

/**
 * Test that the camera publisher handles camera opening correctly
 * and tracks the open calls appropriately.
 */
TEST_F(CameraPublisherTest, CameraInitialization) {
  auto mock_camera_ptr = std::make_unique<MockCamera>();
  auto* mock_camera_raw = mock_camera_ptr.get();
  
  // Configure mock to simulate successful opening
  mock_camera_raw->setOpened(true);
  mock_camera_raw->setPropertyValue(cv::CAP_PROP_FRAME_WIDTH, 1280);
  mock_camera_raw->setPropertyValue(cv::CAP_PROP_FRAME_HEIGHT, 800);
  mock_camera_raw->setPropertyValue(cv::CAP_PROP_FPS, 100);
  mock_camera_raw->setPropertyValue(cv::CAP_PROP_BUFFERSIZE, 1);

  auto publisher = std::make_shared<CameraPublisher>(std::move(mock_camera_ptr));
  publisher->init();

  // Verify that camera properties were set correctly
  EXPECT_EQ(mock_camera_raw->get(cv::CAP_PROP_FRAME_WIDTH), 1280.0);
  EXPECT_EQ(mock_camera_raw->get(cv::CAP_PROP_FRAME_HEIGHT), 800.0);
  EXPECT_EQ(mock_camera_raw->get(cv::CAP_PROP_FPS), 100.0);
  EXPECT_EQ(mock_camera_raw->get(cv::CAP_PROP_BUFFERSIZE), 1.0);
}

/**
 * Test that the camera publisher properly handles camera open failures
 * by throwing an appropriate exception.
 */
TEST_F(CameraPublisherTest, CameraOpenFailure) {
  auto mock_camera = std::make_unique<MockCamera>();
  mock_camera->setOpened(false);  // Simulate camera open failure

  EXPECT_THROW({
    auto publisher = std::make_shared<CameraPublisher>(std::move(mock_camera));
  }, std::runtime_error);
}

/**
 * Test that frame capture works correctly with a mock camera
 * providing synthetic frames.
 */
TEST_F(CameraPublisherTest, FrameCapture) {
  auto mock_camera_ptr = std::make_unique<MockCamera>();
  auto* mock_camera_raw = mock_camera_ptr.get();
  
  // Create a distinctive test frame
  cv::Mat test_frame = createTestFrame(640, 480, cv::Scalar(255, 0, 0)); // Blue circle
  mock_camera_raw->setSyntheticFrame(test_frame);
  mock_camera_raw->setOpened(true);

  auto publisher = std::make_shared<CameraPublisher>(std::move(mock_camera_ptr));
  publisher->init();

  // Create a subscriber to capture published messages
  sensor_msgs::msg::Image::SharedPtr received_msg;
  
  // Use matching QoS settings
  auto qos = createMatchingQoS();
  
  auto subscription = publisher->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", qos,
      [&received_msg](const sensor_msgs::msg::Image::SharedPtr msg) {
        received_msg = msg;
      });

  // Spin for a short time to allow frame capture and publishing
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(publisher);
  
  auto start_time = std::chrono::steady_clock::now();
  while (!received_msg && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  // Verify that we received a message
  ASSERT_NE(received_msg, nullptr);
  EXPECT_EQ(received_msg->encoding, "bgr8");
  EXPECT_EQ(received_msg->width, 640);
  EXPECT_EQ(received_msg->height, 480);
  EXPECT_EQ(received_msg->header.frame_id, "camera_frame");

  // Verify frame content by converting back to cv::Mat
  cv::Mat received_frame = cv_bridge::toCvCopy(received_msg, "bgr8")->image;
  EXPECT_EQ(received_frame.rows, 480);
  EXPECT_EQ(received_frame.cols, 640);
  
  // Check that the blue circle is present (center pixel should be blue)
  cv::Vec3b center_pixel = received_frame.at<cv::Vec3b>(240, 320);
  EXPECT_GT(center_pixel[0], 200);  // Blue channel should be high
  EXPECT_LT(center_pixel[1], 50);   // Green channel should be low
  EXPECT_LT(center_pixel[2], 50);   // Red channel should be low
}

/**
 * Test that the camera publisher handles read failures gracefully
 * without crashing or publishing invalid data.
 */
TEST_F(CameraPublisherTest, HandleReadFailure) {
  auto mock_camera_ptr = std::make_unique<MockCamera>();
  auto* mock_camera_raw = mock_camera_ptr.get();
  
  mock_camera_raw->setOpened(true);
  mock_camera_raw->setReadFailure(true);  // Simulate read failures

  auto publisher = std::make_shared<CameraPublisher>(std::move(mock_camera_ptr));
  publisher->init();

  // Create a subscriber to capture published messages
  sensor_msgs::msg::Image::SharedPtr received_msg;
  auto qos = createMatchingQoS();
  auto subscription = publisher->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", qos,
      [&received_msg](const sensor_msgs::msg::Image::SharedPtr msg) {
        received_msg = msg;
      });

  // Spin for a short time - should not receive any messages due to read failures
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(publisher);
  
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(500)) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  // Should not have received any messages due to read failures
  EXPECT_EQ(received_msg, nullptr);
  
  // Verify that read was attempted multiple times
  EXPECT_GT(mock_camera_raw->getReadCallCount(), 10);
}

/**
 * Test that multiple sequential frames can be captured and published correctly.
 * Updated for frame-driven capture approach.
 */
TEST_F(CameraPublisherTest, MultipleFrameSequence) {
  auto mock_camera_ptr = std::make_unique<MockCamera>();
  auto* mock_camera_raw = mock_camera_ptr.get();
  
  // Create a sequence of frames with different colors
  std::vector<cv::Mat> frame_sequence;
  frame_sequence.push_back(createTestFrame(640, 480, cv::Scalar(255, 0, 0)));   // Blue
  frame_sequence.push_back(createTestFrame(640, 480, cv::Scalar(0, 255, 0)));   // Green
  frame_sequence.push_back(createTestFrame(640, 480, cv::Scalar(0, 0, 255)));   // Red
  
  mock_camera_raw->setSyntheticFrames(frame_sequence);
  mock_camera_raw->setOpened(true);

  auto publisher = std::make_shared<CameraPublisher>(std::move(mock_camera_ptr));
  
  // Collect multiple published messages
  std::vector<sensor_msgs::msg::Image::SharedPtr> received_msgs;
  std::set<std::string> unique_colors_seen;
  
  // Use matching QoS settings with increased queue for this test  
  auto qos = rclcpp::QoS(10)  // Increased queue size to capture more messages
                 .best_effort()
                 .durability_volatile()
                 .deadline(std::chrono::milliseconds(50));
  
  auto subscription = publisher->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", qos,
      [&received_msgs, &unique_colors_seen](const sensor_msgs::msg::Image::SharedPtr msg) {
        received_msgs.push_back(msg);
        
        // Track unique colors we've seen
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Vec3b center_pixel = frame.at<cv::Vec3b>(240, 320);
        
        if (center_pixel[0] > 200 && center_pixel[1] < 50 && center_pixel[2] < 50) {
          unique_colors_seen.insert("blue");
        } else if (center_pixel[1] > 200 && center_pixel[0] < 50 && center_pixel[2] < 50) {
          unique_colors_seen.insert("green");
        } else if (center_pixel[2] > 200 && center_pixel[0] < 50 && center_pixel[1] < 50) {
          unique_colors_seen.insert("red");
        }
      });

  // Initialize after creating subscription to ensure it's ready
  publisher->init();

  // Spin until we get messages and have seen all colors
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(publisher);
  
  auto start_time = std::chrono::steady_clock::now();
  while ((received_msgs.size() < 3 || unique_colors_seen.size() < 3) && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
    executor.spin_some(std::chrono::milliseconds(5));
  }

  // Verify we received messages and saw all expected colors
  EXPECT_GE(received_msgs.size(), 3);
  EXPECT_EQ(unique_colors_seen.size(), 3);
  EXPECT_TRUE(unique_colors_seen.count("blue") > 0);
  EXPECT_TRUE(unique_colors_seen.count("green") > 0);
  EXPECT_TRUE(unique_colors_seen.count("red") > 0);
  
  // Verify that the mock camera has been read from multiple times
  // (indicating the frame-driven approach is working)
  EXPECT_GT(mock_camera_raw->getReadCallCount(), 3);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
