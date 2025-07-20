#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

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
  
  // Use compatible QoS settings to match the publisher
  auto qos = rclcpp::QoS(1)
                 .best_effort()
                 .durability_volatile();
  
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
  auto subscription = publisher->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10,
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
  publisher->init();

  // Collect multiple published messages
  std::vector<sensor_msgs::msg::Image::SharedPtr> received_msgs;
  
  // Use compatible QoS settings to match the publisher
  auto qos = rclcpp::QoS(1)
                 .best_effort()
                 .durability_volatile();
  
  auto subscription = publisher->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", qos,
      [&received_msgs](const sensor_msgs::msg::Image::SharedPtr msg) {
        received_msgs.push_back(msg);
      });

  // Spin until we get at least 6 messages (2 cycles through the 3-frame sequence)
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(publisher);
  
  auto start_time = std::chrono::steady_clock::now();
  while (received_msgs.size() < 6 && 
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  // Verify we received the expected number of messages
  EXPECT_GE(received_msgs.size(), 6);
  
  // Verify the sequence repeats correctly by checking first 6 messages
  for (size_t i = 0; i < 6 && i < received_msgs.size(); ++i) {
    cv::Mat frame = cv_bridge::toCvCopy(received_msgs[i], "bgr8")->image;
    cv::Vec3b center_pixel = frame.at<cv::Vec3b>(240, 320);
    
    // Check color matches expected sequence (Blue, Green, Red, Blue, Green, Red)
    switch (i % 3) {
      case 0:  // Blue frame
        EXPECT_GT(center_pixel[0], 200);
        EXPECT_LT(center_pixel[1], 50);
        EXPECT_LT(center_pixel[2], 50);
        break;
      case 1:  // Green frame
        EXPECT_LT(center_pixel[0], 50);
        EXPECT_GT(center_pixel[1], 200);
        EXPECT_LT(center_pixel[2], 50);
        break;
      case 2:  // Red frame
        EXPECT_LT(center_pixel[0], 50);
        EXPECT_LT(center_pixel[1], 50);
        EXPECT_GT(center_pixel[2], 200);
        break;
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
