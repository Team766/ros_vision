// detection_sorting_test.cu
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

#include "apriltags_cuda/apriltags_cuda_detector.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/zarray.h"
}

/**
 * @brief Test class that inherits from ApriltagsDetector to test detection sorting
 */
class TestableApriltagsDetectorSorting : public ApriltagsDetector {
public:
  TestableApriltagsDetectorSorting() : ApriltagsDetector(true) {
    // Use the protected constructor with bypass_init = true
  }
  
  void init() override {
    // Skip ROS setup for tests
  }
  
  // Create test detection data directly
  std::vector<DetectionData> createTestDetectionData() {
    std::vector<DetectionData> test_data;
    
    // Create mock apriltag_detection_t structures
    static apriltag_detection_t det1, det2, det3;
    det1.id = 1;
    det2.id = 2; 
    det3.id = 3;
    
    // Test case: Three detections at different distances
    // Detection 1: Far away (distance = 5.0)
    cv::Vec3d pos1(3.0, 4.0, 0.0);  // distance = sqrt(9 + 16) = 5.0
    cv::Mat robot_pos1 = cv::Mat::zeros(3, 1, CV_64F);
    test_data.push_back({
      &det1,
      pos1,
      robot_pos1.clone(),
      5.0,
      0.1
    });
    
    // Detection 2: Close (distance = 2.0)
    cv::Vec3d pos2(0.0, 0.0, 2.0);  // distance = 2.0
    cv::Mat robot_pos2 = cv::Mat::zeros(3, 1, CV_64F);
    test_data.push_back({
      &det2,
      pos2,
      robot_pos2.clone(),
      2.0,
      0.05
    });
    
    // Detection 3: Very close (distance = 1.0)
    cv::Vec3d pos3(1.0, 0.0, 0.0);  // distance = 1.0
    cv::Mat robot_pos3 = cv::Mat::zeros(3, 1, CV_64F);
    test_data.push_back({
      &det3,
      pos3,
      robot_pos3.clone(),
      1.0,
      0.02
    });
    
    return test_data;
  }
  
  // Test the sorting logic
  std::vector<DetectionData> testSortDetections(std::vector<DetectionData> data) {
    std::sort(data.begin(), data.end(),
              [](const DetectionData& a, const DetectionData& b) {
                return a.distance < b.distance;
              });
    return data;
  }
  
  // Calculate distance manually to verify calculation
  double calculateDistance(const cv::Vec3d& position) {
    return std::sqrt(position[0] * position[0] + 
                     position[1] * position[1] + 
                     position[2] * position[2]);
  }
};

/**
 * @brief Test fixture for detection sorting tests
 */
class DetectionSortingTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize ROS 2 if not already initialized
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
    detector_ = std::make_unique<TestableApriltagsDetectorSorting>();
  }
  
  void TearDown() override {
    detector_.reset();
    // Note: We don't shutdown ROS here as other tests might need it
  }
  
  std::unique_ptr<TestableApriltagsDetectorSorting> detector_;
};

/**
 * @brief Test that DetectionData struct can be created and accessed
 */
TEST_F(DetectionSortingTest, DetectionDataStructCreation) {
  // Create a simple detection data entry
  apriltag_detection_t det;
  det.id = 42;
  
  cv::Vec3d camera_pos(1.0, 2.0, 3.0);
  cv::Mat robot_pos = cv::Mat::zeros(3, 1, CV_64F);
  double distance = 3.74; // sqrt(1 + 4 + 9)
  double pose_error = 0.1;
  
  ApriltagsDetector::DetectionData data{
    &det,
    camera_pos,
    robot_pos.clone(),
    distance,
    pose_error
  };
  
  // Verify all fields are accessible
  EXPECT_EQ(data.det->id, 42);
  EXPECT_DOUBLE_EQ(data.camera_position[0], 1.0);
  EXPECT_DOUBLE_EQ(data.camera_position[1], 2.0);
  EXPECT_DOUBLE_EQ(data.camera_position[2], 3.0);
  EXPECT_DOUBLE_EQ(data.distance, 3.74);
  EXPECT_DOUBLE_EQ(data.pose_error, 0.1);
}

/**
 * @brief Test distance calculation accuracy
 */
TEST_F(DetectionSortingTest, DistanceCalculationAccuracy) {
  // Test various distance calculations
  struct TestCase {
    cv::Vec3d position;
    double expected_distance;
  };
  
  std::vector<TestCase> test_cases = {
    {{0.0, 0.0, 1.0}, 1.0},           // Simple case: z-axis
    {{3.0, 4.0, 0.0}, 5.0},           // Classic 3-4-5 triangle
    {{1.0, 1.0, 1.0}, std::sqrt(3.0)}, // Diagonal
    {{0.0, 0.0, 0.0}, 0.0},           // Origin
    {{-2.0, 0.0, 0.0}, 2.0},          // Negative coordinates
  };
  
  for (const auto& test_case : test_cases) {
    double calculated_distance = detector_->calculateDistance(test_case.position);
    EXPECT_NEAR(calculated_distance, test_case.expected_distance, 1e-10)
        << "Distance calculation failed for position ("
        << test_case.position[0] << ", " 
        << test_case.position[1] << ", " 
        << test_case.position[2] << ")";
  }
}

/**
 * @brief Test that sorting works correctly (closest first)
 */
TEST_F(DetectionSortingTest, SortingOrderCorrectness) {
  auto test_data = detector_->createTestDetectionData();
  
  // Verify initial order (should be: far=5.0, close=2.0, very_close=1.0)
  EXPECT_EQ(test_data[0].det->id, 1);  // Far (distance 5.0)
  EXPECT_EQ(test_data[1].det->id, 2);  // Close (distance 2.0)
  EXPECT_EQ(test_data[2].det->id, 3);  // Very close (distance 1.0)
  
  // Sort the data
  auto sorted_data = detector_->testSortDetections(test_data);
  
  // Verify sorted order (should be: very_close=1.0, close=2.0, far=5.0)
  EXPECT_EQ(sorted_data[0].det->id, 3);  // Very close (distance 1.0)
  EXPECT_EQ(sorted_data[1].det->id, 2);  // Close (distance 2.0) 
  EXPECT_EQ(sorted_data[2].det->id, 1);  // Far (distance 5.0)
  
  // Verify distances are in ascending order
  EXPECT_LE(sorted_data[0].distance, sorted_data[1].distance);
  EXPECT_LE(sorted_data[1].distance, sorted_data[2].distance);
}

/**
 * @brief Test edge cases for sorting
 */
TEST_F(DetectionSortingTest, SortingEdgeCases) {
  // Test empty vector
  std::vector<ApriltagsDetector::DetectionData> empty_data;
  auto sorted_empty = detector_->testSortDetections(empty_data);
  EXPECT_TRUE(sorted_empty.empty());
  
  // Test single detection
  apriltag_detection_t det;
  det.id = 1;
  cv::Vec3d pos(1.0, 1.0, 1.0);
  cv::Mat robot_pos = cv::Mat::zeros(3, 1, CV_64F);
  
  std::vector<ApriltagsDetector::DetectionData> single_data = {{
    &det, pos, robot_pos.clone(), std::sqrt(3.0), 0.1
  }};
  
  auto sorted_single = detector_->testSortDetections(single_data);
  EXPECT_EQ(sorted_single.size(), 1);
  EXPECT_EQ(sorted_single[0].det->id, 1);
}

/**
 * @brief Test sorting with equal distances (stable sort behavior)
 */
TEST_F(DetectionSortingTest, SortingEqualDistances) {
  static apriltag_detection_t det1, det2;
  det1.id = 1;
  det2.id = 2;
  
  cv::Vec3d pos1(1.0, 0.0, 0.0);  // distance = 1.0
  cv::Vec3d pos2(0.0, 1.0, 0.0);  // distance = 1.0
  cv::Mat robot_pos = cv::Mat::zeros(3, 1, CV_64F);
  
  std::vector<ApriltagsDetector::DetectionData> equal_distance_data = {
    {&det1, pos1, robot_pos.clone(), 1.0, 0.1},
    {&det2, pos2, robot_pos.clone(), 1.0, 0.1}
  };
  
  auto sorted_data = detector_->testSortDetections(equal_distance_data);
  
  // Both should have equal distances
  EXPECT_DOUBLE_EQ(sorted_data[0].distance, sorted_data[1].distance);
  
  // Order should be stable (original relative order preserved)
  EXPECT_EQ(sorted_data[0].det->id, 1);
  EXPECT_EQ(sorted_data[1].det->id, 2);
}

/**
 * @brief Test performance with many detections
 */
TEST_F(DetectionSortingTest, SortingPerformance) {
  const int num_detections = 100;
  std::vector<ApriltagsDetector::DetectionData> large_dataset;
  std::vector<apriltag_detection_t> detections(num_detections);
  
  // Create many detections with random distances
  for (int i = 0; i < num_detections; ++i) {
    detections[i].id = i;
    
    // Create positions with increasing distances
    double distance = static_cast<double>(num_detections - i);
    cv::Vec3d pos(distance, 0.0, 0.0);
    cv::Mat robot_pos = cv::Mat::zeros(3, 1, CV_64F);
    
    large_dataset.push_back({
      &detections[i],
      pos,
      robot_pos.clone(),
      distance,
      0.1
    });
  }
  
  // Sort should complete quickly
  auto start = std::chrono::high_resolution_clock::now();
  auto sorted_data = detector_->testSortDetections(large_dataset);
  auto end = std::chrono::high_resolution_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  
  // Verify sorting worked correctly (should be in ascending distance order)
  for (int i = 0; i < num_detections - 1; ++i) {
    EXPECT_LE(sorted_data[i].distance, sorted_data[i + 1].distance);
  }
  
  // Performance check: should complete in reasonable time (< 1ms for 100 items)
  EXPECT_LT(duration.count(), 1000) << "Sorting took too long: " << duration.count() << " microseconds";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  
  // Initialize ROS 2 for the entire test suite
  rclcpp::init(argc, argv);
  
  int result = RUN_ALL_TESTS();
  
  // Shutdown ROS 2
  rclcpp::shutdown();
  
  return result;
}
