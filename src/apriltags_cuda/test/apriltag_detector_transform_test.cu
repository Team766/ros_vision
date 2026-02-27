// apriltag_detector_transform_test.cu
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include "apriltags_cuda/apriltags_cuda_detector.hpp"

/**
 * @brief Test class that inherits from ApriltagsDetector to test transformation logic
 * 
 * This approach tests the actual detector class rather than a mock, giving us confidence
 * that the real transformation method works correctly. We bypass the config loading 
 * to avoid file dependencies in tests.
 */
class TestableApriltagsDetector : public ApriltagsDetector {
public:
  TestableApriltagsDetector() : ApriltagsDetector(true) {
    // Use the protected constructor with bypass_init = true
    // This avoids all the config file loading and ROS setup
  }
  
  // Override init() to prevent config file loading during tests
  void init() override {
    // Do nothing - skip all the file loading and ROS setup for tests
    // This allows us to test just the transformation logic
  }
  
  // Expose protected methods and members for testing
  void setExtrinsicRotation(const cv::Mat& rotation) {
    extrinsic_rotation_ = rotation;
  }
  
  void setExtrinsicOffset(const cv::Mat& offset) {
    extrinsic_offset_ = offset;
  }
  
  cv::Mat getExtrinsicRotation() const {
    return extrinsic_rotation_;
  }
  
  cv::Mat getExtrinsicOffset() const {
    return extrinsic_offset_;
  }
  
  // Expose the protected transformation method for testing
  cv::Mat testTransformCameraToRobot(const cv::Vec3d& camera_point) const {
    return transformCameraToRobot(camera_point);
  }
};

/**
 * @brief Test fixture for transformation tests using the real detector class
 */
class ApriltagDetectorTransformTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize ROS 2 if not already initialized
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    
    // Create detector instance (using the real class)
    detector_ = std::make_shared<TestableApriltagsDetector>();
    
    // Set up test scenarios
    setupTestTransforms();
  }

  void TearDown() override {
    detector_.reset();
  }

  void setupTestTransforms() {
    // Identity transform
    identity_rotation_ = cv::Mat::eye(3, 3, CV_64F);
    identity_offset_ = cv::Mat::zeros(3, 1, CV_64F);
    
    // 90-degree rotation around Z-axis
    rotation_90z_ = cv::Mat::zeros(3, 3, CV_64F);
    rotation_90z_.at<double>(0, 1) = -1.0; // cos(90°) = 0, -sin(90°) = -1
    rotation_90z_.at<double>(1, 0) = 1.0;  // sin(90°) = 1, cos(90°) = 0
    rotation_90z_.at<double>(2, 2) = 1.0;  // Z unchanged
    
    // Translation only
    translation_offset_ = (cv::Mat_<double>(3, 1) << 1.5, -2.0, 0.5);
    
    // Real-world-like transformation (camera mounted at an angle)
    // 30-degree rotation around X (camera tilted down) + translation
    double angle = 30.0 * M_PI / 180.0; // 30 degrees in radians
    realistic_rotation_ = cv::Mat::eye(3, 3, CV_64F);
    realistic_rotation_.at<double>(1, 1) = cos(angle);
    realistic_rotation_.at<double>(1, 2) = -sin(angle);
    realistic_rotation_.at<double>(2, 1) = sin(angle);
    realistic_rotation_.at<double>(2, 2) = cos(angle);
    realistic_offset_ = (cv::Mat_<double>(3, 1) << 0.2, 0.0, 0.3); // 20cm forward, 30cm up
  }

  std::shared_ptr<TestableApriltagsDetector> detector_;
  cv::Mat identity_rotation_, identity_offset_;
  cv::Mat rotation_90z_;
  cv::Mat translation_offset_;
  cv::Mat realistic_rotation_, realistic_offset_;
  
  static constexpr double TOLERANCE = 1e-10;
  static constexpr double REALISTIC_TOLERANCE = 1e-6; // Slightly more lenient for realistic scenarios
};

// Test 1: Identity transformation in detector
TEST_F(ApriltagDetectorTransformTest, DetectorIdentityTransform) {
  detector_->setExtrinsicRotation(identity_rotation_);
  detector_->setExtrinsicOffset(identity_offset_);
  
  cv::Vec3d camera_pos(1.0, 2.0, 3.0);
  cv::Mat robot_pos = detector_->testTransformCameraToRobot(camera_pos);
  
  EXPECT_NEAR(robot_pos.at<double>(0), 1.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), 2.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 3.0, TOLERANCE);
}

// Test 2: Rotation-only transformation in detector
TEST_F(ApriltagDetectorTransformTest, DetectorRotationTransform) {
  detector_->setExtrinsicRotation(rotation_90z_);
  detector_->setExtrinsicOffset(identity_offset_);
  
  cv::Vec3d camera_pos(1.0, 0.0, 0.0);
  cv::Mat robot_pos = detector_->testTransformCameraToRobot(camera_pos);
  
  // 90° rotation around Z: [1,0,0] → [0,1,0]
  EXPECT_NEAR(robot_pos.at<double>(0), 0.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), 1.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 0.0, TOLERANCE);
}

// Test 3: Translation-only transformation in detector
TEST_F(ApriltagDetectorTransformTest, DetectorTranslationTransform) {
  detector_->setExtrinsicRotation(identity_rotation_);
  detector_->setExtrinsicOffset(translation_offset_);
  
  cv::Vec3d camera_pos(0.0, 0.0, 0.0);
  cv::Mat robot_pos = detector_->testTransformCameraToRobot(camera_pos);
  
  EXPECT_NEAR(robot_pos.at<double>(0), 1.5, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), -2.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 0.5, TOLERANCE);
}

// Test 4: Realistic transformation scenario
TEST_F(ApriltagDetectorTransformTest, DetectorRealisticTransform) {
  detector_->setExtrinsicRotation(realistic_rotation_);
  detector_->setExtrinsicOffset(realistic_offset_);
  
  // AprilTag directly in front of camera at 1 meter distance
  cv::Vec3d camera_pos(0.0, 0.0, 1.0);
  cv::Mat robot_pos = detector_->testTransformCameraToRobot(camera_pos);
  
  // With 30-degree downward tilt:
  // Z component should be reduced due to tilt
  // Y component should be negative (tag appears "below" robot frame)
  // X component should be offset by 0.2m (camera forward offset)
  
  double angle = 30.0 * M_PI / 180.0;
  double expected_x = 0.0 + 0.2;  // No X rotation + offset
  double expected_y = 0.0 * cos(angle) - 1.0 * sin(angle) + 0.0;  // Y rotation + offset
  double expected_z = 0.0 * sin(angle) + 1.0 * cos(angle) + 0.3;  // Z rotation + offset
  
  EXPECT_NEAR(robot_pos.at<double>(0), expected_x, REALISTIC_TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), expected_y, REALISTIC_TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), expected_z, REALISTIC_TOLERANCE);
}

// Test 5: Multiple AprilTags at different positions
TEST_F(ApriltagDetectorTransformTest, DetectorMultipleTagPositions) {
  detector_->setExtrinsicRotation(rotation_90z_);
  detector_->setExtrinsicOffset(translation_offset_);
  
  std::vector<cv::Vec3d> camera_positions = {
    cv::Vec3d(1.0, 0.0, 0.0),   // Tag 1
    cv::Vec3d(0.0, 1.0, 0.0),   // Tag 2
    cv::Vec3d(-1.0, 0.0, 0.0),  // Tag 3
    cv::Vec3d(0.0, -1.0, 0.0)   // Tag 4
  };
  
  std::vector<cv::Vec3d> expected_robot_positions = {
    cv::Vec3d(1.5, -1.0, 0.5),  // [0,1,0] + offset = [0+1.5, 1+(-2), 0+0.5]
    cv::Vec3d(0.5, -2.0, 0.5),  // [-1,0,0] + offset = [-1+1.5, 0+(-2), 0+0.5]  
    cv::Vec3d(1.5, -3.0, 0.5),  // [0,-1,0] + offset = [0+1.5, -1+(-2), 0+0.5]
    cv::Vec3d(2.5, -2.0, 0.5)   // [1,0,0] + offset = [1+1.5, 0+(-2), 0+0.5]
  };
  
  for (size_t i = 0; i < camera_positions.size(); ++i) {
    cv::Mat robot_pos = detector_->testTransformCameraToRobot(camera_positions[i]);
    
    EXPECT_NEAR(robot_pos.at<double>(0), expected_robot_positions[i][0], TOLERANCE)
        << "Tag " << i + 1 << " X position mismatch";
    EXPECT_NEAR(robot_pos.at<double>(1), expected_robot_positions[i][1], TOLERANCE)
        << "Tag " << i + 1 << " Y position mismatch";
    EXPECT_NEAR(robot_pos.at<double>(2), expected_robot_positions[i][2], TOLERANCE)
        << "Tag " << i + 1 << " Z position mismatch";
  }
}

// Test 6: Consistency check - same transformation applied twice
TEST_F(ApriltagDetectorTransformTest, DetectorTransformationConsistency) {
  detector_->setExtrinsicRotation(realistic_rotation_);
  detector_->setExtrinsicOffset(realistic_offset_);
  
  cv::Vec3d camera_pos(0.5, -0.3, 1.2);
  
  // Apply transformation multiple times - should get same result
  cv::Mat result1 = detector_->testTransformCameraToRobot(camera_pos);
  cv::Mat result2 = detector_->testTransformCameraToRobot(camera_pos);
  cv::Mat result3 = detector_->testTransformCameraToRobot(camera_pos);
  
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(result1.at<double>(i), result2.at<double>(i), TOLERANCE);
    EXPECT_NEAR(result2.at<double>(i), result3.at<double>(i), TOLERANCE);
  }
}

// Test 7: Verify setters and getters work correctly
TEST_F(ApriltagDetectorTransformTest, DetectorExtrinsicSettersGetters) {
  detector_->setExtrinsicRotation(realistic_rotation_);
  detector_->setExtrinsicOffset(realistic_offset_);
  
  cv::Mat retrieved_rotation = detector_->getExtrinsicRotation();
  cv::Mat retrieved_offset = detector_->getExtrinsicOffset();
  
  // Check rotation matrix
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(retrieved_rotation.at<double>(i, j), realistic_rotation_.at<double>(i, j), TOLERANCE);
    }
  }
  
  // Check offset vector
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(retrieved_offset.at<double>(i, 0), realistic_offset_.at<double>(i, 0), TOLERANCE);
  }
}

// Test 8: Edge case - very large distances (simulating far AprilTags)
TEST_F(ApriltagDetectorTransformTest, DetectorLargeDistanceTransform) {
  detector_->setExtrinsicRotation(identity_rotation_);
  detector_->setExtrinsicOffset(identity_offset_);
  
  cv::Vec3d camera_pos(0.0, 0.0, 100.0); // 100 meters away
  cv::Mat robot_pos = detector_->testTransformCameraToRobot(camera_pos);
  
  EXPECT_NEAR(robot_pos.at<double>(0), 0.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), 0.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 100.0, TOLERANCE);
}

// Test 9: Edge case - very small distances (close AprilTags)
TEST_F(ApriltagDetectorTransformTest, DetectorSmallDistanceTransform) {
  detector_->setExtrinsicRotation(identity_rotation_);
  detector_->setExtrinsicOffset(identity_offset_);
  
  cv::Vec3d camera_pos(0.01, 0.02, 0.1); // 10cm away
  cv::Mat robot_pos = detector_->testTransformCameraToRobot(camera_pos);
  
  EXPECT_NEAR(robot_pos.at<double>(0), 0.01, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), 0.02, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 0.1, TOLERANCE);
}

// === Quaternion conversion tests ===

// Test: Identity rotation → quaternion (1, 0, 0, 0)
TEST_F(ApriltagDetectorTransformTest, QuaternionIdentity) {
  cv::Vec4d q = ApriltagsDetector::rotationMatrixToQuaternion(identity_rotation_);
  EXPECT_NEAR(q[0], 1.0, TOLERANCE);  // w
  EXPECT_NEAR(q[1], 0.0, TOLERANCE);  // x
  EXPECT_NEAR(q[2], 0.0, TOLERANCE);  // y
  EXPECT_NEAR(q[3], 0.0, TOLERANCE);  // z
}

// Test: 90° rotation around Z → quaternion (cos(45°), 0, 0, sin(45°))
TEST_F(ApriltagDetectorTransformTest, Quaternion90DegZ) {
  cv::Vec4d q = ApriltagsDetector::rotationMatrixToQuaternion(rotation_90z_);
  double expected_w = std::cos(M_PI / 4.0);  // cos(45°)
  double expected_z = std::sin(M_PI / 4.0);  // sin(45°)
  EXPECT_NEAR(q[0], expected_w, REALISTIC_TOLERANCE);
  EXPECT_NEAR(q[1], 0.0, REALISTIC_TOLERANCE);
  EXPECT_NEAR(q[2], 0.0, REALISTIC_TOLERANCE);
  EXPECT_NEAR(q[3], expected_z, REALISTIC_TOLERANCE);
}

// Test: 180° rotation around X (trace = -1 edge case)
TEST_F(ApriltagDetectorTransformTest, Quaternion180DegX) {
  cv::Mat rot180x = cv::Mat::eye(3, 3, CV_64F);
  rot180x.at<double>(1, 1) = -1.0;
  rot180x.at<double>(2, 2) = -1.0;

  cv::Vec4d q = ApriltagsDetector::rotationMatrixToQuaternion(rot180x);
  // Expected: (0, 1, 0, 0) — pure 180° rotation around X
  EXPECT_NEAR(q[0], 0.0, REALISTIC_TOLERANCE);
  EXPECT_NEAR(std::abs(q[1]), 1.0, REALISTIC_TOLERANCE);  // sign ambiguity is OK
  EXPECT_NEAR(q[2], 0.0, REALISTIC_TOLERANCE);
  EXPECT_NEAR(q[3], 0.0, REALISTIC_TOLERANCE);
}

// Test: Composed rotation (extrinsic * tag_rotation) → quaternion
TEST_F(ApriltagDetectorTransformTest, QuaternionComposedRotation) {
  // 90° around Z (extrinsic) * 90° around X (tag) should compose correctly
  cv::Mat rot90x = cv::Mat::eye(3, 3, CV_64F);
  rot90x.at<double>(1, 1) = 0.0;
  rot90x.at<double>(1, 2) = -1.0;
  rot90x.at<double>(2, 1) = 1.0;
  rot90x.at<double>(2, 2) = 0.0;

  cv::Mat composed = rotation_90z_ * rot90x;
  cv::Vec4d q = ApriltagsDetector::rotationMatrixToQuaternion(composed);

  // Verify it's a valid unit quaternion
  double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  EXPECT_NEAR(norm, 1.0, REALISTIC_TOLERANCE);

  // Verify by converting back: q should reproduce the composed rotation matrix
  // R = I + 2w*[x,y,z]x + 2*[x,y,z]x^2  (Rodrigues form from quaternion)
  double w = q[0], x = q[1], y = q[2], z = q[3];
  cv::Mat reconstructed = (cv::Mat_<double>(3, 3) <<
    1-2*(y*y+z*z),  2*(x*y-w*z),    2*(x*z+w*y),
    2*(x*y+w*z),    1-2*(x*x+z*z),  2*(y*z-w*x),
    2*(x*z-w*y),    2*(y*z+w*x),    1-2*(x*x+y*y));

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(reconstructed.at<double>(i, j), composed.at<double>(i, j),
                  REALISTIC_TOLERANCE)
          << "Mismatch at (" << i << "," << j << ")";
    }
  }
}

// Test: Quaternion is always unit-length for various rotations
TEST_F(ApriltagDetectorTransformTest, QuaternionNormalization) {
  std::vector<cv::Mat> rotations = {
    identity_rotation_, rotation_90z_, realistic_rotation_
  };

  // Add a few more arbitrary rotations
  for (double angle : {45.0, 120.0, 270.0}) {
    double rad = angle * M_PI / 180.0;
    cv::Mat r = cv::Mat::eye(3, 3, CV_64F);
    r.at<double>(0, 0) = std::cos(rad);
    r.at<double>(0, 2) = std::sin(rad);
    r.at<double>(2, 0) = -std::sin(rad);
    r.at<double>(2, 2) = std::cos(rad);
    rotations.push_back(r);
  }

  for (const auto& rot : rotations) {
    cv::Vec4d q = ApriltagsDetector::rotationMatrixToQuaternion(rot);
    double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    EXPECT_NEAR(norm, 1.0, REALISTIC_TOLERANCE);
  }
}

// Main function to run the tests
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  
  // Initialize ROS 2 for the test
  rclcpp::init(argc, argv);
  
  int result = RUN_ALL_TESTS();
  
  // Clean up ROS 2
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  
  return result;
}
