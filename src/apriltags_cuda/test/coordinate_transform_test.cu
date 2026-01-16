// coordinate_transform_test.cu
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <cmath>

/**
 * @brief Test fixture for coordinate transformation tests
 * 
 * This tests the transformation from camera frame to robot frame using
 * known extrinsic parameters (rotation matrix and offset vector).
 */
class CoordinateTransformTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Set up known extrinsic parameters for testing
    setupIdentityTransform();
    setupRotationOnlyTransform(); 
    setupTranslationOnlyTransform();
    setupComplexTransform();
  }

  void TearDown() override {
    // Cleanup if needed
  }

  /**
   * @brief Apply the same transformation logic as in ApriltagsDetector::imageCallback
   */
  cv::Mat transformCameraToRobot(const cv::Vec3d& camera_pos, 
                                const cv::Mat& rotation, 
                                const cv::Mat& offset) {
    cv::Mat camera_pos_mat = cv::Mat(camera_pos);
    return rotation * camera_pos_mat + offset;
  }

  /**
   * @brief Helper to create a 3x3 rotation matrix around Z-axis
   */
  cv::Mat createRotationZ(double angle_rad) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);
    
    R.at<double>(0, 0) = cos_a;  R.at<double>(0, 1) = -sin_a; R.at<double>(0, 2) = 0;
    R.at<double>(1, 0) = sin_a;  R.at<double>(1, 1) = cos_a;  R.at<double>(1, 2) = 0;
    R.at<double>(2, 0) = 0;      R.at<double>(2, 1) = 0;      R.at<double>(2, 2) = 1;
    
    return R;
  }

  /**
   * @brief Helper to create a 3x3 rotation matrix around Y-axis
   */
  cv::Mat createRotationY(double angle_rad) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);
    
    R.at<double>(0, 0) = cos_a;   R.at<double>(0, 1) = 0; R.at<double>(0, 2) = sin_a;
    R.at<double>(1, 0) = 0;       R.at<double>(1, 1) = 1; R.at<double>(1, 2) = 0;
    R.at<double>(2, 0) = -sin_a;  R.at<double>(2, 1) = 0; R.at<double>(2, 2) = cos_a;
    
    return R;
  }

  /**
   * @brief Helper to create a 3x3 rotation matrix around X-axis
   */
  cv::Mat createRotationX(double angle_rad) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);
    
    R.at<double>(0, 0) = 1; R.at<double>(0, 1) = 0;      R.at<double>(0, 2) = 0;
    R.at<double>(1, 0) = 0; R.at<double>(1, 1) = cos_a;  R.at<double>(1, 2) = -sin_a;
    R.at<double>(2, 0) = 0; R.at<double>(2, 1) = sin_a;  R.at<double>(2, 2) = cos_a;
    
    return R;
  }

  void setupIdentityTransform() {
    // Identity transformation (no rotation, no translation)
    identity_rotation_ = cv::Mat::eye(3, 3, CV_64F);
    identity_offset_ = cv::Mat::zeros(3, 1, CV_64F);
  }

  void setupRotationOnlyTransform() {
    // 90-degree rotation around Z-axis, no translation
    rotation_only_rotation_ = createRotationZ(M_PI / 2.0);
    rotation_only_offset_ = cv::Mat::zeros(3, 1, CV_64F);
  }

  void setupTranslationOnlyTransform() {
    // No rotation, translation by [1, 2, 3]
    translation_only_rotation_ = cv::Mat::eye(3, 3, CV_64F);
    translation_only_offset_ = (cv::Mat_<double>(3, 1) << 1.0, 2.0, 3.0);
  }

  void setupComplexTransform() {
    // Complex transformation: 45-degree rotation around Z, then translation
    complex_rotation_ = createRotationZ(M_PI / 4.0);
    complex_offset_ = (cv::Mat_<double>(3, 1) << 0.5, -0.5, 1.0);
  }

  // Test transformation matrices
  cv::Mat identity_rotation_, identity_offset_;
  cv::Mat rotation_only_rotation_, rotation_only_offset_;
  cv::Mat translation_only_rotation_, translation_only_offset_;
  cv::Mat complex_rotation_, complex_offset_;
  
  // Tolerance for floating point comparisons
  static constexpr double TOLERANCE = 1e-10;
};

// Test 1: Identity transformation should preserve coordinates
TEST_F(CoordinateTransformTest, IdentityTransformPreservesCoordinates) {
  cv::Vec3d camera_pos(1.0, 2.0, 3.0);
  
  cv::Mat robot_pos = transformCameraToRobot(camera_pos, identity_rotation_, identity_offset_);
  
  EXPECT_NEAR(robot_pos.at<double>(0), 1.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), 2.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 3.0, TOLERANCE);
}

// Test 2: Translation-only transformation
TEST_F(CoordinateTransformTest, TranslationOnlyTransform) {
  cv::Vec3d camera_pos(1.0, 2.0, 3.0);
  
  cv::Mat robot_pos = transformCameraToRobot(camera_pos, translation_only_rotation_, translation_only_offset_);
  
  // Should be camera_pos + offset = [1,2,3] + [1,2,3] = [2,4,6]
  EXPECT_NEAR(robot_pos.at<double>(0), 2.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), 4.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 6.0, TOLERANCE);
}

// Test 3: Rotation-only transformation (90° around Z-axis)
TEST_F(CoordinateTransformTest, RotationOnlyTransform) {
  cv::Vec3d camera_pos(1.0, 0.0, 0.0);
  
  cv::Mat robot_pos = transformCameraToRobot(camera_pos, rotation_only_rotation_, rotation_only_offset_);
  
  // 90° rotation around Z: [1,0,0] → [0,1,0]
  EXPECT_NEAR(robot_pos.at<double>(0), 0.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), 1.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 0.0, TOLERANCE);
}

// Test 4: Another rotation test with different input
TEST_F(CoordinateTransformTest, RotationOnlyTransformYAxis) {
  cv::Vec3d camera_pos(0.0, 1.0, 0.0);
  
  cv::Mat robot_pos = transformCameraToRobot(camera_pos, rotation_only_rotation_, rotation_only_offset_);
  
  // 90° rotation around Z: [0,1,0] → [-1,0,0]
  EXPECT_NEAR(robot_pos.at<double>(0), -1.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), 0.0, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 0.0, TOLERANCE);
}

// Test 5: Complex transformation (rotation + translation)
TEST_F(CoordinateTransformTest, ComplexTransform) {
  cv::Vec3d camera_pos(1.0, 0.0, 0.0);
  
  cv::Mat robot_pos = transformCameraToRobot(camera_pos, complex_rotation_, complex_offset_);
  
  // 45° rotation around Z: [1,0,0] → [cos(45°), sin(45°), 0] = [√2/2, √2/2, 0]
  // Then add offset [0.5, -0.5, 1.0]
  double sqrt2_over_2 = sqrt(2.0) / 2.0;
  double expected_x = sqrt2_over_2 + 0.5;
  double expected_y = sqrt2_over_2 - 0.5;
  double expected_z = 0.0 + 1.0;
  
  EXPECT_NEAR(robot_pos.at<double>(0), expected_x, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), expected_y, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), expected_z, TOLERANCE);
}

// Test 6: Zero input should work correctly
TEST_F(CoordinateTransformTest, ZeroInputTransform) {
  cv::Vec3d camera_pos(0.0, 0.0, 0.0);
  
  cv::Mat robot_pos = transformCameraToRobot(camera_pos, complex_rotation_, complex_offset_);
  
  // With zero input, result should just be the offset
  EXPECT_NEAR(robot_pos.at<double>(0), 0.5, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(1), -0.5, TOLERANCE);
  EXPECT_NEAR(robot_pos.at<double>(2), 1.0, TOLERANCE);
}

// Test 7: Transformation preserves distances from origin when no translation
TEST_F(CoordinateTransformTest, RotationPreservesDistances) {
  cv::Vec3d camera_pos(3.0, 4.0, 0.0); // Distance from origin = 5.0
  
  cv::Mat robot_pos = transformCameraToRobot(camera_pos, rotation_only_rotation_, rotation_only_offset_);
  
  // Calculate distance from origin in robot frame
  double robot_distance = sqrt(
    robot_pos.at<double>(0) * robot_pos.at<double>(0) +
    robot_pos.at<double>(1) * robot_pos.at<double>(1) +
    robot_pos.at<double>(2) * robot_pos.at<double>(2)
  );
  
  EXPECT_NEAR(robot_distance, 5.0, TOLERANCE);
}

// Test 8: Verify inverse transformation property
TEST_F(CoordinateTransformTest, InverseTransformationProperty) {
  cv::Vec3d original_pos(2.0, 3.0, 1.0);
  
  // Apply forward transformation
  cv::Mat robot_pos = transformCameraToRobot(original_pos, complex_rotation_, complex_offset_);
  
  // Apply inverse transformation (R^T * (pos - offset))
  cv::Mat inverse_rotation = complex_rotation_.t(); // Transpose for inverse rotation
  cv::Mat recovered_pos = inverse_rotation * (robot_pos - complex_offset_);
  
  // Should recover original position
  EXPECT_NEAR(recovered_pos.at<double>(0), original_pos[0], TOLERANCE);
  EXPECT_NEAR(recovered_pos.at<double>(1), original_pos[1], TOLERANCE);
  EXPECT_NEAR(recovered_pos.at<double>(2), original_pos[2], TOLERANCE);
}

// Test 9: Test multiple successive rotations
TEST_F(CoordinateTransformTest, MultipleRotationsComposition) {
  cv::Vec3d camera_pos(1.0, 0.0, 0.0);
  
  // Apply 90° rotation twice = 180° total
  cv::Mat first_rotation = transformCameraToRobot(camera_pos, rotation_only_rotation_, rotation_only_offset_);
  cv::Vec3d intermediate_pos(first_rotation.at<double>(0), first_rotation.at<double>(1), first_rotation.at<double>(2));
  cv::Mat final_rotation = transformCameraToRobot(intermediate_pos, rotation_only_rotation_, rotation_only_offset_);
  
  // 180° rotation: [1,0,0] → [-1,0,0]
  EXPECT_NEAR(final_rotation.at<double>(0), -1.0, TOLERANCE);
  EXPECT_NEAR(final_rotation.at<double>(1), 0.0, TOLERANCE);
  EXPECT_NEAR(final_rotation.at<double>(2), 0.0, TOLERANCE);
}

// Test 10: Edge case - very small values
TEST_F(CoordinateTransformTest, SmallValuePrecision) {
  cv::Vec3d camera_pos(1e-12, 1e-12, 1e-12);
  
  cv::Mat robot_pos = transformCameraToRobot(camera_pos, identity_rotation_, identity_offset_);
  
  EXPECT_NEAR(robot_pos.at<double>(0), 1e-12, 1e-15);
  EXPECT_NEAR(robot_pos.at<double>(1), 1e-12, 1e-15);
  EXPECT_NEAR(robot_pos.at<double>(2), 1e-12, 1e-15);
}

// Main function to run the tests
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
