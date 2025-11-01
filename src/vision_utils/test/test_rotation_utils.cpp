// Copyright 2025 Team766
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include "vision_utils/rotation_utils.hpp"
#include <cmath>

namespace vision_utils
{

class RotationUtilsTest : public ::testing::Test
{
protected:
  // Tolerance for floating point comparisons
  static constexpr double kTolerance = 1e-6;

  // Helper function to compare matrices with tolerance
  void ExpectMatrixNear(const Matrix3x3& expected, const Matrix3x3& actual, double tolerance = kTolerance)
  {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        EXPECT_NEAR(expected(i, j), actual(i, j), tolerance) 
            << "Matrix element [" << i << "][" << j << "] differs";
      }
    }
  }

  // Helper function to compare vectors with tolerance
  void ExpectVectorNear(const Vector3& expected, const Vector3& actual, double tolerance = kTolerance)
  {
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(expected[i], actual[i], tolerance) 
          << "Vector element [" << i << "] differs";
    }
  }
};

TEST_F(RotationUtilsTest, DegToRadConversion)
{
  EXPECT_NEAR(0.0, deg_to_rad(0.0), kTolerance);
  EXPECT_NEAR(M_PI / 2.0, deg_to_rad(90.0), kTolerance);
  EXPECT_NEAR(M_PI, deg_to_rad(180.0), kTolerance);
  EXPECT_NEAR(2.0 * M_PI, deg_to_rad(360.0), kTolerance);
  EXPECT_NEAR(-M_PI / 2.0, deg_to_rad(-90.0), kTolerance);
}

TEST_F(RotationUtilsTest, RadToDegConversion)
{
  EXPECT_NEAR(0.0, rad_to_deg(0.0), kTolerance);
  EXPECT_NEAR(90.0, rad_to_deg(M_PI / 2.0), kTolerance);
  EXPECT_NEAR(180.0, rad_to_deg(M_PI), kTolerance);
  EXPECT_NEAR(360.0, rad_to_deg(2.0 * M_PI), kTolerance);
  EXPECT_NEAR(-90.0, rad_to_deg(-M_PI / 2.0), kTolerance);
}

TEST_F(RotationUtilsTest, IdentityMatrix)
{
  Matrix3x3 identity = identity_matrix();
  Matrix3x3 expected = Matrix3x3::eye();
  
  ExpectMatrixNear(expected, identity);
}

TEST_F(RotationUtilsTest, RotationXZeroDegrees)
{
  Matrix3x3 rotation = rotation_x(0.0);
  Matrix3x3 expected = identity_matrix();
  
  ExpectMatrixNear(expected, rotation);
}

TEST_F(RotationUtilsTest, RotationX90Degrees)
{
  Matrix3x3 rotation = rotation_x(90.0);
  Matrix3x3 expected(1.0, 0.0, 0.0,
                     0.0, 0.0, -1.0,
                     0.0, 1.0, 0.0);
  
  ExpectMatrixNear(expected, rotation);
}

TEST_F(RotationUtilsTest, RotationYZeroDegrees)
{
  Matrix3x3 rotation = rotation_y(0.0);
  Matrix3x3 expected = identity_matrix();
  
  ExpectMatrixNear(expected, rotation);
}

TEST_F(RotationUtilsTest, RotationY90Degrees)
{
  Matrix3x3 rotation = rotation_y(90.0);
  Matrix3x3 expected(0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0,
                     -1.0, 0.0, 0.0);
  
  ExpectMatrixNear(expected, rotation);
}

TEST_F(RotationUtilsTest, RotationZZeroDegrees)
{
  Matrix3x3 rotation = rotation_z(0.0);
  Matrix3x3 expected = identity_matrix();
  
  ExpectMatrixNear(expected, rotation);
}

TEST_F(RotationUtilsTest, RotationZ90Degrees)
{
  Matrix3x3 rotation = rotation_z(90.0);
  Matrix3x3 expected(0.0, -1.0, 0.0,
                     1.0, 0.0, 0.0,
                     0.0, 0.0, 1.0);
  
  ExpectMatrixNear(expected, rotation);
}

TEST_F(RotationUtilsTest, MatrixMultiplyIdentity)
{
  Matrix3x3 identity = identity_matrix();
  Matrix3x3 test_matrix = rotation_x(45.0);
  
  Matrix3x3 result1 = identity * test_matrix;
  Matrix3x3 result2 = test_matrix * identity;
  
  ExpectMatrixNear(test_matrix, result1);
  ExpectMatrixNear(test_matrix, result2);
}

TEST_F(RotationUtilsTest, MatrixVectorMultiplyIdentity)
{
  Matrix3x3 identity = identity_matrix();
  Vector3 test_vector(1.0, 2.0, 3.0);
  
  Vector3 result = identity * test_vector;
  
  ExpectVectorNear(test_vector, result);
}

TEST_F(RotationUtilsTest, MatrixVectorMultiplyRotationX90)
{
  Matrix3x3 rotation = rotation_x(90.0);
  Vector3 input(1.0, 0.0, 1.0);
  Vector3 expected(1.0, -1.0, 0.0);
  
  Vector3 result = rotation * input;
  
  ExpectVectorNear(expected, result);
}

TEST_F(RotationUtilsTest, ComposeRotationsAllZero)
{
  Matrix3x3 composed = compose_rotations_xyz(0.0, 0.0, 0.0);
  Matrix3x3 expected = identity_matrix();
  
  ExpectMatrixNear(expected, composed);
}

TEST_F(RotationUtilsTest, ComposeRotationsXOnly)
{
  Matrix3x3 composed = compose_rotations_xyz(90.0, 0.0, 0.0);
  Matrix3x3 expected = rotation_x(90.0);
  
  ExpectMatrixNear(expected, composed);
}

TEST_F(RotationUtilsTest, ComposeRotationsYOnly)
{
  Matrix3x3 composed = compose_rotations_xyz(0.0, 90.0, 0.0);
  Matrix3x3 expected = rotation_y(90.0);
  
  ExpectMatrixNear(expected, composed);
}

TEST_F(RotationUtilsTest, ComposeRotationsZOnly)
{
  Matrix3x3 composed = compose_rotations_xyz(0.0, 0.0, 90.0);
  Matrix3x3 expected = rotation_z(90.0);
  
  ExpectMatrixNear(expected, composed);
}

TEST_F(RotationUtilsTest, CameraToRobotTransformation)
{
  Matrix3x3 camera_to_robot_matrix = camera_to_robot();
  
  // Test that this is equivalent to compose_rotations_xyz(-90, 90, 0)
  Matrix3x3 expected = compose_rotations_xyz(-90.0, 90.0, 0.0);
  
  ExpectMatrixNear(expected, camera_to_robot_matrix);
}

TEST_F(RotationUtilsTest, CameraToRobotWithCameraZAxis)
{
  Matrix3x3 camera_to_robot_matrix = camera_to_robot();
  
  // Camera Z axis (0, 0, 1) should transform to some direction in robot frame
  Vector3 camera_z(0.0, 0.0, 1.0);
  Vector3 robot_direction = camera_to_robot_matrix * camera_z;
  
  // The result should be a unit vector (or close to it)
  double magnitude = std::sqrt(robot_direction[0] * robot_direction[0] + 
                              robot_direction[1] * robot_direction[1] + 
                              robot_direction[2] * robot_direction[2]);
  EXPECT_NEAR(1.0, magnitude, kTolerance);
}

TEST_F(RotationUtilsTest, RotationOrthogonality)
{
  // Test that rotation matrices are orthogonal (R * R^T = I)
  Matrix3x3 rotation = rotation_x(45.0);
  
  // Use OpenCV's transpose method
  Matrix3x3 rotation_transpose = rotation.t();
  
  // Multiply R * R^T
  Matrix3x3 result = rotation * rotation_transpose;
  Matrix3x3 identity = identity_matrix();
  
  ExpectMatrixNear(identity, result);
}

TEST_F(RotationUtilsTest, RotationDeterminant)
{
  // Test that rotation matrices have determinant = 1
  Matrix3x3 rotation = rotation_z(30.0);
  
  // Use OpenCV's determinant method
  double det = cv::determinant(rotation);
  
  EXPECT_NEAR(1.0, det, kTolerance);
}

TEST_F(RotationUtilsTest, NegativeAngles)
{
  // Test that negative angles work correctly
  Matrix3x3 positive_rotation = rotation_z(45.0);
  Matrix3x3 negative_rotation = rotation_z(-45.0);
  
  // R(-θ) should be the transpose of R(θ) for rotation matrices
  Matrix3x3 positive_transpose = positive_rotation.t();
  
  ExpectMatrixNear(positive_transpose, negative_rotation);
}

TEST_F(RotationUtilsTest, MatchPythonImplementationSanityCheck)
{
  // Test specific cases that match the Python implementation
  
  // LEFT FRONT CAMERA: roll=0, pitch=23, yaw=180
  Matrix3x3 left_front_rotation = compose_rotations_xyz(0.0, 23.0, 180.0);
  Matrix3x3 full_left_front = left_front_rotation * camera_to_robot();
  
  Vector3 cam_vec_z(0.0, 0.0, 1.0);
  Vector3 result_left_front = full_left_front * cam_vec_z;
  
  // Should point roughly towards -X and up (negative X, positive Z components)
  EXPECT_LT(result_left_front[0], 0.0);  // X should be negative
  EXPECT_GT(result_left_front[2], 0.0);  // Z should be positive
  
  // LEFT BACK CAMERA: roll=0, pitch=0, yaw=30
  Matrix3x3 left_back_rotation = compose_rotations_xyz(0.0, 0.0, 30.0);
  Matrix3x3 full_left_back = left_back_rotation * camera_to_robot();
  
  Vector3 result_left_back = full_left_back * cam_vec_z;
  
  // Should point roughly forward and slightly to the left
  EXPECT_GT(result_left_back[0], 0.0);  // X should be positive (forward)
}

}  // namespace vision_utils
