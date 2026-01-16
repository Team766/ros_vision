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

#ifndef VISION_UTILS__ROTATION_UTILS_HPP_
#define VISION_UTILS__ROTATION_UTILS_HPP_

#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace vision_utils
{

/**
 * @brief 3x3 rotation matrix using OpenCV's fixed-size matrix
 */
using Matrix3x3 = cv::Matx33d;

/**
 * @brief 3D vector using OpenCV's fixed-size vector
 */
using Vector3 = cv::Vec3d;

/**
 * @brief Create a 3x3 rotation matrix about the X-axis (right-handed).
 * 
 * @param angle_degrees Rotation angle in degrees
 * @return Matrix3x3 The rotation matrix
 */
Matrix3x3 rotation_x(double angle_degrees);

/**
 * @brief Create a 3x3 rotation matrix about the Y-axis (right-handed).
 * 
 * @param angle_degrees Rotation angle in degrees
 * @return Matrix3x3 The rotation matrix
 */
Matrix3x3 rotation_y(double angle_degrees);

/**
 * @brief Create a 3x3 rotation matrix about the Z-axis (right-handed).
 * 
 * @param angle_degrees Rotation angle in degrees
 * @return Matrix3x3 The rotation matrix
 */
Matrix3x3 rotation_z(double angle_degrees);

/**
 * @brief Compose rotations in the order Rx(roll) * Ry(pitch) * Rz(yaw).
 * 
 * @param roll_deg Roll angle in degrees (rotation about X-axis)
 * @param pitch_deg Pitch angle in degrees (rotation about Y-axis)
 * @param yaw_deg Yaw angle in degrees (rotation about Z-axis)
 * @return Matrix3x3 The composed rotation matrix
 */
Matrix3x3 compose_rotations_xyz(double roll_deg, double pitch_deg, double yaw_deg);

/**
 * @brief Get the transformation matrix from camera coordinates to robot frame.
 * 
 * Robot coordinates (right-handed):
 *   X forward, Y to the right, Z up.
 * 
 * Camera coordinates (OpenCV style):
 *   Z out of the camera (optical axis),
 *   X to the right (columns),
 *   Y down (rows).
 * 
 * @return Matrix3x3 The camera-to-robot transformation matrix
 */
Matrix3x3 camera_to_robot();

/**
 * @brief Multiply two 3x3 matrices using OpenCV's built-in operator.
 * 
 * @param a First matrix
 * @param b Second matrix
 * @return Matrix3x3 Result of a * b
 */
Matrix3x3 matrix_multiply(const Matrix3x3& a, const Matrix3x3& b);

/**
 * @brief Multiply a 3x3 matrix by a 3D vector using OpenCV's built-in operator.
 * 
 * @param matrix The 3x3 matrix
 * @param vector The 3D vector
 * @return Vector3 Result of matrix * vector
 */
Vector3 matrix_vector_multiply(const Matrix3x3& matrix, const Vector3& vector);

/**
 * @brief Create an identity 3x3 matrix using OpenCV.
 * 
 * @return Matrix3x3 The identity matrix
 */
Matrix3x3 identity_matrix();

/**
 * @brief Convert degrees to radians.
 * 
 * @param degrees Angle in degrees
 * @return double Angle in radians
 */
constexpr double deg_to_rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

/**
 * @brief Convert radians to degrees.
 * 
 * @param radians Angle in radians
 * @return double Angle in degrees
 */
constexpr double rad_to_deg(double radians)
{
  return radians * 180.0 / M_PI;
}

}  // namespace vision_utils

#endif  // VISION_UTILS__ROTATION_UTILS_HPP_
