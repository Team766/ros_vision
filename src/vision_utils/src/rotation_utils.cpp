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

#include "vision_utils/rotation_utils.hpp"

namespace vision_utils
{

Matrix3x3 rotation_x(double angle_degrees)
{
  const double theta = deg_to_rad(angle_degrees);
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  
  return Matrix3x3(1.0, 0.0, 0.0,
                   0.0, c,   -s,
                   0.0, s,   c);
}

Matrix3x3 rotation_y(double angle_degrees)
{
  const double theta = deg_to_rad(angle_degrees);
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  
  return Matrix3x3(c,   0.0, s,
                   0.0, 1.0, 0.0,
                   -s,  0.0, c);
}

Matrix3x3 rotation_z(double angle_degrees)
{
  const double theta = deg_to_rad(angle_degrees);
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  
  return Matrix3x3(c,   -s,  0.0,
                   s,   c,   0.0,
                   0.0, 0.0, 1.0);
}

Matrix3x3 compose_rotations_xyz(double roll_deg, double pitch_deg, double yaw_deg)
{
  const Matrix3x3 Rx = rotation_x(roll_deg);
  const Matrix3x3 Ry = rotation_y(pitch_deg);
  const Matrix3x3 Rz = rotation_z(yaw_deg);
  
  // Use OpenCV's built-in matrix multiplication operator
  return Rx * Ry * Rz;
}

Matrix3x3 camera_to_robot()
{
  // Converts camera coords to robot frame
  // This matches the Python implementation: compose_rotations_xyz(-90, 90, 0)
  return compose_rotations_xyz(-90.0, 90.0, 0.0);
}

Matrix3x3 matrix_multiply(const Matrix3x3& a, const Matrix3x3& b)
{
  // Use OpenCV's built-in matrix multiplication operator
  return a * b;
}

Vector3 matrix_vector_multiply(const Matrix3x3& matrix, const Vector3& vector)
{
  // Use OpenCV's built-in matrix-vector multiplication operator
  return matrix * vector;
}

Matrix3x3 identity_matrix()
{
  return Matrix3x3::eye();
}

}  // namespace vision_utils
