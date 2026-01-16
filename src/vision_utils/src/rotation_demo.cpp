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
#include <iostream>
#include <iomanip>

namespace vision_utils
{

void print_matrix(const Matrix3x3& matrix, const std::string& name)
{
  std::cout << name << ":\n";
  for (int i = 0; i < 3; ++i) {
    std::cout << "[";
    for (int j = 0; j < 3; ++j) {
      std::cout << std::setw(8) << std::fixed << std::setprecision(4) << matrix(i, j);
      if (j < 2) std::cout << ", ";
    }
    std::cout << "]\n";
  }
  std::cout << "\n";
}

void print_vector(const Vector3& vector, const std::string& name)
{
  std::cout << name << ": [";
  for (int i = 0; i < 3; ++i) {
    std::cout << std::setw(8) << std::fixed << std::setprecision(4) << vector[i];
    if (i < 2) std::cout << ", ";
  }
  std::cout << "]\n";
}

void demonstrate_camera_rotations()
{
  std::cout << "=== Robot Rotation Utilities Demo ===\n\n";
  
  std::cout << "Robot coordinates (right-handed):\n";
  std::cout << "  X forward, Y to the right, Z up.\n\n";
  
  std::cout << "Camera coordinates (OpenCV style):\n";
  std::cout << "  Z out of the camera (optical axis),\n";
  std::cout << "  X to the right (columns),\n";
  std::cout << "  Y down (rows).\n\n";
  
  Vector3 cam_vec_z(0.0, 0.0, 1.0);
  
  // LEFT FRONT CAMERA
  std::cout << "LEFT FRONT CAMERA:\n";
  std::cout << "Camera is pointed backwards towards -X and up slightly\n";
  double LF_roll = 0.0, LF_pitch = 23.0, LF_yaw = 180.0;
  
  Matrix3x3 R_left_front = compose_rotations_xyz(LF_roll, LF_pitch, LF_yaw) * camera_to_robot();
  
  Vector3 left_front_direction = R_left_front * cam_vec_z;
  print_vector(left_front_direction, "left front sanity check");
  std::cout << "\n";
  
  // RIGHT FRONT CAMERA
  std::cout << "RIGHT FRONT CAMERA:\n";
  std::cout << "Same as left front camera\n";
  double RF_roll = 0.0, RF_pitch = 23.0, RF_yaw = 180.0;
  
  Matrix3x3 R_right_front = compose_rotations_xyz(RF_roll, RF_pitch, RF_yaw) * camera_to_robot();
  
  Vector3 right_front_direction = R_right_front * cam_vec_z;
  print_vector(right_front_direction, "right front sanity check");
  std::cout << "\n";
  
  // LEFT BACK CAMERA
  std::cout << "LEFT BACK CAMERA:\n";
  std::cout << "Camera points forward but yaws 30 degrees\n";
  double LB_roll = 0.0, LB_pitch = 0.0, LB_yaw = 30.0;
  
  Matrix3x3 R_left_back = compose_rotations_xyz(LB_roll, LB_pitch, LB_yaw) * camera_to_robot();
  
  Vector3 left_back_direction = R_left_back * cam_vec_z;
  print_vector(left_back_direction, "left back sanity check");
  std::cout << "\n";
  
  // RIGHT BACK CAMERA
  std::cout << "RIGHT BACK CAMERA:\n";
  std::cout << "Same as left back camera but yaws -30 degrees\n";
  double RB_roll = 0.0, RB_pitch = 0.0, RB_yaw = -30.0;
  
  Matrix3x3 R_right_back = compose_rotations_xyz(RB_roll, RB_pitch, RB_yaw) * camera_to_robot();
  
  Vector3 right_back_direction = R_right_back * cam_vec_z;
  print_vector(right_back_direction, "right back sanity check");
  std::cout << "\n";
  
  // Print the rotation matrices
  std::cout << "=== Rotation Matrices ===\n";
  print_matrix(R_left_front, "Left Front");
  print_matrix(R_right_front, "Right Front");
  print_matrix(R_left_back, "Left Back");
  print_matrix(R_right_back, "Right Back");
}

}  // namespace vision_utils

int main()
{
  vision_utils::demonstrate_camera_rotations();
  return 0;
}
