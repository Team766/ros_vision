// Example usage of vision_utils rotation utilities

#include "vision_utils/rotation_utils.hpp"
#include <iostream>

int main()
{
    using namespace vision_utils;
    
    // Create a simple rotation about Z-axis
    Matrix3x3 rotation = rotation_z(45.0);
    
    // Test with a vector
    Vector3 test_vector = {{1.0, 0.0, 0.0}};
    Vector3 rotated = matrix_vector_multiply(rotation, test_vector);
    
    std::cout << "Original vector: [" << test_vector[0] << ", " 
              << test_vector[1] << ", " << test_vector[2] << "]\n";
    std::cout << "Rotated 45° about Z: [" << rotated[0] << ", " 
              << rotated[1] << ", " << rotated[2] << "]\n";
    
    // Camera to robot transformation
    Matrix3x3 camera_transform = camera_to_robot();
    Vector3 camera_z = {{0.0, 0.0, 1.0}};
    Vector3 robot_direction = matrix_vector_multiply(camera_transform, camera_z);
    
    std::cout << "Camera Z-axis in robot frame: [" << robot_direction[0] << ", " 
              << robot_direction[1] << ", " << robot_direction[2] << "]\n";
    
    return 0;
}
