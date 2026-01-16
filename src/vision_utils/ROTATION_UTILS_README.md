# Vision Utils Rotation Utilities

This package provides C++ implementations of 3D rotation utilities for robotics applications, specifically designed for camera-to-robot coordinate transformations.

## Features

- 3x3 rotation matrices about X, Y, and Z axes (right-handed coordinate system)
- Composition of rotations in XYZ order (Rx * Ry * Rz)
- Camera-to-robot coordinate transformation
- Matrix and vector operations
- Comprehensive unit tests
- Header-only convenience functions

## Coordinate Systems

### Robot Coordinates (Right-handed)
- X: forward
- Y: to the right  
- Z: up

### Camera Coordinates (OpenCV style)
- Z: out of the camera (optical axis)
- X: to the right (columns)
- Y: down (rows)

## API Reference

### Basic Rotation Functions

```cpp
#include "vision_utils/rotation_utils.hpp"

// Create rotation matrices
Matrix3x3 rx = vision_utils::rotation_x(90.0);  // 90° about X-axis
Matrix3x3 ry = vision_utils::rotation_y(45.0);  // 45° about Y-axis  
Matrix3x3 rz = vision_utils::rotation_z(30.0);  // 30° about Z-axis
```

### Composed Rotations

```cpp
// Compose rotations: Rx(roll) * Ry(pitch) * Rz(yaw)
Matrix3x3 combined = vision_utils::compose_rotations_xyz(10.0, 20.0, 30.0);
```

### Camera-to-Robot Transformation

```cpp
// Get transformation from camera coordinates to robot frame
Matrix3x3 transform = vision_utils::camera_to_robot();

// Transform a camera Z-axis vector to robot frame
Vector3 camera_z = {{0.0, 0.0, 1.0}};
Vector3 robot_direction = vision_utils::matrix_vector_multiply(transform, camera_z);
```

### Matrix Operations

```cpp
// Matrix multiplication
Matrix3x3 result = vision_utils::matrix_multiply(matrix_a, matrix_b);

// Matrix-vector multiplication  
Vector3 result = vision_utils::matrix_vector_multiply(matrix, vector);

// Identity matrix
Matrix3x3 identity = vision_utils::identity_matrix();
```

### Utility Functions

```cpp
// Angle conversions
double radians = vision_utils::deg_to_rad(90.0);
double degrees = vision_utils::rad_to_deg(M_PI/2);
```

## Building

The rotation utilities are part of the `vision_utils` package. To build:

```bash
source /opt/ros/humble/setup.bash
source build_env_vars.sh
colcon build --packages-select vision_utils
```

## Testing

Run the comprehensive unit tests:

```bash
colcon test --packages-select vision_utils
colcon test-result --verbose
```

The test suite includes:
- Basic rotation matrix validation
- Composition correctness  
- Orthogonality and determinant checks
- Comparison with reference Python implementation
- Edge cases and error conditions

## Demo

A demo executable is provided to verify the implementation:

```bash
./build/vision_utils/rotation_demo
```

This outputs the same camera orientation vectors and rotation matrices as the original Python implementation.

## Integration

To use in your own CMake project:

```cmake
find_package(vision_utils REQUIRED)

add_executable(my_app src/main.cpp)
target_link_libraries(my_app vision_utils::vision_utils)
ament_target_dependencies(my_app vision_utils)
```

## Camera Configurations

The library supports the standard camera configurations used in the robotics platform:

- **Left/Right Front Cameras**: Point backwards towards -X, angled up 23°
- **Left/Right Back Cameras**: Point forward, yawed ±30° for wider coverage

Example usage matching the Python implementation:

```cpp
// Left front camera (pointing backwards and up)
Matrix3x3 left_front = matrix_multiply(
    compose_rotations_xyz(0.0, 23.0, 180.0),
    camera_to_robot()
);

// Left back camera (pointing forward, yawed 30°)  
Matrix3x3 left_back = matrix_multiply(
    compose_rotations_xyz(0.0, 0.0, 30.0),
    camera_to_robot()
);
```

## Type Definitions

```cpp
// 3x3 matrix as array of arrays
using Matrix3x3 = std::array<std::array<double, 3>, 3>;

// 3D vector as array
using Vector3 = std::array<double, 3>;
```

All functions use double precision for accuracy and are designed to be efficient for real-time robotics applications.
