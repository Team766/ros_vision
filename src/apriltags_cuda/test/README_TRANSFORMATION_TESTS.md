# AprilTags CUDA Coordinate Transformation Tests

## Overview

These tests ensure that the coordinate transformation from camera frame to robot frame is implemented correctly and remains robust against future changes.

## Problem Statement

The transformation implemented in the `ApriltagsDetector` class:
```cpp
cv::Mat aprilTagInCameraFrameAsMat = cv::Mat(aprilTagInCameraFrame);
cv::Mat aprilTagInRobotFrame = extrinsic_rotation_ * aprilTagInCameraFrameAsMat + extrinsic_offset_;
```

This is a critical operation that converts AprilTag positions from the camera's coordinate system to the robot's coordinate system using:
- **Rotation Matrix** (`extrinsic_rotation_`): 3x3 matrix describing camera orientation relative to robot
- **Translation Vector** (`extrinsic_offset_`): 3x1 vector describing camera position relative to robot

## Test Strategy

### Integration Tests (`apriltag_detector_transform_test.cu`)

**Purpose**: Test the transformation within the actual `ApriltagsDetector` class using the production code path

**Test Cases**:
- **Identity Transform**: No rotation, no translation - coordinates should be preserved
- **Rotation Only**: Pure rotations (90°, 45°) around different axes  
- **Translation Only**: Pure translations with various offset vectors
- **Complex Transforms**: Combined rotation and translation with realistic camera mounting scenarios
- **Realistic Scenarios**: Camera mounted at typical angles (e.g., 30° downward tilt)
- **Multiple Tags**: Simulate multiple AprilTags at different positions
- **Edge Cases**: Zero inputs, very small/large values, distance preservation
- **State Management**: Verify extrinsic parameters are set/retrieved correctly

**Benefits**:
- Tests actual production code path using `transformCameraToRobot()` method
- Validates class member interactions and state management
- Ensures no side effects from other class methods
- Fast execution with comprehensive mathematical validation
- Uses `TestableApriltagsDetector` class that bypasses GPU initialization for testing

### Existing GPU Tests (Enhanced)

**Integration Point**: These transformation tests complement the existing GPU detection tests by ensuring that detected tags are correctly transformed to robot coordinates.

## Test Implementation Details

### TestableApriltagsDetector Class
The tests use a `TestableApriltagsDetector` class that inherits from the actual `ApriltagsDetector` class:

```cpp
class TestableApriltagsDetector : public ApriltagsDetector {
public:
    TestableApriltagsDetector() : ApriltagsDetector(true) {} // bypass_init = true
    
    cv::Mat testTransformCameraToRobot(const cv::Vec3d& camera_pos) {
        return transformCameraToRobot(camera_pos);
    }
};
```

**Key Features**:
- **Bypass Constructor**: Uses `bypass_init=true` to skip GPU initialization and ROS setup
- **Protected Method Access**: Exposes the `transformCameraToRobot()` method for direct testing
- **Real Implementation**: Tests the actual production transformation code
- **No Mocking**: Uses the real class with real member variables and methods

### Test Categories Explained

### Mathematical Validation Tests
- **Rotation Matrices**: Verify rotations preserve distances and follow rotation matrix properties
- **Translation Vectors**: Verify translations are applied correctly
- **Composition**: Verify multiple transformations compose correctly
- **Inverse Operations**: Verify transformations can be reversed

### Realistic Scenario Tests
- **Typical Camera Mounts**: 
  - Forward-facing with downward tilt (common for robot vision)
  - Side-mounted cameras
  - Overhead cameras
- **Distance Ranges**: From 10cm to 100m (covering typical AprilTag detection ranges)
- **Multiple Tag Scenarios**: Testing with several tags at different positions

### Edge Case Tests
- **Numerical Precision**: Very small values (1e-12) to test floating-point precision
- **Large Values**: Very large distances to test numerical stability
- **Zero Cases**: Tags at origin, zero rotations, zero translations

## Usage

### Running Tests
```bash
# Build with tests enabled (tests build by default now)
colcon build --packages-select apriltags_cuda
source install/setup.bash

# Run the transformation tests
./build/apriltags_cuda/apriltag_detector_transform_test

# Run all apriltags tests
./build/apriltags_cuda/gpu_detector_test
./build/apriltags_cuda/apriltag_detector_transform_test
```

### Adding New Test Cases

To add new test scenarios to `apriltag_detector_transform_test.cu`:

Example new test case:
```cpp
TEST_F(ApriltagDetectorTransformTest, MyNewScenario) {
    TestableApriltagsDetector detector;
    
    // Set up transformation parameters
    cv::Mat rotation = ...; // Your rotation matrix
    cv::Mat offset = ...;   // Your offset vector
    detector.setExtrinsicRotation(rotation);
    detector.setExtrinsicOffset(offset);
    
    // Test the transformation
    cv::Vec3d input(x, y, z);
    cv::Mat result = detector.testTransformCameraToRobot(input);
    
    EXPECT_NEAR(result.at<double>(0), expected_x, TOLERANCE);
    EXPECT_NEAR(result.at<double>(1), expected_y, TOLERANCE);
    EXPECT_NEAR(result.at<double>(2), expected_z, TOLERANCE);
}
```

## What These Tests Catch

### Common Bugs These Tests Will Detect:
1. **Matrix Multiplication Order**: `offset + rotation * point` vs `rotation * point + offset`
2. **Incorrect Matrix Dimensions**: 3x1 vs 1x3 vectors
3. **Wrong Rotation Direction**: Transpose vs original matrix
4. **Units Mixing**: Meters vs millimeters, degrees vs radians
5. **Coordinate System Confusion**: Right-hand vs left-hand coordinate systems
6. **Memory Issues**: Using uninitialized or corrupted extrinsic parameters
7. **Class State Issues**: Incorrect getter/setter behavior for extrinsic parameters

### Regression Protection:
- Any change that breaks the transformation will cause immediate test failures
- Changes to extrinsic parameter loading will be validated
- Modifications to the transformation math will be caught
- Refactoring of the `ApriltagsDetector` class is protected against regressions

## Integration with CI/CD

These tests should be run:
- **Before every commit** (pre-commit hooks)
- **On every pull request** (CI pipeline)  
- **Before releases** (release validation)

The tests are designed to be fast and reliable, making them suitable for frequent execution.

**Build Configuration**: Tests are now enabled by default and build automatically with `colcon build` - no need for `-DBUILD_TESTING=ON`.

## Maintenance

### When to Update Tests:
- **New camera mounting configurations**: Add realistic scenario tests
- **Changes to coordinate systems**: Update expected values
- **Performance optimizations**: Ensure accuracy is maintained
- **New transformation features**: Add corresponding test coverage
- **Class refactoring**: Update test setup if `ApriltagsDetector` interface changes

### Test Tolerance Values:
- **TOLERANCE (1e-6)**: For real-world scenarios with potential floating-point accumulation
- Adjust these values if legitimate precision changes occur, but investigate any significant changes carefully.

## Current Test Suite (9 Tests)

1. **DetectorIdentityTransform**: Identity transformation preserves coordinates
2. **DetectorRotationTransform**: 90° Z-axis rotation validation
3. **DetectorTranslationTransform**: Pure translation validation  
4. **DetectorRealisticTransform**: Combined rotation + translation (realistic camera mount)
5. **DetectorMultipleTagPositions**: Multiple tags at different positions
6. **DetectorTransformationConsistency**: Same input produces same output
7. **DetectorExtrinsicSettersGetters**: Validate extrinsic parameter management
8. **DetectorLargeDistanceTransform**: Large distance numerical stability
9. **DetectorSmallDistanceTransform**: Small distance precision validation

All tests use the actual `ApriltagsDetector` class and validate the production `transformCameraToRobot()` method.
