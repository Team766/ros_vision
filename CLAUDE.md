# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### Initial Setup and Build
```bash
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh

# First-time build (builds OpenCV, WPILib, and other deps from source)
./bootstrap.sh
```

### Regular Development Build
```bash
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh
source install/setup.bash

# Build all packages
colcon build

# Build specific packages
colcon build --packages-select apriltags_cuda
colcon build --packages-select usb_camera
colcon build --packages-select vision_utils
colcon build --packages-select game_piece_detection
```

### Testing
```bash
# Run all tests (requires GPU)
./run_tests.sh

# Run CPU-only tests (skips gpu_detector_test, suitable for CI)
./run_tests.sh --cpu-only

# Run tests for specific package
./run_tests.sh --packages-select apriltags_cuda

# View test results
colcon test-result --all
```

### Release and Deployment
```bash
# Full release: bump version, build, bundle, git tag
./release.sh patch    # 1.0.0 -> 1.0.1
./release.sh minor    # 1.0.1 -> 1.1.0
./release.sh major    # 1.1.0 -> 2.0.0

# Build and create bundle without version bump
./release.sh bundle

# Create bundle from existing build (fastest)
./release.sh bundle-only

# Version management
./version_manager.sh show
./version_manager.sh bump patch
```

Bundles are `ros_vision_bundle-v{version}.tar.gz` containing the full install directory with dereferenced symlinks for portable deployment.

## Launch
```bash
source install/setup.bash
ros2 launch ros_vision_launch launch_vision.py

# With options
ros2 launch ros_vision_launch launch_vision.py enable_bag_recording:=false
ros2 launch ros_vision_launch launch_vision.py measurement_mode:=true
ros2 launch ros_vision_launch launch_vision.py log_level:=debug

# Optimized launch with CPU pinning and real-time scheduling
./optimized_launch.bsh
```

## High-Level Architecture

ROS2 Humble computer vision system for FRC Team 766. Provides real-time AprilTag detection (CUDA-accelerated) and game piece detection (YOLOv11/TensorRT).

### Pipeline Flow

1. **usb_camera** captures frames, publishes to `cameras/{location}/image_raw`
2. **apriltags_cuda** subscribes, runs full detection on GPU, publishes poses to `apriltags/{location}/pose` and annotated images to `apriltags/{location}/images`
3. **game_piece_detection** runs YOLOv11 via TensorRT for object detection
4. Pose estimates are sent to robot via WPILib **NetworkTables**

### Camera Auto-Discovery

The launch system (`src/ros_vision_launch/launch/launch_vision.py`) scans `/dev/v4l/by-id/` for cameras, extracts serial numbers, maps them to locations via `system_config.json`, and generates camera + detector node pairs dynamically. Topic names use `{location}` template substitution to scale automatically.

### Configuration System

All configuration lives in `src/vision_config_data/data/`:
- **`system_config.json`** - Central config: camera serial-to-location mapping, camera settings (resolution, format, frame rate), extrinsics (rotation matrices + offsets), bag recording topics, CPU pinning config, NetworkTables address, game piece detection settings
- **`calibration/calibrationmatrix_{serial}.json`** - Per-camera intrinsic calibration

Changing config only requires rebuilding `vision_config_data`, not C++ packages.

### Build System Details

- Uses **Clang 17** as both C++ and CUDA compiler (better C++20 support than nvcc)
- `CMAKE_CUDA_ARCHITECTURES` in `build_env_vars.sh` must match your GPU (default: 87 for Jetson Orin)
- External dependencies built from source via `ExternalProject_Add` in `src/external/CMakeLists.txt`: OpenCV 4.9.0 (with CUDA), WPILib, NVIDIA CCCL, nlohmann/json, AprilTag (FRC 971 fork), Seasocks
- Custom OpenCV must build **before** cv_bridge (bootstrap.sh handles ordering)
- All packages link explicitly to custom deps via RPATH to avoid system library conflicts
- Code style: Google C++ (clang-format)

### Packages

- **apriltags_cuda** - CUDA kernels for full GPU-side AprilTag detection pipeline (threshold -> label -> quad detect -> decode -> pose). Custom ROS messages: `TagDetection.msg`, `TagDetectionArray.msg`
- **usb_camera** - Camera capture with OpenCV backend, abstracted camera interface for testability
- **game_piece_detection** - YOLOv11 inference via TensorRT (FP16/INT8). Model pipeline: PyTorch (.pt) -> ONNX -> TensorRT (.engine)
- **vision_utils** - Shared library: `ConfigLoader` (JSON config), `ProcessScheduler` (CPU pinning + RT scheduling), rotation utilities
- **vision_config_data** - JSON config files, calibration data (ament resource package)
- **ros_vision_launch** - Launch files with auto-discovery logic
- **camera_calibration** / **extrinsic_calibration** - Calibration tools (Charuco/Checkerboard)
- **seasocks_viewer** - WebSocket-based image viewer
- **external/vision_deps** - All third-party dependency builds

### CPU Pinning Pattern

Nodes accept `pin_to_core` and `priority` ROS parameters. The launch system assigns sequential cores: camera N gets core N*2, its detector gets core N*2+1. Available cores are configured in `system_config.json` under `performance_optimization`.

### CI/CD

GitHub Actions (`.github/workflows/cmake-single-platform.yml`): Docker-based builds, build caching by source hash, CPU-only tests, artifact upload of bundles. Multi-arch workflow supports amd64 + arm64 via QEMU.
