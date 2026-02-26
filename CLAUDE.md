# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### Initial Setup and Build
```bash
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh

# First-time build (pulls dependencies like OpenCV, WpiLIB)
./bootstrap.sh
```

### Regular Development Build
```bash
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh
source install/setup.bash

colcon build                                      # Build all packages
colcon build --packages-select apriltags_cuda      # Build single package
```

### Build Environment
- Uses clang++-17 as both C++ and CUDA compiler (set in `build_env_vars.sh`)
- `CMAKE_CUDA_ARCHITECTURES` must match target GPU compute capability (e.g., 87 for Jetson Orin)
- External dependencies (OpenCV, WpiLIB, nlohmann/json, Seasocks, CCCL, apriltag) are managed via `src/external/` and built by `bootstrap.sh` into `install/vision_deps/`

### Testing
```bash
colcon test                                       # Run all tests
colcon test --packages-select vision_utils        # Run tests for one package
colcon test-result --all                          # View test results

# Run test executables directly
./build/vision_utils/test_config_loader
./build/vision_utils/test_config_loader_utilities
```

### Versioned Release
```bash
./release.sh patch    # Bump version, build, bundle, and tag
./release.sh bundle   # Build and create bundle without version bump
./version_manager.sh show  # Show current version info
```

Creates portable `ros_vision_bundle-v{version}.tar.gz` with dereferenced symlinks.

## Launch

```bash
./start_vision.bsh                                    # Recommended: auto-sources env
./start_vision.bsh enable_bag_recording:=false         # Disable bag recording
./start_vision.bsh measurement_mode:=true              # Enable timing CSV logging
./start_vision.bsh log_level:=debug                    # Set log level

# Or manually:
source install/setup.bash
ros2 launch ros_vision_launch launch_vision.py
```

For low-latency with CPU pinning and real-time scheduling: `./optimized_launch.bsh`

## Architecture

ROS2 Humble computer vision system for FRC Team 766 providing real-time AprilTag detection with CUDA acceleration and YOLOv11-based game piece detection with TensorRT.

### Data Flow

```
USB Camera → usb_camera_node → cameras/{location}/image_raw topic
                                         ↓
                              apriltags_cuda_node → apriltags/{location}/pose topic
                                                  → apriltags/{location}/images topic
                                         ↓
                              foxglove_bridge (visualization)
                              bag_recorder (data capture)
```

### Camera Auto-Detection
The launch system (`ros_vision_launch/launch/launch_vision.py`) scans `/dev/v4l/by-id/` for camera devices, extracts serial numbers, and maps them to locations using `system_config.json`. Each camera gets a paired `usb_camera_node` + `apriltags_cuda_node` with sequential CPU core assignment when performance optimizations are enabled.

### Configuration: `system_config.json`
Central config at `src/vision_config_data/data/system_config.json` with these sections:
- **camera_mounted_positions**: Maps camera serial → location name + capture settings (resolution, format, frame rate, API preference)
- **extrinsics**: Per-location 3x3 rotation matrix + offset vector for camera-to-robot transforms
- **bag_recording**: Topic patterns (supports `{location}` template expansion), output directory, size/duration limits
- **performance_optimization**: CPU core pinning list and RT scheduling priority
- **network_tables_config**: FRC NetworkTables address/table for robot communication
- **game_piece_detection**: TensorRT engine path, input channels, class names

After editing config, rebuild with: `colcon build --packages-select vision_config_data`

### Key Packages (under `src/`)

| Package | Language | Purpose |
|---------|----------|---------|
| `apriltags_cuda` | CUDA/C++ | GPU-accelerated AprilTag detection with custom CUDA kernels |
| `usb_camera` | C++ | Camera capture node, auto-detection by serial number |
| `vision_utils` | C++ | Shared utilities: ConfigLoader, CPU pinning (`process_scheduler.hpp`), rotation math |
| `vision_config_data` | Data | JSON config files and per-camera calibration matrices |
| `ros_vision_launch` | Python | Launch system with camera scanning and node orchestration |
| `game_piece_detection` | C++/CUDA | YOLOv11 inference via TensorRT for object detection |
| `bag_utils` | Python | Extract images from ROS bags (`extract_images` tool) |
| `extrinsic_calibration` | Python | Camera extrinsic calibration tooling |
| `camera_calibration` | Python | Camera intrinsic calibration |
| `seasocks_viewer` | C++ | Web-based image viewer (alternative to Foxglove) |
| `external` | CMake | Dependency management for vision_deps |

### ROS2 Topic Naming Convention
- Camera images: `cameras/{location}/image_raw`
- AprilTag detection images: `apriltags/{location}/images`
- AprilTag poses: `apriltags/{location}/pose`, `apriltags/{location}/pose_camera`
- Compressed variants: append `/compressed`

### Docker
```bash
docker build -t ros_vision:latest .               # x64 build
./docker_build_arm64.sh                            # ARM64 cross-build for Jetson
```

### Game Piece Detection Model Pipeline
PyTorch (.pt) → ONNX (.onnx) → TensorRT engine (.engine). Engine files are GPU-architecture-specific and must be regenerated per target. See `src/game_piece_detection/README.md` for conversion steps and benchmark tools.
