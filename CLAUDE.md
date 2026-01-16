# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### Initial Setup and Build
```bash
# Source ROS2 and set build environment
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh

# First-time build (pulls dependencies like OpenCV, WpiLIB)
./bootstrap.sh
```

### Regular Development Build
```bash
# Source environment
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh
source install/setup.bash

# Build all packages
colcon build

# Build specific packages
colcon build --packages-select apriltags_cuda
colcon build --packages-select usb_camera
colcon build --packages-select vision_utils
```

### Testing
```bash
# Run tests for all packages
colcon test

# Run tests for specific package
colcon test --packages-select apriltags_cuda

# View test results
colcon test-result --all
```

### Versioned Release and Bundle Creation

The workspace uses semantic versioning for deployment bundles while keeping individual package versions independent.

#### Version Management
```bash
# Show current workspace version and package versions
./version_manager.sh show

# Manually set workspace version
./version_manager.sh set 1.0.0

# Bump workspace version
./version_manager.sh bump patch  # 1.0.0 -> 1.0.1
./version_manager.sh bump minor  # 1.0.1 -> 1.1.0
./version_manager.sh bump major  # 1.1.0 -> 2.0.0
```

#### Release Process
```bash
# Complete release workflow (bump version, build, bundle, tag)
./release.sh patch    # Patch release: 1.0.0 -> 1.0.1
./release.sh minor    # Minor release: 1.0.1 -> 1.1.0  
./release.sh major    # Major release: 1.1.0 -> 2.0.0

# Build workspace and create versioned bundle (no version bump)
./release.sh bundle

# Create versioned bundle without building (fastest)
./release.sh bundle-only

# Show version information
./release.sh version
```

#### Bundle Details
- Creates `ros_vision_bundle-v{version}.tar.gz` (e.g., `ros_vision_bundle-v1.0.1.tar.gz`)
- Contains complete install directory with all dependencies
- Symlinks are dereferenced (copied as actual files) for portability
- Includes all libraries, binaries, and configuration files needed for deployment
- Workspace versioning is independent of individual package.xml versions

## Launch Commands

### Standard Launch
```bash
source install/setup.bash
ros2 launch ros_vision_launch launch_vision.py
```

### Optimized Launch (Low Latency)
```bash
# Uses CPU pinning and real-time scheduling
./optimized_launch.bsh
```

### Launch with Options
```bash
# Disable bag recording
ros2 launch ros_vision_launch launch_vision.py enable_bag_recording:=false

# Enable measurement mode for timing analysis
ros2 launch ros_vision_launch launch_vision.py measurement_mode:=true

# Set log level
ros2 launch ros_vision_launch launch_vision.py log_level:=debug
```

## High-Level Architecture

This is a ROS2-based computer vision system designed for robotics applications, specifically for Team 766. The system provides real-time AprilTag detection using CUDA acceleration.

### Core Components

#### 1. Camera Pipeline
- **usb_camera**: USB camera capture node that auto-detects cameras by serial number
- **apriltags_cuda**: GPU-accelerated AprilTag detection and pose estimation
- **seasocks_viewer**: Web-based image viewer as alternative to Foxglove

#### 2. Configuration System
- **vision_config_data**: Centralized configuration for cameras, calibration, and system settings
- **system_config.json**: Main configuration file mapping camera serials to locations
- **calibration files**: Per-camera calibration matrices in JSON format

#### 3. Performance Optimization
- **vision_utils**: Utilities for CPU pinning and real-time scheduling
- **process_scheduler.hpp**: Core scheduling and CPU affinity management
- Supports binding processes to specific CPU cores with real-time priorities

#### 4. Data Transport
- **image_transport_plugins**: Compressed image transport for bandwidth efficiency
- **foxglove_bridge**: Integration with Foxglove Studio for visualization
- **ROS bag recording**: Built-in data recording with configurable topics

### Package Structure

- `apriltags_cuda/`: CUDA-based AprilTag detection with custom CUDA kernels
- `usb_camera/`: Camera capture with OpenCV backend
- `vision_utils/`: Shared utilities (config loading, scheduling, rotation math)
- `vision_config_data/`: Configuration files and calibration data
- `ros_vision_launch/`: Main launch system with camera auto-detection
- `seasocks_viewer/`: Alternative web viewer using Seasocks library
- `external/`: External dependency management (vision_deps)

### Key Features

#### Camera Auto-Detection
The system automatically scans `/dev/v4l/by-id/` for camera devices, extracts serial numbers, and maps them to configured locations. Supports multiple camera patterns including Arducam devices.

#### Performance Optimization
- CPU core pinning for camera and AprilTags processes
- Real-time FIFO scheduling with configurable priorities  
- Jetson-specific optimizations with `jetson_clocks`
- Sequential core assignment for multi-camera setups

#### Configuration-Driven
All camera settings (resolution, format, frame rate) are configured via JSON rather than hardcoded. The system supports camera location mapping, extrinsics, and bag recording configuration.

#### CUDA Integration
AprilTag detection runs entirely on GPU using custom CUDA kernels for maximum performance. The system requires NVIDIA GPU with appropriate compute capability.

### WSL Support

The system includes comprehensive WSL support via usbipd-win for USB camera forwarding from Windows to WSL2. See WSL_README.md for setup details.

### Timing Analysis

Built-in measurement mode captures detailed timing metrics for performance analysis:
- CSV logging of detection timing
- Python utilities for generating timing reports and plots
- Comprehensive statistics including CDF analysis