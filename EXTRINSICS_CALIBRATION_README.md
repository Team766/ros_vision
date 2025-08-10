# Extrinsic Camera Calibration

This document describes how to use the extrinsic calibration system to collect synchronized camera data for calibrating the relative positions and orientations of multiple cameras.

The purpose of this procedure is to locate the position of each camera as exactly as possible on the robot.  This includes solving for the position (X, Y, Z) and the rotation (rool, pitch, yaw) of each camera relative to robot center.

As such, your cameras should be mounted in their desired location on the robot.  If you change these locations you will need to rerun the extrinsics calibration procedure.

## Overview

The extrinsic cal procedure consists of two parts:
- Collecting syncronized frames from the cameras with fiducial targets visible in them (data collection)
- Running a solver to solve for the position and rotation of each camera by identifying the fiducial targets in the images and solving for the position/rotation which minimizes the location error of each target

The extrinsic calibration system automatically:
- Discovers all connected USB cameras
- Loads camera configurations from `system_config.json`
- Captures synchronized frames from all cameras
- Publishes live image streams for visualization
- Saves timestamped calibration data to disk

## Prerequisites

**IMPORTANT**: Cameras must be intrinsically calibrated before running the extrinsic solver. The solver relies on accurate intrinsic parameters to correctly interpret AprilTag poses. See [CALIBRATION_README.md](CALIBRATION_README.md) for intrinsic calibration procedures.

**Camera Placement Requirement**: This extrinsic calibration method requires **overlapping fields of view** between cameras. The solver works by detecting the same AprilTags in multiple cameras simultaneously and minimizing the difference in their computed 3D positions. If cameras have completely non-overlapping fields of view, this method will not work as no common AprilTags will be visible to establish the relative camera poses.

# Quick Start

## Run The Data Collection Utility

### 1. Connect Cameras
Ensure all cameras you want to calibrate are connected via USB and visible in `/dev/v4l/by-id/`.

### 2. Launch the System
```bash
source install/setup.bash
ros2 launch ros_vision_launch extrinsic_calibration.launch.py
```

### 3. Visualize in Foxglove Studio
- Open Foxglove Studio
- Connect to WebSocket: `ws://localhost:8765`
- Add Image panels for each camera topic: `/cameras/{location}/image_raw`

### 4. Collect Calibration Data

The data collection process requires an AprilTag calibration target visible to multiple cameras:

#### Setup:
- Print an AprilTag grid (recommended: 6x8 grid of tag36h11 family tags)
- Ensure adequate lighting for clear tag detection
- Position cameras so their fields of view overlap where you'll hold the target

#### Data Collection Procedure:
1. **Hold the AprilTag grid** in the overlapping field of view of both cameras
2. **Move the target through various positions and orientations**:
   - Different distances from cameras (near, far, medium)
   - Various angles and rotations
   - Different positions across the shared field of view
   - Ensure the target is clearly visible in both cameras simultaneously
3. **Hold each position steady** for 2-3 seconds to ensure clear capture
4. **Collect 30-50 synchronized frame pairs** for robust calibration

#### Monitoring:
- Watch the Foxglove image feeds to ensure AprilTags are visible in each image
- Ensure both cameras can see the target clearly in each position
- Avoid motion blur by holding positions steady during capture

The system will automatically:
- Capture frames at 2.0 Hz for 60 seconds (configurable)
- Save images to `./calibration_data/calibration_data_YYYY_MM_DD_HH_MM_SS/`
- Generate a `framesets.json` file with metadata

## Run The Solver

### 5. Solve for Extrinsics (Desktop Machine)
Transfer the calibration data to a desktop computer and run the solver.

#### Solver Configuration File
Create a `solver_config.yaml` file with your camera parameters:

```yaml
frameset_dir: "/path/to/calibration_data_YYYY_MM_DD_HH_MM_SS/"
num_iterations: 1000
learning_rate: 0.01
cameras:
  cam14:
    intrinsics_filename: "calibrationmatrix_cam14.json"
    initial_rotations_degrees: [0.0, 23.0, 180.0]
    initial_translation_meters: [0.033, -0.327, 0.270]
    rotation_params_adjustable: True
    translation_params_adjustable: True
  cam13:
    intrinsics_filename: "calibrationmatrix_cam13.json"
    initial_rotations_degrees: [0.0, 23.0, 180.0]
    initial_translation_meters: [0.087, 0.318, 0.392]
    rotation_params_adjustable: True
    translation_params_adjustable: True
```

**Configuration Parameters:**
- `frameset_dir`: Path to directory containing calibration images
- `num_iterations`: Number of optimization iterations (default: 1000)
- `learning_rate`: Adam optimizer learning rate (default: 0.01)
- `cameras`: Dictionary of camera configurations
  - `intrinsics_filename`: Path to camera intrinsic calibration file
  - `initial_rotations_degrees`: Initial guess for camera rotation [roll, pitch, yaw]
  - `initial_translation_meters`: Initial guess for camera position [x, y, z]
  - `rotation_params_adjustable`: Whether to optimize rotation parameters
  - `translation_params_adjustable`: Whether to optimize translation parameters

#### Run The Solver

```bash
# Copy the entire calibration data directory to your desktop
# On desktop: Create virtual environment and install dependencies
cd /path/to/ros_vision/src/extrinsic_calibration/
python3 -m venv solver_env
source solver_env/bin/activate
pip install -r requirements.txt

# Create solver configuration file (solver_config.yaml)
# Run the solver
python3 extrinsic_calibration/solver.py solver_config.yaml
```



## Configuration

### Camera Configuration
Camera settings are loaded from `vision_config_data/system_config.json`:

```json
{
  "camera_mounted_positions": {
    "766": {
      "location": "center_front",
      "format": "MJPG",
      "height": 800,
      "width": 1280,
      "frame_rate": 100,
      "api_preference": "V4L2"
    }
  }
}
```

### Launch Parameters
Customize data collection by modifying the launch file or passing parameters:

```bash
ros2 launch ros_vision_launch extrinsic_calibration.launch.py \
  capture_length_seconds:=120.0 \
  frame_rate_hz:=1.0 \
  output_dir:="/path/to/calibration/data"
```

Available parameters:
- `capture_length_seconds`: How long to collect data (default: 60.0)
- `frame_rate_hz`: Frame capture rate (default: 0.5)
- `output_dir`: Where to save calibration data (default: "./calibration_data")

## Camera Discovery

The system uses `/dev/v4l/by-id/` to map camera serial numbers to video device indices. Supported camera naming patterns:
- `Camera_(\d+)` - Generic cameras with numeric serials
- `USB_Camera_([A-Z0-9]+)` - Arducam and similar devices
- `([A-Z0-9]+)-video-index0$` - Generic alphanumeric serials

## Output Structure

Each calibration session creates a timestamped directory:
```
calibration_data_YYYY_MM_DD_HH_MM_SS/
├── frame_1_766.png          # Frame 1 from camera 766
├── frame_1_UC762.png        # Frame 1 from camera UC762
├── frame_2_766.png          # Frame 2 from camera 766
├── frame_2_UC762.png        # Frame 2 from camera UC762
├── ...
└── framesets.json           # Metadata linking synchronized frames
```

### framesets.json Format
```json
[
  {
    "cap_idx": 1,
    "frames": {
      "766": "/path/to/frame_1_766.png",
      "UC762": "/path/to/frame_1_UC762.png"
    }
  }
]
```

## ROS Topics

The system publishes live camera feeds to:
- `/cameras/{location}/image_raw` - Raw camera images (sensor_msgs/Image)

Where `{location}` comes from the camera's configuration in `system_config.json`.

## Troubleshooting

### No Cameras Found
```
[ERROR] No cameras found! Please check camera connections and /dev/v4l/by-id/
```
**Solutions:**
- Verify cameras are connected and powered
- Check that `/dev/v4l/by-id/` exists and contains camera devices
- Ensure camera device names contain "Camera" or "camera"
- For Arducam devices, verify serial number configuration
- If you are on ubuntu using WSL2 from windows then read WSL_README.md

### Missing Camera Configuration
```
[ERROR] Missing system configuration for cameras: ['12345']
```
**Solutions:**
- Add the camera serial to `vision_config_data/data/system_config.json`
- Rebuild the `vision_config_data` package after changes

### Camera Initialization Failed
```
[ERROR] Cannot open camera 766 at video index 0
```
**Solutions:**
- Check camera permissions: `ls -la /dev/video*`
- Verify no other process is using the cameras
- Try different API preferences in camera config (V4L2, GSTREAMER, etc.)

## Running Without Launch File

For debugging, you can run the data collector directly:
```bash
ros2 run extrinsic_calibration data_collector
```

However, this won't start the Foxglove bridge, so you won't be able to visualize the images.
