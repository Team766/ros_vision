# Extrinsic Camera Calibration

This document describes how to use the extrinsic calibration system to collect synchronized camera data for calibrating the relative positions and orientations of multiple cameras.

## Overview

The extrinsic calibration system automatically:
- Discovers all connected USB cameras
- Loads camera configurations from `system_config.json`
- Captures synchronized frames from all cameras
- Publishes live image streams for visualization
- Saves timestamped calibration data to disk

## Quick Start

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

### 4. Collect Data
The system will automatically:
- Capture frames at 0.5 Hz for 60 seconds (configurable)
- Save images to `./calibration_data/calibration_data_YYYY_MM_DD_HH_MM_SS/`
- Generate a `framesets.json` file with metadata

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

## Integration with Calibration Tools

The output format is designed to work with standard camera calibration tools:
- Each synchronized frame set is clearly identified in `framesets.json`
- Images are saved as PNG files with consistent naming
- Frame timestamps and camera mappings are preserved

Use the synchronized frame sets to calibrate:
- Intrinsic parameters for each camera individually
- Extrinsic parameters (relative poses) between camera pairs
- Complete multi-camera system calibration