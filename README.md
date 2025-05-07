# ros_vision
A ROS2 based vision system for robotics


# Overview

This repo contains a ROS2 based version of the vision system for Team766.  As of now this is a prototype for evaluation.

## Requirements

- OS: ubuntu 22.04
- cuda 11.8

## Install ROS

To install ROS2 follow these [instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

## Build The Code

Clone the repo: `git clone git@github.com:Team766/ros_vision.git`

```
cd ros_vision
source /opt/ros/humble/setup.bash
colcon build
```

### Run The Example

- plug in a USB camera
- in one terminal run:
```
source install/setup.bash
ros2 run usb_camera usb_camera_node
```
This will start the camera process which initializes the camera, reads frames from it, and publishes them to a topic called `camera/image_raw`.  If there is an error it probably means your camera is not on `/dev/video0`.  Try another index and rebuild.

If everything is working you should see something like this:

```
[ WARN:0] global ./modules/videoio/src/cap_gstreamer.cpp (1100) open OpenCV | GStreamer warning: Cannot query video position: status=0, value=-1, duration=-1
[INFO] [1746587393.692884138] [camera_publisher]: Opening camera on idx: '0'
[ WARN:0] global ./modules/videoio/src/cap_gstreamer.cpp (1100) open OpenCV | GStreamer warning: Cannot query video position: status=0, value=-1, duration=-1
[INFO] [1746587394.465825090] [camera_publisher]: Width: '1920'
[INFO] [1746587394.465877755] [camera_publisher]: Height: '1080'
[INFO] [1746587394.465889978] [camera_publisher]: FPS: '5'

```

- in another terminal run:

```
source install/setup.bash
ros2 run usb_camera image_processor_node
```

If everything is working properly you should see messages scrolling by like this:

```
[INFO] [1746586567.808330542] [image_processor]: Mean Intensity: 46.40, Processing Time: 11 ms
[INFO] [1746586568.001492335] [image_processor]: Mean Intensity: 46.48, Processing Time: 5 ms
[INFO] [1746586568.206662896] [image_processor]: Mean Intensity: 46.56, Processing Time: 4 ms
[INFO] [1746586568.412797075] [image_processor]: Mean Intensity: 46.59, Processing Time: 2 ms
[INFO] [1746586568.623250841] [image_processor]: Mean Intensity: 46.53, Processing Time: 4 ms
[INFO] [1746586568.829484276] [image_processor]: Mean Intensity: 46.56, Processing Time: 4 ms
[INFO] [1746586569.035463728] [image_processor]: Mean Intensity: 45.67, Processing Time: 2 ms
[INFO] [1746586569.247437095] [image_processor]: Mean Intensity: 47.45, Processing Time: 4 ms
[INFO] [1746586569.450778828] [image_processor]: Mean Intensity: 48.38, Processing Time: 2 ms
[INFO] [1746586569.663615818] [image_processor]: Mean Intensity: 47.88, Processing Time: 4 ms
```

What is happening is that the image_processor node is reading messages off of the `camera/image_raw` topic, and is processing each image and computing the mean value.

