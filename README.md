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
colcon build --event-handlers console_direct+
```

### Run The Example

- plug in a USB camera
- in one terminal run:
```
source install/setup.bash
ros2 run usb_camera usb_camera_node --ros-args -p camera_idx:=0 -p topic_name:=camera/image_raw
```
This will start the camera process which initializes the camera in index 0, reads frames from it, and publishes them to a topic called `camera/image_raw`.  If there is an error it probably means your camera is not on `/dev/video0`.  Try another index and rebuild.

If everything is working you should see something like this:

```
[INFO] [1746927683.481375637] [camera_publisher]: Opening camera on idx: '0'
[INFO] [1746927683.781965633] [camera_publisher]: Width: '1280'
[INFO] [1746927683.782025468] [camera_publisher]: Height: '800'
[INFO] [1746927683.782056474] [camera_publisher]: FPS: '100'
[INFO] [1746927683.782076952] [camera_publisher]: Pubishing on Topic: 'camera/image_raw'

```

- in another terminal run:

```
source install/setup.bash
ros2 run seasocks_viewer seasocks_viewer_node --ros-args -p port:=9090 -p topic_name:=camera/image_raw
```

If everything is working properly you should see the following output

```
info: Serving content from /home/cpadwick/code/ros_vision/install/ros_vision/share/seasocks_viewer/web
info: Listening on http://cpadwick-GS60-6QE:9090/
[INFO] [1746928224.850683239] [seasocks_viewer_node]: Seasocks viewer running at ws://localhost:'9090'/image
[INFO] [1746928224.850807410] [seasocks_viewer_node]: Reading images from topic: 'camera/image_raw'
```

What is happening is that the seasocks viewer node is reading messages off of the `camera/image_raw` topic, and is showing it on the web page at http://localhost:9090 . Navigate to the web page and verify that you can see the camera images.

