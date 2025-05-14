# ros_vision
A ROS2 based vision system for robotics


# Overview

This repo contains a ROS2 based version of the vision system for Team766.  As of now this is a prototype for evaluation.

## Requirements

- OS: ubuntu 22.04
- a functioning Nvidia GPU - AMD not supported

## Install ROS

To install ROS2 follow these [instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

## Build The Code

- Clone the repo: `git clone git@github.com:Team766/ros_vision.git`
- `cd ros_vision`
- `source /opt/ros/humble/setup.bash`
- Run the install deps script: `sudo src/ros_vision/install_deps.sh`
- inspect your nvidia gpu compute capability with the following command `nvidia-smi --query-gpu compute_cap --format=csv`  
- edit the file `src/ros_vision/src/apriltags_cuda/CMakeLists.txt`, ensure the line that says `set(CMAKE_CUDA_ARCHITECTURES 52 CACHE STRING "CUDA architectures" FORCE)` matches the compute capability of your machine.  Note that nvidia-smi reports the capability as `X.Y` but you need to put in `XY` in the file.  Save the file.

- Now you are ready to build.  Note that the build will take quite a while as it pulls down dependencies like OpenCV and WpiLIB.

### Orin Building Instructions

The Orin is a bit wimpy so we need to make sure it only uses 2 processors during the build, otherwise it will fail.
```
colcon build --cmake-args -DNUM_PROCESSORS=2--event-handlers console_direct+
```

### x86_64 Build Instructions
```
colcon build --event-handlers console_direct+
```

## Run The Pipeline

### Start The USB Camera Node

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
### Start the Image Viewer

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


### Start The Apriltag Pipeline

- in another terminal run:

```
source install/setup.bash
ros2 run apriltags_cuda apriltags_cuda_node 
```

Find a 36H11 type apriltag and put it in front of the camera.  If everything is working you should see the following:

```
[INFO] [1747188386.237090632] [apriltags_detector]: GPU Apriltag Detector created, took 66 ms
[INFO] [1747188386.249408019] [apriltags_detector]: Tag id: 2, x: 0.037569, y: -0.174967, z: 0.590192, err: 0.000000
[INFO] [1747188386.254917685] [apriltags_detector]: Total Time: 14 ms, Det Time: 3 ms
[INFO] [1747188386.265243683] [apriltags_detector]: Tag id: 2, x: 0.037595, y: -0.174905, z: 0.590014, err: 0.000000
[INFO] [1747188386.269137448] [apriltags_detector]: Total Time: 10 ms, Det Time: 3 ms
[INFO] [1747188386.279630664] [apriltags_detector]: Tag id: 2, x: 0.037568, y: -0.174903, z: 0.589915, err: 0.000000
```
and a bunch of tag detections will get printed out to the screen.  What is happening is that the apriltags_cuda_node is listening to the topic /camera/image_raw.  When a new image is published to that topic, the node receives the image and processes the image with the apriltag algorithm.  If any tags are detected they are printed out to the screen.  The node also publishes an image (with or without any detections) to a topic called `/apriltags/images.

### Fire Up Another Viewer To View The Apriltag Detections

- in another terminal run:

```
source install/setup.bash
ros2 run seasocks_viewer seasocks_viewer_node --ros-args -p port:=9099 -p topic_name:=apriltags/images
```

If everything is working properly you should see the following output

```
info: Serving content from /home/cpadwick/code/ros_vision/install/ros_vision/share/seasocks_viewer/web
info: Listening on http://cpadwick-GS60-6QE:9099/
[INFO] [1746928224.850683239] [seasocks_viewer_node]: Seasocks viewer running at ws://localhost:'9099'/image
[INFO] [1746928224.850807410] [seasocks_viewer_node]: Reading images from topic: 'apriltags/images'
```

What is happening is that the seasocks viewer node is reading messages off of the `apriltags/images` topic, and is showing it on the web page at http://localhost:9099 . Navigate to the web page and verify that you can see the camera images with an apriltag detection shown on the image.





