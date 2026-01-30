# ros_vision
[![Build](https://github.com/Team766/ros_vision/actions/workflows/cmake-single-platform.yml/badge.svg?branch=main)](https://github.com/Team766/ros_vision/actions/workflows/cmake-single-platform.yml)

A ROS2 based vision system for robotics


# Overview

This repo contains a ROS2 based version of the vision system for Team766.  As of now this is a prototype for evaluation.

## Requirements

- OS: ubuntu 22.04
- a functioning Nvidia GPU - AMD not supported

## Install Foxglove

Foxglove studio is a powerful tool that lets you inspect ros image messages, topics, and outputs.  I recommend installing it.

- Download Foxglove Studio from this [link](https://foxglove.dev/download)
- install with `sudo apt install ./foxglove-studio-latest-linux-amd64.deb`
- `sudo apt update && sudo apt install foxglove-studio`

Install the ros2 Foxglove bridge:

- `sudo apt install ros-humble-foxglove-bridge`

## Build The Code

- Clone the repo: `git clone git@github.com:Team766/ros_vision.git`
- `cd ros_vision`
- `source /opt/ros/humble/setup.bash`
- Run the install deps script: `sudo ./install_deps.sh`
- inspect your nvidia gpu compute capability with the following command `nvidia-smi --query-gpu compute_cap --format=csv`  
- edit the file `./build_env_vars.sh`, ensure the line that says `export CMAKE_CUDA_ARCHITECTURES=52` matches the compute capability of your machine.  Note that nvidia-smi reports the capability as `X.Y` but you need to put in `XY` in the file.  Save the file.
- source the file `source ./build_env_vars.sh`.  Now your environment is ready and you can build the code.
- Now you are ready to build.  Note that the build will take quite a while as it pulls down dependencies like OpenCV and WpiLIB.

### Building Instructions

Run the following command to build the code the first time.  This will pull down the dependencies and build them in the right order.
```
./bootstrap.sh
```

### Building The Code in a Docker Container

- [Install docker on ubuntu](https://docs.docker.com/engine/install/ubuntu/)

- Follow the instructions for [installing the nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

- Edit the nvidia container config file `sudo vim /etc/nvidia-container-runtime/config.toml` and ensure that `no-cgroups = false`, and save the file.

- Restart the docker daemon: `sudo systemctl restart docker`

- Run the docker build command in the current directory as follows: `docker build -t ros_vision:latest .`

- When the docker build completes, run the docker in interactive mode with the following command: 

```bash
 docker run -it -v/tmp:/tmp --runtime=nvidia --gpus all -p 8765:8765 -v /dev/v4l/:/dev/v4l  --user $(id -u):$(id -g) -e USER=$USER -e HOME=$HOME -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro -v /home/$USER:/home/$USER ros_vision:latest /bin/bash
```

This command maps your home directory on the host to the container, and sets your user id and group id to the same as on the host.  This allows you to run as your own user id inside the container instead of root.

If you want to connect a camera and try some testing then add the tag `--device /dev/video0` to the command.

Now you can build just like you would outside the container:

```
cd ~
mkdir code
cd code
git clone git@github.com:Team766/ros_vision.git
cd ros_vision
```

- inspect your nvidia gpu compute capability with the following command `nvidia-smi --query-gpu compute_cap --format=csv`  
- edit the file `./build_env_vars.sh`, ensure the line that says `export CMAKE_CUDA_ARCHITECTURES=52` matches the compute capability of your machine.  Note that nvidia-smi reports the capability as `X.Y` but you need to put in `XY` in the file.  Save the file.

Now run `./boostrap.sh` to build the code.

## Run The Pipeline

### Start The System Using The Launcher Script (Recommended)

- plug in a USB camera.  Arducam works best (at least we've tested it!)
- in a terminal type:

```
./start_vision.bsh
```

This convenient script automatically sources all necessary environment files and launches the vision system. For help with launch arguments:

```
./start_vision.bsh --help
```

### Install As A Ubuntu Service

You can install the ros_vision system as a service that will run on each boot.  To install, run the install script as follows:

```
./install_service.bsh
```

You can verify that the service is working with:

```
systemctl status ros_vision.service
```

You can inspect logs with 
```
journalctl -u ros_vision.service
```


### Start The System Manually

Alternatively, you can launch manually:

```
source install/setup.bash
ros2 launch ros_vision_launch launch_vision.py
```

This will launches the following nodes:
- the camera will attempt to be autodetected.  This will succeed if the string Camera is found in the camera entry in the /dev/v4l/by-id directory.  
- usb_camera_node which collects frames from the camera and publishes them to a topic
- apriltags_cuda node which takes frames from the topic above and runs the apriltag detection alg on the GPU.  It prints out detection data to the screen and also publishes detection images to a second topic.
- a foxglove bridge node, which makes it easy to view the messages in FoxGlove.

If everything is working properly you should see some text scrolling by on the screen corresponding to apriltag detections.

### Startup Foxglove Studio

On the host machine startup Foxglove Studio (Activities -> Foxglove).  If it is your first time installing it you need to sign up for an account.  It's free so go ahead and do that.  If this bothers you then you don't have to use foxglove, you could use the Seasocks viewer (see below).

Once foxglove is running, click Open Connection and select the local webserver as shown below.

![Opening a connecction in foxglove](res/openconnection.png)

Once the connection is opened you should see the two images displayed, one from the usb camera and one from the aprtiltag detection.

![Foxglove studio showing the two camera feeds](res/foxglovestudio.png)

# ROS Bag Recording

The vision system includes built-in support for recording ROS bags to capture vision data for analysis, debugging, and replay.

## Configuration

Bag recording is configured in the `system_config.json` file under the `bag_recording` section:

```json
{
  "bag_recording": {
    "output_directory": "/tmp/ros_vision_bags",
    "max_bag_size": "1GB",
    "compression": "zstd",
    "topics": [
      "cameras/{location}/image_raw",
      "apriltags/{location}/images", 
      "apriltags/{location}/pose",
      "apriltags/{location}/pose_camera"
    ],
    "auto_split": true,
    "max_duration": 300
  }
}
```

### Configuration Options

- **output_directory**: Directory where bag files will be saved (with timestamp subdirectories)
- **max_bag_size**: Maximum size per bag file in bytes
- **topics**: List of topics to record (supports `{location}` template expansion)
- **auto_split**: Whether to automatically split into multiple files
- **max_duration**: Maximum recording duration in seconds

### Disable Bag Recording

Bag recording is enabled by default.  To disable it use:

```bash
./start_vision.bsh enable_bag_recording:=false
```

Or manually:

```bash
source install/setup.bash
ros2 launch ros_vision_launch launch_vision.py enable_bag_recording:=false
```

### Output

Bags are saved with timestamped directories like:
```
/tmp/ros_vision_bags/ros_vision_20250803_175413/
```

### Topic Template Expansion

The `{location}` template in topic names automatically expands for each camera location. For example:
- `cameras/{location}/image_raw` becomes:
  - `cameras/center_front/image_raw`
  - `cameras/left_side/image_raw`

### Viewing Recorded Data

You can replay recorded bags using standard ROS2 tools:

```bash
# Play back the entire bag
ros2 bag play /tmp/ros_vision_bags/ros_vision_20250803_175413/

# Play back specific topics only
ros2 bag play /tmp/ros_vision_bags/ros_vision_20250803_175413/ --topics /cameras/center_front/image_raw

# Play at different speeds
ros2 bag play /tmp/ros_vision_bags/ros_vision_20250803_175413/ --rate 0.5
```

### Viewing Data In Foxglove

You can also view the data in Foxglove Studio but you need to convert it to mcap format, as it can't read all the messages in db3 format.  To convert do this:
 - cd to the root directory, e.g. `/tmp`
 - run the following command: 

```
cat << EOF > convert.yaml
output_bags:
  - uri: ros2_output
    storage_id: mcap
    all: true
EOF
```

This will write a file called `convert.yaml` in the `/tmp` directory.  Now if you run `ros2 bag convert -i <path_to_ros_bag_directory> -o convert.yaml` a new directory called ros2_output will be created that contains mcap versions of the db3 files.  You can load this into foxglove directly.

# Running With CPU Pinning and Realtime Prority Scheduling

The system supports pinning the usb_camera and apriltags_cuda processes to different CPU's and adjusting the process priority with the realtime scheduler.  To enable this, edit the system_config.json file.  Find the section called performance_optimizations:

```
"performance_optimization": {
      "enable_optimizations": false,
      "available_cpu_cores": [4, 5],
      "default_priority": 80
  }
```

Set enable_optimizations to true, and modify the available_cpu_cores list to match your environment.  This will give priority to the threads running the vision system and will generally give the lowest latency and therefore is recommended for most applications.


Currently the usb_camera_node and apriltags_cuda_node will be pinned to different CPU's.  So if you have available_cpu_cores set to [4,5] and have one camera, then the usb_camera_node will get pinned to core 4 and the apriltags_cuda_node will be pinned to core 5.  Note that if you have 2 cameras, you should put 4 cores in the available_cpu_cores list, otherwise multiple processes will get pinned to the same core, and the second one that gets pinned will get no cycles because of the realtime FIFO scheduling.

You may also need to add the following line: `nvidia		-	rtprio		90` to /etc/security/limits.conf and reboot in order to have the right permissions to use the realtime scheduler.

Rebuild the pipeline to pick up the newly edited config file `colcon build --packages-select vision_config_data`

Launch the pipeline as normal, and you should see messages like the following:

```
[apriltags_cuda_node-2] [INFO] [1754440650.881978858] [apriltags_UC762]: Timing CSV path: 'apriltags_timing_20250805_173730.csv'
[apriltags_cuda_node-2] [INFO] [1754440650.882137513] [apriltags_UC762]: Successfully pinned process to CPU core 5
[apriltags_cuda_node-2] [INFO] [1754440650.882267336] [apriltags_UC762]: Successfully set real-time FIFO scheduling with priority 80
[apriltags_cuda_node-2] [INFO] [1754440650.882301927] [apriltags_UC762]: CPU affinity verification: Successfully pinned to core 5
[apriltags_cuda_node-2] [INFO] [1754440650.882324519] [apriltags_UC762]: Scheduling verification: FIFO policy with priority 80

```

## Optional: Isolate CPU's

You can isolate cores 4 and 5 and prevent the OS from assigning any processes to run on them, except ones that you specify.  This will guarantee that no processes will preempt the vision processes.

To do this:
1. sudo nano /boot/extlinux/extlinux.conf
2. Find the line starting with `APPEND ${cbootargs} ...`
3. Add `isolcpus=4,5` to the end of the line.  Save the file.
4. Reboot the machine `sudo reboot`

When the device reboots cores 4 and 5 should show at 0% utilization.  When you launch the vision system you will see that cores 4 and 5 will start to be come active, and will go back to 0% utilization when the vision system is shutdown.

# Running Timing Tests

## Enable Measurement Mode and Collect Timing Data

To collect timing measurements from the apriltags_cuda node:

1. Launch the vision system with measurement mode enabled:
   ```bash
   ./start_vision.bsh measurement_mode:=true
   ```
   
   Or manually:
   ```bash
   source install/setup.bash
   ros2 launch ros_vision_launch launch_vision.py measurement_mode:=true
   ```
   This will create a timing CSV file (e.g., `apriltags_timing_<timestamp>.csv`) in your workspace.

2. Let the system run for as long as you want to collect timing data, then stop it with `Ctrl+C`.

## Generate Timing Reports and Plots

1. Install Python dependencies for the timing report utility:
   ```bash
   pip install -r src/vision_utils/requirements.txt
   ```

2. Run the timing report utility on your CSV file:
   ```bash
   python src/vision_utils/timing_report.py apriltags_timing_<timestamp>.csv
   ```
   This will print a summary report, including CDF statistics, and generate plots (line, histogram, and CDF) for each timing metric. The plots will be saved as PNG files in your workspace.

## PDF Report Generation

To generate a PDF timing report from the Markdown file, you need to have [pandoc](https://pandoc.org/) and a LaTeX engine installed. On Linux, you can install them with:

```bash
sudo apt install pandoc texlive-latex-base texlive-latex-recommended
```

The timing_report.py utility will automatically attempt to generate a PDF if pandoc is available. If not, you can manually run:

```bash
pandoc apriltags_timing_<timestamp>_report.md -o apriltags_timing_<timestamp>_report.pdf
```

This will create a PDF with all tables and embedded images from your timing analysis.




