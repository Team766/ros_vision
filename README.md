# ros_vision
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
docker run -it -v/tmp:/tmp --runtime=nvidia --gpus all -p 8765:8765 -v /dev/v4l/:/dev/v4l  --device /dev/video0 ros_vision:latest /bin/bash
```
This command maps the /tmp drive on the host to the /tmp drive in the docker container.

- At the container cmd line:

```
cd /tmp
mkdir code
cd code
git clone git@github.com:Team766/ros_vision.git
cd ros_vision
```

- inspect your nvidia gpu compute capability with the following command `nvidia-smi --query-gpu compute_cap --format=csv`  
- edit the file `./build_env_vars.sh`, ensure the line that says `export CMAKE_CUDA_ARCHITECTURES=52` matches the compute capability of your machine.  Note that nvidia-smi reports the capability as `X.Y` but you need to put in `XY` in the file.  Save the file.

Now run `./boostrap.sh` to build the code.

## Run The Pipeline

### Start The System Using The Launcher

- plug in a USB camera.  Arducam works best (at least we've tested it!)
- in a terminal type:

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

# Running In Optimized Mode

If you want to run the pipeline with the lowest latency possible, then launch the pipeline using the `./optimized_launch.bsh` command.  This command will:
 - Set the clocks to max, by running `sudo jetson_clocks`
 - Launch the vision processes exclusively on cores 4 and 5
 - Launch the vision processes using the realtime FIFO scheduler at priority 90

This will give priority to the threads running the vision system and will generally give the lowest latency and therefore is recommended for most applications.

## Optional: Isolate CPU's

You can isolate cores 4 and 5 and prevent the OS from assigning any processes to run on them, except ones that you specify.  This will guarantee that no processes will preempt the vision processes.

To do this:
1. sudo nano /boot/extlinux/extlinux.conf
2. Find the line starting with `APPEND ${cbootargs} ...`
3. Add `isolcpus=4,5` to the end of the line.  Save the file.
4. Reboot the machine `sudo reboot`

When the device reboots cores 4 and 5 should show at 0% utilization.  When you launch the vision system using optimized_launch.bsh, you will see that cores 4 and 5 will start to be come active, and will go back to 0% utilization when the vision system is shutdown.

# Running Timing Tests

## Enable Measurement Mode and Collect Timing Data

To collect timing measurements from the apriltags_cuda node:

1. Launch the vision system with measurement mode enabled:
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




