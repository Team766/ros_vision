#!/bin/bash

install_tensorrt_x86() {
    # Check and see if we have the file locally or not.
    deb_file="nv-tensorrt-local-repo-ubuntu2204-10.6.0-cuda-11.8_1.0-1_amd64.deb"
    if ! [ -f "$deb_file" ]; then
        # We need to get "gdown" so we can download the deb file, so build a python virtual
        # env and install it there.
		if ! [ -d ".venv" ]; then
			python3 -m venv .venv
		fi
        source .venv/bin/activate
        pip install --upgrade pip
        pip install gdown
		echo "Downloading tensorrt .deb file..."
		# This file is hosted in our team 766 drive.  If you don't have access to it you will need
		# to login to the Nvidia Developer Portal https://developer.nvidia.com/tensorrt/download/10x
		# and download it.  The code in this repo requires version >= 10.6. The site is not paywalled,
		# but you will need to sign up for a free NVIDIA developer account before you can download.
        gdown 'https://drive.google.com/uc?id=18aKOPAA338Q6QFhzaVigEoMcPxc5r9gF&export=download' -O "$deb_file"
        deactivate
    fi
    sudo dpkg -i "$deb_file"
	sudo cp /var/nv-tensorrt-local-repo-ubuntu2204-10.6.0-cuda-11.8/nv-tensorrt-local-2C8302C3-keyring.gpg /usr/share/keyrings/

    # Remove any existing NVIDIA CUDA repo lists that might have newer TensorRT
    sudo rm -f /etc/apt/sources.list.d/cuda*.list 2>/dev/null || true

    sudo apt-get update

    # Install TensorRT from the local repo (10.6 for CUDA 11.8)
    sudo apt-get -y install tensorrt
}

install_tensorrt_jetson() {
	# These packages only exist in jetson, not main ubuntu/debian
	sudo apt install -y tensorrt nvidia-tensorrt-dev python3-libnvinfer-dev
}


install_cuda_x86() {
	wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
	sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
	wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb
	sudo dpkg -i cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb
	sudo cp /var/cuda-repo-ubuntu2204-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
	sudo apt-get update
	sudo apt-get -y install cuda-11-8
}

install_cuda_jetson() {
	wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/arm64/cuda-ubuntu2004.pin
	sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
	wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-tegra-repo-ubuntu2004-11-8-local_11.8.0-1_arm64.deb
	sudo dpkg -i cuda-tegra-repo-ubuntu2004-11-8-local_11.8.0-1_arm64.deb
	sudo cp /var/cuda-tegra-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
	sudo apt-get update
	sudo apt-get -y install cuda-11-8
}

install_llvm() {
	wget https://apt.llvm.org/llvm.sh
	chmod +x llvm.sh
	sudo ./llvm.sh 17
}

export DEBIAN_FRONTEND=noninteractive

arch=$(uname -m)

# Remove NVIDIA online repos early to prevent wrong TensorRT version from being pulled
# as a dependency during other apt installs
sudo rm -f /etc/apt/sources.list.d/cuda*.list 2>/dev/null || true

sudo apt update -y
sudo apt install -y wget build-essential cmake python3-dev python3-venv python3-numpy libprotobuf-dev protobuf-compiler
sudo apt install -y libgoogle-glog-dev libgtest-dev libssh-dev libxrandr-dev libxinerama-dev libstdc++-12-dev
sudo apt install -y golang

# Install ROS sources if needed
if ! grep -q "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d/* 2>/dev/null; then
    sudo apt update
    sudo apt install -y curl gnupg2 lsb-release
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/ros2-latest.list
    sudo apt update
fi
# ----------------------------------------------------------------------

# Install ROS packages
sudo apt install -y ros-humble-ros-base ros-dev-tools
sudo apt-get install -y libboost-all-dev python3-dev python3-numpy
sudo apt-get install -y libboost-python-dev libboost-system-dev libboost-thread-dev
sudo apt-get install -y ros-humble-image-transport
sudo apt-get install -y libtheora-dev libogg-dev pkg-config
sudo apt-get install -y ros-humble-foxglove-bridge


# Check if clang-17 is installed.
if ! dpkg -l | grep clang-17; then
	install_llvm
fi

# Install TensorRT if needed
if ! dpkg -l | grep -qi tensorrt; then
    echo "$arch"
    if [ "$arch" = "x86_64" ]; then
        install_tensorrt_x86
    elif [ "$arch" = "aarch64" ] || [ "$arch" = "arm64" ]; then
        install_tensorrt_jetson
    else
        echo "Unsupported architecture: $arch"
        exit 1
    fi
else
    echo "TensorRT is already installed."
fi

# Check if cuda-toolkit is installed
if dpkg -l | grep -q cuda-toolkit-11-8; then
	echo "cuda-toolkit 11-8 is installed."
	exit 0
fi

case $arch in
    x86_64)
        echo "Installing cuda on x86"
        install_cuda_x86
        ;;
    aarch64)
        case $(cat /proc/cpuinfo) in
	    *0xd42*)
                echo "Installing cuda on jetson"
                install_cuda_jetson
                ;;    
	    *)
                echo "Device is not detected to be Jetson Orin Nano; Not installing CUDA."
	        ;;
        esac
	;;
    *)
        echo "Unknown architecture: $arch"
        # Handle unknown architecture here
        ;;
esac


