#!/bin/bash

source /opt/ros/humble/setup.bash
source ./build_env_vars.sh

env | grep CMAKE

if [ ! -e "src/vision_opencv" ] ; then
    echo "Cloning vision_opencv into workspace"
    cd src
    git clone -b humble https://github.com/ros-perception/vision_opencv.git
    cd ..
fi

if [ ! -e "src/image_transport_plugins" ] ; then
    echo "Cloning image transport plugins into workspace"
    cd src
    git clone -b humble https://github.com/ros-perception/image_transport_plugins.git
    cd ..
fi

# Build the vision deps package first so our version of opencv builds.
if [ ! -e "install/vision_deps/opencv-install/share/opencv4/" ] ; then
    echo "Building vision_deps"
    colcon build --packages-select vision_deps
fi

source install/setup.bash
export OpenCV_DIR=$PWD/install/vision_deps/opencv-install/lib/cmake/opencv4

# Now build cv_bridge against our version of opencv.
if [ ! -e "install/cv_bridge/lib/libcv_bridge.so" ] ; then
    echo "Building cv_bridge"
    colcon build --packages-select cv_bridge
fi

# Now build the image transport packages (allow overriding ROS-provided plugins)
colcon build --packages-select compressed_image_transport theora_image_transport \
  --allow-overriding compressed_image_transport theora_image_transport

# Download YOLO models if not present
if [ ! -d "src/game_piece_detection/models/2026_rebuilt" ] ; then
    echo "Downloading YOLO models..."
    pushd src/game_piece_detection/models
    ./setup_env.sh
    ./run.sh ./download_models.sh
    popd
fi

# Now we should be good to go for the rest of the build
echo "Building all"
colcon build --allow-overriding compressed_image_transport theora_image_transport