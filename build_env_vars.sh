#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export CMAKE_BUILD_TYPE=Release
export CMAKE_CUDA_COMPILER=/usr/bin/clang++-17
export CUDACXX=/usr/bin/clang++-17
export CMAKE_CXX_COMPILER=/usr/bin/clang++-17
export CMAKE_CUDA_ARCHITECTURES=87

export LD_LIBRARY_PATH="$SCRIPT_DIR/install/vision_deps/opencv-install/lib:/usr/lib/aarch64-linux-gnu/nvidia/:$LD_LIBRARY_PATH"
export PYTHONPATH="$SCRIPT_DIR/install/vision_deps/opencv-install/lib/python3.10/dist-packages:$PYTHONPATH"

