#!/usr/bin/env bash
# Build the ros_vision Docker image for linux/arm64 on an x64 host (e.g. for Jetson deployment).
#
# One-time setup on the x64 host:
#   docker run --privileged --rm tonistiigi/binfmt --install all
# (enables QEMU so the host can emulate arm64 during the build)
#
# To copy the image to the arm64 device:
#   docker save ros_vision:arm64 | gzip > ros_vision_arm64.tar.gz
#   # transfer file to device, then on device:
#   docker load < ros_vision_arm64.tar.gz

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Building ros_vision for linux/arm64 (slower under QEMU emulation)..."
docker buildx build --platform linux/arm64 -t ros_vision:arm64 --load .

echo "Done. Image: ros_vision:arm64"
echo "On arm64 device: docker run -it --runtime=nvidia --gpus all ros_vision:arm64 /bin/bash"
