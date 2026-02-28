#!/bin/bash
#
# Deploy system_config.json to a remote host and restart the vision service.
#
# Usage: ./deploy_config.sh <host>
#
# Example: ./deploy_config.sh orin
#          ./deploy_config.sh cpadwick@192.168.1.100

set -euo pipefail

if [ $# -ne 1 ]; then
    echo "Usage: $0 <host>"
    echo "Example: $0 orin"
    exit 1
fi

HOST="$1"
LOCAL_FILE="src/vision_config_data/data/system_config.json"
REMOTE_DIR="code/ros_vision/src/vision_config_data/data"
REMOTE_FILE="${REMOTE_DIR}/system_config.json"

if [ ! -f "$LOCAL_FILE" ]; then
    echo "ERROR: $LOCAL_FILE not found. Run this from the ros_vision workspace root."
    exit 1
fi

echo "=== Deploying system_config.json to ${HOST} ==="
echo ""

echo "[1/3] Copying $LOCAL_FILE to ${HOST}:~/${REMOTE_FILE} ..."
scp "$LOCAL_FILE" "${HOST}:~/${REMOTE_FILE}"
echo "      Done."
echo ""

echo "[2/3] Building vision_config_data on ${HOST} ..."
ssh "$HOST" "cd ~/code/ros_vision && source /opt/ros/humble/setup.bash && source ./build_env_vars.sh && source install/setup.bash && colcon build --packages-select vision_config_data"
echo "      Done."
echo ""

echo "[3/3] Restarting ros_vision service on ${HOST} ..."
ssh -t "$HOST" "sudo systemctl restart ros_vision.service"
echo "      Done."
echo ""

echo "=== Deploy complete ==="
