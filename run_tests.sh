#!/bin/bash

# Convenience script for running tests in the ros_vision workspace

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh
source install/setup.bash

echo "Running colcon test..."
colcon test "$@"

echo -e "\n=== Test Results Summary ==="
colcon test-result --all

echo -e "\n=== Detailed Results (use --verbose for more) ==="
if [[ "$*" == *"--verbose"* ]]; then
    colcon test-result --verbose
else
    echo "Run with --verbose flag to see detailed test output"
fi