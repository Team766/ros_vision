#!/bin/bash

# Convenience script for running tests in the ros_vision workspace

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh
source install/setup.bash

# Parse arguments for CPU-only mode
CPU_ONLY=false
COLCON_ARGS=""

for arg in "$@"; do
    case $arg in
        --cpu-only)
            CPU_ONLY=true
            ;;
        --help|-h)
            echo "ROS Vision Test Runner"
            echo ""
            echo "Usage: $0 [OPTIONS] [COLCON_ARGS]"
            echo ""
            echo "Options:"
            echo "  --cpu-only        Skip GPU-dependent tests (excludes gpu_detector_test)"
            echo "  --help, -h        Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Run all tests"
            echo "  $0 --cpu-only                        # Run CPU-only tests"
            echo "  $0 --packages-select apriltags_cuda  # Test specific package"
            echo "  $0 --cpu-only --packages-select apriltags_cuda"
            echo ""
            echo "All other arguments are passed directly to 'colcon test'."
            echo "See 'colcon test --help' for additional colcon options."
            exit 0
            ;;
        *)
            COLCON_ARGS="$COLCON_ARGS $arg"
            ;;
    esac
done

if [ "$CPU_ONLY" = true ]; then
    echo "🖥️  Running CPU-only tests (excluding GPU-dependent tests)..."
    echo "Excluded tests: gpu_detector_test"
    echo ""
    
    echo "Running colcon test with GPU test exclusions..."
    # Use ctest exclusion pattern to skip gpu_detector_test
    colcon test --ctest-args "-E" "gpu_detector_test" $COLCON_ARGS
else
    echo "🚀 Running all tests (including GPU-dependent tests)..."
    echo "Running colcon test..."
    colcon test $COLCON_ARGS
fi

echo -e "\n=== Test Results Summary ==="
colcon test-result --all

echo -e "\n=== Detailed Results ==="
colcon test-result --verbose