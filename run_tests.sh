#!/bin/bash

# Convenience script for running tests in the ros_vision workspace

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh
source install/setup.bash

# Avoid user-site Python packages (e.g., ~/.local) overriding ROS Humble's
# bundled test tooling (notably launch_pytest's compatibility with pytest).
#
# Opt out by setting: ROS_VISION_ALLOW_USER_SITE=1
if [ -z "${ROS_VISION_ALLOW_USER_SITE:-}" ]; then
    export PYTHONNOUSERSITE=1
fi

# Parse arguments for CPU-only mode
CPU_ONLY=false
INCLUDE_REALSENSE_TESTS=false
DELETE_STALE_REALSENSE_RESULTS=true
COLCON_ARGS=""

for arg in "$@"; do
    case $arg in
        --cpu-only)
            CPU_ONLY=true
            ;;
        --include-realsense-tests)
            INCLUDE_REALSENSE_TESTS=true
            ;;
        --keep-stale-realsense-results)
            DELETE_STALE_REALSENSE_RESULTS=false
            ;;
        --help|-h)
            echo "ROS Vision Test Runner"
            echo ""
            echo "Usage: $0 [OPTIONS] [COLCON_ARGS]"
            echo ""
            echo "Options:"
            echo "  --cpu-only        Skip GPU-dependent tests (excludes gpu_detector_test)"
            echo "  --include-realsense-tests  Include realsense2_camera tests (hardware/integration)"
            echo "  --keep-stale-realsense-results  Do not delete old realsense2_camera test results when skipping"
            echo "  --help, -h        Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Run all tests"
            echo "  $0 --cpu-only                        # Run CPU-only tests"
            echo "  $0 --include-realsense-tests          # Also run realsense2_camera tests"
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

# CI-friendly default: the realsense2_camera package contains integration-style tests
# that may require RealSense hardware and/or large rosbag playback.
if [ "$INCLUDE_REALSENSE_TESTS" = false ]; then
    COLCON_ARGS="--packages-skip realsense2_camera $COLCON_ARGS"

    # Avoid reporting stale failures from previous runs when the package is skipped.
    if [ "$DELETE_STALE_REALSENSE_RESULTS" = true ]; then
        rm -rf build/realsense2_camera/Testing 2>/dev/null || true
        rm -rf build/realsense2_camera/test_results 2>/dev/null || true
        rm -rf build/realsense2_camera/ament_cmake_pytest 2>/dev/null || true
    fi
fi

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