import json
import os
import logging
from datetime import datetime

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from ros_vision_launch.utils import scan_for_cameras, get_config_data

# Configure global logging
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("ros_vision_launch")

BY_ID_PATH = "/dev/v4l/by-id/"
"""Path to the directory containing persistent V4L device symlinks by hardware ID."""


def load_bag_recording_config():
    """
    Load bag recording configuration from system_config.json

    Returns:
        dict: Bag recording configuration, or empty dict if not found
    """
    try:
        data_path = os.path.join(
            get_package_share_directory("vision_config_data"),
            "data",
            "system_config.json",
        )

        with open(data_path, "r") as f:
            config_data = json.load(f)

        bag_config = config_data.get("bag_recording", {})

        if bag_config:
            logger.info("Bag recording configuration loaded")
        else:
            logger.info("No bag recording configuration found")
        return bag_config

    except Exception as e:
        logger.warning(f"Could not load bag recording config: {e}")
        return {}


def load_performance_optimization_config():
    """
    Load performance optimization configuration from system_config.json

    Returns:
        dict: Performance optimization configuration, or empty dict if not found
    """
    try:
        data_path = os.path.join(
            get_package_share_directory("vision_config_data"),
            "data",
            "system_config.json",
        )

        with open(data_path, "r") as f:
            config_data = json.load(f)

        perf_config = config_data.get("performance_optimization", {})

        if perf_config:
            logger.info("Performance optimization configuration loaded")
        else:
            logger.info("No performance optimization configuration found")
        return perf_config

    except Exception as e:
        logger.warning(f"Could not load performance optimization config: {e}")
        return {}


def create_bag_recording_node(bag_config, camera_locations):
    """
    Create a ROS bag recording node based on configuration

    Args:
        bag_config (dict): Bag recording configuration
        camera_locations (dict): Mapping of serial->location for cameras

    Returns:
        ExecuteProcess: ROS bag recording process node with condition
    """
    if not bag_config:
        raise Exception(
            "Bag recording configuration is required when enable_bag_recording is true"
        )

    # Create output directory with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(
        bag_config.get("output_directory", "/tmp/ros_vision_bags"),
        f"ros_vision_{timestamp}",
    )

    # Build topic list
    topics = []

    # Add configured topics, expanding for each camera location
    configured_topics = bag_config.get("topics", [])
    for topic in configured_topics:
        if "{location}" in topic:
            # Expand topic for each camera location
            for location in set(camera_locations.values()):
                topics.append(topic.format(location=location))
        else:
            topics.append(topic)

    # Remove duplicates
    topics = list(set(topics))

    # Build ros2 bag record command
    cmd = ["ros2", "bag", "record"]

    # Add output directory
    cmd.extend(["-o", output_dir])

    # Add max bag size if specified
    max_size = bag_config.get("max_bag_size")
    if max_size:
        cmd.extend(["--max-bag-size", max_size])

    # Add max duration if specified
    max_duration = bag_config.get("max_duration")
    if max_duration:
        cmd.extend(["--max-bag-duration", str(max_duration)])

    # Add topics
    cmd.extend(topics)

    logger.info(f"Creating bag recording node with {len(topics)} topics")
    logger.info(f"Output directory: {output_dir}")
    logger.debug(f"Topics to record: {topics}")

    return ExecuteProcess(
        cmd=cmd,
        name="bag_recorder",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_bag_recording")),
    )


def generate_launch_description():
    """
    Generate a ROS2 LaunchDescription for all detected cameras.

    Scans for cameras, loads configuration, and sets up the usb_camera, apriltags, and foxglove_bridge nodes.

    Returns:
        LaunchDescription: The ROS2 launch description object with configured nodes.

    Raises:
        Exception: If camera detection or configuration loading fails.
    """
    logger.info("=== Starting ROS Vision Launch Configuration ===")

    # Add launch arguments for logging control
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for all nodes (debug, info, warn, error, fatal)",
    )
    measurement_mode_arg = DeclareLaunchArgument(
        "measurement_mode",
        default_value="false",
        description="Enable timing measurement and CSV logging for apriltags node.",
    )
    timing_csv_path_arg = DeclareLaunchArgument(
        "timing_csv_path",
        default_value="",
        description="Path to timing CSV file for apriltags node.",
    )

    enable_bag_recording_arg = DeclareLaunchArgument(
        "enable_bag_recording",
        default_value="true",
        description="Enable ROS bag recording of vision topics",
    )

    try:
        # First we scan for cameras.
        cameras_by_serial_id = scan_for_cameras()
        logger.info(f"Detected cameras: {cameras_by_serial_id}")

        cameras_by_location = get_config_data(cameras_by_serial_id)
        logger.info(f"Camera location mapping: {cameras_by_location}")

        # Load performance optimization configuration
        perf_config = load_performance_optimization_config()
        enable_optimizations = perf_config.get("enable_optimizations", False)
        available_cpu_cores = perf_config.get("available_cpu_cores", [])
        default_priority = perf_config.get("default_priority", 80)

        if enable_optimizations:
            logger.info(
                f"Performance optimizations enabled. Available CPU cores: {available_cpu_cores}, Default priority: {default_priority}"
            )
        else:
            logger.info("Performance optimizations disabled")

        # For each camera found, set up the image processing pipeline.
        nodes = [
            log_level_arg,
            measurement_mode_arg,
            timing_csv_path_arg,
            enable_bag_recording_arg,
        ]

        # Add launch info messages
        nodes.append(
            LogInfo(
                msg=f"Launching vision system with {len(cameras_by_serial_id)} cameras"
            )
        )

        node_count = 0
        camera_count = 0
        for serial_id, camera_idx in cameras_by_serial_id.items():
            cam_location = cameras_by_location[serial_id]
            logger.info(
                f"Setting up nodes for camera {serial_id} at location '{cam_location}'"
            )

            # Sequential CPU core assignment for camera and apriltags pairs
            camera_pin_to_core = -1  # Default: no pinning
            apriltags_pin_to_core = -1
            priority = default_priority

            if enable_optimizations and available_cpu_cores:
                # Sequential assignment: camera gets core N*2, apriltags gets core N*2+1
                camera_core_index = (camera_count * 2) % len(available_cpu_cores)
                apriltags_core_index = (camera_count * 2 + 1) % len(available_cpu_cores)

                camera_pin_to_core = available_cpu_cores[camera_core_index]
                apriltags_pin_to_core = available_cpu_cores[apriltags_core_index]

                logger.info(
                    f"Assigning camera {serial_id} to CPU core {camera_pin_to_core}"
                )
                logger.info(
                    f"Assigning AprilTags {serial_id} to CPU core {apriltags_pin_to_core}"
                )

            # USB Camera Node
            camera_node = Node(
                package="usb_camera",
                executable="usb_camera_node",
                name=f"camera_{serial_id}",
                parameters=[
                    {
                        "camera_idx": camera_idx,
                        "camera_serial": serial_id,
                        "topic_name": f"cameras/{cam_location}/image_raw",
                        "pin_to_core": camera_pin_to_core,
                        "priority": priority,
                    },
                ],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                output="screen",
            )
            nodes.append(camera_node)
            node_count += 1
            logger.debug(f"Added USB camera node for serial {serial_id}")

            # AprilTags Node (assuming it will also support pin_to_core and priority parameters)
            apriltags_node = Node(
                package="apriltags_cuda",
                executable="apriltags_cuda_node",
                name=f"apriltags_{serial_id}",
                parameters=[
                    {
                        "topic_name": f"cameras/{cam_location}/image_raw",
                        "camera_serial": serial_id,
                        "publish_images_to_topic": f"apriltags/{cam_location}/images",
                        "publish_pose_to_topic": f"apriltags/{cam_location}/pose",
                        "measurement_mode": LaunchConfiguration("measurement_mode"),
                        "timing_csv_path": LaunchConfiguration("timing_csv_path"),
                        "pin_to_core": apriltags_pin_to_core,
                        "priority": priority,
                    }
                ],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                output="screen",
            )
            nodes.append(apriltags_node)
            node_count += 1
            logger.debug(f"Added AprilTags node for serial {serial_id}")

            camera_count += 1

        # Add the foxglove bridge.
        foxglove_node = Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            output="screen",
        )
        nodes.append(foxglove_node)
        node_count += 1
        logger.info("Added Foxglove bridge node")

        # Add bag recording node (conditionally executed based on launch argument)
        bag_config = load_bag_recording_config()
        bag_recorder = create_bag_recording_node(bag_config, cameras_by_location)
        nodes.append(bag_recorder)
        node_count += 1
        logger.info(
            "Added ROS bag recording node (conditional on enable_bag_recording argument)"
        )

        logger.info(
            f"Launch configuration completed successfully with {node_count} nodes"
        )
        logger.info("=== ROS Vision Launch Configuration Complete ===")

        return LaunchDescription(nodes)

    except Exception as e:
        logger.error(f"Failed to generate launch description: {str(e)}")
        logger.error("=== ROS Vision Launch Configuration Failed ===")
        raise
