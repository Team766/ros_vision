import json
import os
import re
import sys
import logging

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Configure global logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger('ros_vision_launch')

BY_ID_PATH = "/dev/v4l/by-id/"
"""Path to the directory containing persistent V4L device symlinks by hardware ID."""


def scan_for_cameras():
    """
    For each camera, find the /dev/video* device associated with its -index0 symlink,
    and map the serial number to the actual video index (as integer).
    """
    logger.info("Starting camera scan...")
    
    if not os.path.exists(BY_ID_PATH):
        logger.error(f"{BY_ID_PATH} does not exist")
        raise Exception(f"Error: {BY_ID_PATH} does not exist.")

    logger.debug(f"Scanning directory: {BY_ID_PATH}")
    cameras_by_serial_id = {}
    entries = os.listdir(BY_ID_PATH)
    logger.info(f"Found {len(entries)} entries in {BY_ID_PATH}")
    
    for entry in entries:
        logger.debug(f"Processing entry: {entry}")
        
        # Only consider symlinks ending in -index0 (main video stream)
        if not entry.endswith("index0"):
            logger.debug(f"Skipping {entry}: does not end with 'index0'")
            continue
            
        # Check if this is a camera device (broader pattern matching)
        if "Camera" not in entry and "camera" not in entry.lower():
            logger.debug(f"Skipping {entry}: does not contain 'Camera' or 'camera'")
            continue

        logger.debug(f"Processing camera device: {entry}")

        # Extract the serial number from the symlink name
        # First try the original pattern for devices with Camera_(\d+)
        match = re.search(r"Camera_(\d+)", entry)
        if match:
            serial = match.group(1)
            logger.debug(f"Extracted serial using Camera pattern: {serial}")
        else:
            # Try Arducam pattern: extract the serial from UC### or similar
            arducam_match = re.search(r"USB_Camera_([A-Z0-9]+)", entry)
            if arducam_match:
                serial = arducam_match.group(1)
                logger.debug(f"Extracted serial using Arducam pattern: {serial}")
            else:
                # Generic fallback: extract any alphanumeric identifier before -video-index0
                generic_match = re.search(r"([A-Z0-9]+)-video-index0$", entry)
                if generic_match:
                    serial = generic_match.group(1)
                    logger.debug(f"Extracted serial using generic pattern: {serial}")
                else:
                    logger.warning(f"Could not extract serial from {entry}, skipping")
                    continue

        # Resolve symlink to get the /dev/video* device path
        device_path = os.path.realpath(os.path.join(BY_ID_PATH, entry))
        logger.debug(f"Resolved device path: {device_path}")

        # Robustly extract the number from /dev/videoN
        vid_match = re.search(r"/dev/video(\d+)$", device_path)
        if not vid_match:
            logger.warning(f"Could not extract video index from {device_path}, skipping")
            continue

        video_index = int(vid_match.group(1))
        cameras_by_serial_id[serial] = video_index
        logger.info(f"Found camera: serial={serial}, video_index={video_index}, device={entry}")

    if not cameras_by_serial_id:
        error_msg = "\n\nError: No camera devices found in by-id entries!\n\n"
        error_msg += "This utility depends on finding the string 'Camera' or 'camera' somewhere in the filename.\n"
        error_msg += f"We found these devices: {list(os.listdir(BY_ID_PATH))}\n\n"
        error_msg += "To debug, try listing the contents of /dev/v4l/by-id and confirming that Camera is in the device filename.\n"
        error_msg += "With an arducam, try resetting the serial number so that the Device Name is 'Camera': https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/"
        logger.error(error_msg)
        raise Exception(error_msg)

    logger.info(f"Camera scan completed. Found {len(cameras_by_serial_id)} cameras")
    return cameras_by_serial_id


def check_format(config_data):
    """
    Placeholder function to validate the format of the configuration data.

    Args:
        config_data (dict): The configuration data loaded from JSON.

    Returns:
        None

    Raises:
        NotImplementedError: If validation is not yet implemented.
    """
    logger.debug("Checking configuration format...")
    # TODO: implement this
    logger.warning("Configuration format validation not yet implemented")


def get_config_data(cameras_by_serial_id):
    """
    Load and validate camera configuration data from the ROS package.

    Args:
        cameras_by_serial_id (dict): Mapping of camera serials (str) to video indices (int).

    Returns:
        dict: Mapping of camera serial numbers (str) to their camera locations from config.

    Raises:
        Exception: If the configuration file is not found or required keys are missing.
    """
    logger.info("Loading configuration data...")
    
    data_path = os.path.join(
        get_package_share_directory("vision_config_data"), "data", "system_config.json"
    )
    logger.debug(f"Configuration file path: {data_path}")
    
    if not os.path.exists(data_path):
        logger.error(f"Configuration file not found at {data_path}")
        raise Exception(f"Error: unable to find system_config.json at {data_path}")

    logger.debug("Reading configuration file...")
    with open(data_path, "r") as f:
        config_data = json.load(f)

    logger.info("Configuration file loaded successfully")
    check_format(config_data)

    camera_mounted_positions = config_data["camera_mounted_positions"]
    logger.debug(f"Available camera positions in config: {list(camera_mounted_positions.keys())}")
    
    result = {}
    for k, v in cameras_by_serial_id.items():
        if k not in camera_mounted_positions:
            logger.error(f"Camera serial {k} not found in configuration")
            raise Exception(f"Camera serial {k} not found in camera_mounted_positions configuration")
        
        # Extract the location from the camera config object
        camera_config = camera_mounted_positions[k]
        if isinstance(camera_config, dict) and "location" in camera_config:
            # New format: camera config is an object with location field
            result[k] = camera_config["location"]
            logger.debug(f"Mapped camera {k} to location {camera_config['location']} (new format)")
        elif isinstance(camera_config, str):
            # Old format: camera config is just a location string (backward compatibility)
            result[k] = camera_config
            logger.debug(f"Mapped camera {k} to location {camera_config} (legacy format)")
        else:
            logger.error(f"Invalid camera config format for serial {k}: {camera_config}")
            raise Exception(f"Invalid camera config format for serial {k}: expected dict with 'location' field or string")

    logger.info(f"Configuration mapping completed for {len(result)} cameras")
    return result


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
        'log_level',
        default_value='info',
        description='Logging level for all nodes (debug, info, warn, error, fatal)'
    )
    measurement_mode_arg = DeclareLaunchArgument(
        'measurement_mode',
        default_value='false',
        description='Enable timing measurement and CSV logging for apriltags node.'
    )
    timing_csv_path_arg = DeclareLaunchArgument(
        'timing_csv_path',
        default_value='',
        description='Path to timing CSV file for apriltags node.'
    )
    
    try:
        # First we scan for cameras.
        cameras_by_serial_id = scan_for_cameras()
        logger.info(f"Detected cameras: {cameras_by_serial_id}")

        cameras_by_location = get_config_data(cameras_by_serial_id)
        logger.info(f"Camera location mapping: {cameras_by_location}")

        # For each camera found, set up the image processing pipeline.
        nodes = [log_level_arg, measurement_mode_arg, timing_csv_path_arg]
        
        # Add launch info messages
        nodes.append(LogInfo(msg=f"Launching vision system with {len(cameras_by_serial_id)} cameras"))
        
        node_count = 0
        for serial_id, camera_idx in cameras_by_serial_id.items():
            cam_location = cameras_by_location[serial_id]
            logger.info(f"Setting up nodes for camera {serial_id} at location '{cam_location}'")
            
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
                    },
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
            nodes.append(camera_node)
            node_count += 1
            logger.debug(f"Added USB camera node for serial {serial_id}")
            
            # AprilTags Node
            apriltags_node = Node(
                package="apriltags_cuda",
                executable="apriltags_cuda_node",
                name=f"apriltags_{serial_id}",
                parameters=[{
                    "topic_name": f"cameras/{cam_location}/image_raw",
                    "camera_serial": serial_id,
                    "publish_images_to_topic": f"apriltags/{cam_location}/images",
                    "publish_pose_to_topic": f"apriltags/{cam_location}/pose",
                    "measurement_mode": LaunchConfiguration('measurement_mode'),
                    "timing_csv_path": LaunchConfiguration('timing_csv_path'),
                }],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
            nodes.append(apriltags_node)
            node_count += 1
            logger.debug(f"Added AprilTags node for serial {serial_id}")

        # Add the foxglove bridge.
        foxglove_node = Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output="screen",
        )
        nodes.append(foxglove_node)
        node_count += 1
        logger.info("Added Foxglove bridge node")

        logger.info(f"Launch configuration completed successfully with {node_count} nodes")
        logger.info("=== ROS Vision Launch Configuration Complete ===")
        
        return LaunchDescription(nodes)
        
    except Exception as e:
        logger.error(f"Failed to generate launch description: {str(e)}")
        logger.error("=== ROS Vision Launch Configuration Failed ===")
        raise
