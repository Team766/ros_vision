import json
import os
import re
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

BY_ID_PATH = "/dev/v4l/by-id/"
"""Path to the directory containing persistent V4L device symlinks by hardware ID."""


def scan_for_cameras():
    """
    For each camera, find the /dev/video* device associated with its -index0 symlink,
    and map the serial number to the actual video index (as integer).
    """
    if not os.path.exists(BY_ID_PATH):
        raise Exception(f"Error: {BY_ID_PATH} does not exist.")

    cameras_by_serial_id = {}
    for entry in os.listdir(BY_ID_PATH):
        # Only consider symlinks ending in -index0 (main video stream)
        if not entry.endswith("index0"):
            continue
            
        # Check if this is a camera device (broader pattern matching)
        if "Camera" not in entry and "camera" not in entry.lower():
            continue

        # Extract the serial number from the symlink name
        # First try the original pattern for devices with Camera_(\d+)
        match = re.search(r"Camera_(\d+)", entry)
        if match:
            serial = match.group(1)
        else:
            # Try Arducam pattern: extract the serial from UC### or similar
            arducam_match = re.search(r"USB_Camera_([A-Z0-9]+)", entry)
            if arducam_match:
                serial = arducam_match.group(1)
            else:
                # Generic fallback: extract any alphanumeric identifier before -video-index0
                generic_match = re.search(r"([A-Z0-9]+)-video-index0$", entry)
                if generic_match:
                    serial = generic_match.group(1)
                else:
                    print(f"Warning: Could not extract serial from {entry}, skipping")
                    continue

        # Resolve symlink to get the /dev/video* device path
        device_path = os.path.realpath(os.path.join(BY_ID_PATH, entry))

        # Robustly extract the number from /dev/videoN
        vid_match = re.search(r"/dev/video(\d+)$", device_path)
        if not vid_match:
            print(f"Warning: Could not extract video index from {device_path}, skipping")
            continue

        video_index = int(vid_match.group(1))
        cameras_by_serial_id[serial] = video_index
        print(f"Found camera: serial={serial}, video_index={video_index}, device={entry}")

    if not cameras_by_serial_id:
        msg = "\n\nError: No camera devices found in by-id entries!\n\n"
        msg += "This utility depends on finding the string 'Camera' or 'camera' somewhere in the filename.\n"
        msg += f"We found these devices: {list(os.listdir(BY_ID_PATH))}\n\n"
        msg += "To debug, try listing the contents of /dev/v4l/by-id and confirming that Camera is in the device filename.\n"
        msg += "With an arducam, try resetting the serial number so that the Device Name is 'Camera': https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/"
        raise Exception(msg)

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
    # TODO: implement this
    pass


def get_config_data(cameras_by_serial_id):
    """
    Load and validate camera configuration data from the ROS package.

    Args:
        cameras_by_serial_id (dict): Mapping of camera serials (str) to video indices (int).

    Returns:
        dict: Mapping of camera serial numbers (str) to their mounted positions from config.

    Raises:
        Exception: If the configuration file is not found or required keys are missing.
    """
    data_path = os.path.join(
        get_package_share_directory("vision_config_data"), "data", "system_config.json"
    )
    if not os.path.exists(data_path):
        raise Exception(f"Error: unable to find system_config.json at {data_path}")

    with open(data_path, "r") as f:
        config_data = json.load(f)

    check_format(config_data)

    camera_mounted_positions = config_data["camera_mounted_positions"]
    result = {}
    for k, v in cameras_by_serial_id.items():
        result[k] = camera_mounted_positions[k]

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
    # First we scan for cameras.
    cameras_by_serial_id = scan_for_cameras()
    print(f"Found cameras: {cameras_by_serial_id}")

    cameras_by_location = get_config_data(cameras_by_serial_id)
    print(f"Found locations: {cameras_by_location}")

    # For each camera found, set up the image processing pipeline.
    nodes = []
    for serial_id, camera_idx in cameras_by_serial_id.items():
        cam_location = cameras_by_location[serial_id]
        nodes.append(
            Node(
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
            )
        )
        nodes.append(
            Node(
                package="apriltags_cuda",
                executable="apriltags_cuda_node",
                name=f"apriltags",
                parameters=[
                    {
                        "topic_name": f"cameras/{cam_location}/image_raw",
                        "camera_serial": serial_id,
                        "publish_images_to_topic": f"apriltags/{cam_location}/images",
                        "publish_pose_to_topic": f"apriltags/{cam_location}/pose",
                    },
                ],
            )
        )

    # Add the foxglove bridge.
    nodes.append(
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
        )
    )

    return LaunchDescription(nodes)
