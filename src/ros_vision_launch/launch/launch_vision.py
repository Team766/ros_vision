import json
import os
import re
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

BY_ID_PATH = "/dev/v4l/by-id/"
"""Path to the directory containing persistent V4L device symlinks by hardware ID."""

class CameraData:
    def __init__(self, json_data, serial_number):
        self.serial_number = serial_number
        self.location = json_data["location"]
        # Add more fields as needed, e.g. width, height, frame_rate
        self.width = json_data.get("width")
        self.height = json_data.get("height")
        self.frame_rate = json_data.get("frame_rate")


def scan_for_cameras():
    """
    For each camera, find the /dev/video* device associated with its -index0 symlink,
    and map the serial number (last underscore-separated group before -video-index0)
    to the actual video index (as integer).
    """
    if not os.path.exists(BY_ID_PATH):
        raise Exception(f"Error: {BY_ID_PATH} does not exist.")

    cameras_by_serial_id = {}
    for entry in os.listdir(BY_ID_PATH):
        # Only consider symlinks ending in -index0 (main video stream)
        if not entry.endswith("index0"):
            continue

        # Extract the serial number as the last group before -video-index0
        match = re.search(r"_([^_]+)-video-index0$", entry)
        if not match:
            continue  # Or raise error, up to you

        serial = match.group(1).replace(".", "")

        # Resolve symlink to get the /dev/video* device path
        device_path = os.path.realpath(os.path.join(BY_ID_PATH, entry))

        # Robustly extract the number from /dev/videoN
        vid_match = re.search(r"/dev/video(\d+)$", device_path)
        if not vid_match:
            continue  # Or raise error, up to you

        video_index = int(vid_match.group(1))
        cameras_by_serial_id[serial] = video_index

    if not cameras_by_serial_id:
        msg = "\n\nError: No camera devices found in by-id entries!\n\n"
        msg += f"We found these devices: {list(os.listdir(BY_ID_PATH))}\n\n"
        msg += "To debug, try listing the contents of /dev/v4l/by-id and confirming that your cameras are present.\n"
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
        result[k] = CameraData(camera_mounted_positions[k], k)
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

    cameras = get_config_data(cameras_by_serial_id)
    print(f"Found camera data: {cameras}")

    nodes = []
    for serial_id, camera_idx in cameras_by_serial_id.items():
        cam_data = cameras[serial_id]
        cam_location = cam_data.location
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
                        "publish_to_topic": f"apriltags/{cam_location}/images",
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

