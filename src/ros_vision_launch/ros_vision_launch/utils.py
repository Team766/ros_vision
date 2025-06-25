import json
import os
import re

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
        if "Camera" not in entry or not entry.endswith("index0"):
            continue

        # Extract the serial number from the symlink name
        match = re.search(r"Camera_(\d+)", entry)
        if not match:
            continue  # Or raise error, up to you

        serial = match.group(1)

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
        msg += "This utility depends on finding the string 'Camera' somewhere in the filename.\n"
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

