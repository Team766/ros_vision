import json
import os
import re
import logging

from ament_index_python.packages import get_package_share_directory

# Configure logging for utils
logger = logging.getLogger("ros_vision_launch.utils")

BY_ID_PATH = "/dev/v4l/by-id/"
"""Path to the directory containing persistent V4L device symlinks by hardware ID."""

BY_PATH_PATH = "/dev/v4l/by-path/"
"""Path to the directory containing persistent V4L device symlinks by USB path."""


def _resolve_video_index(directory, entry):
    """
    Resolve a symlink in a /dev/v4l/ directory to its /dev/videoN index.

    Args:
        directory: The directory containing the symlink.
        entry: The symlink filename.

    Returns:
        int or None: The video index, or None if it couldn't be resolved.
    """
    device_path = os.path.realpath(os.path.join(directory, entry))
    vid_match = re.search(r"/dev/video(\d+)$", device_path)
    if not vid_match:
        logger.warning(
            f"Could not extract video index from {device_path}, skipping"
        )
        return None
    return int(vid_match.group(1))


def scan_by_id():
    """
    Scan /dev/v4l/by-id/ for cameras and return a mapping of serial numbers
    to video indices.

    Returns:
        dict: Mapping of {serial_number: video_index}. If multiple physical
            cameras share the same serial, only one will appear (the last one
            found by the filesystem listing).
    """
    if not os.path.exists(BY_ID_PATH):
        logger.info(f"{BY_ID_PATH} does not exist, skipping by-id scan")
        return {}

    logger.debug(f"Scanning directory: {BY_ID_PATH}")
    cameras = {}
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
            # Try Arducam pattern: extract the serial from Camera_xxx or USB_Camera_xxx
            arducam_match = re.search(r"(?:USB_)?Camera_([a-zA-Z0-9]+)", entry)
            if arducam_match:
                serial = arducam_match.group(1)
                logger.debug(f"Extracted serial using Arducam pattern: {serial}")
            else:
                # Generic fallback: extract any alphanumeric identifier before -video-index0
                generic_match = re.search(r"([a-zA-Z0-9]+)-video-index0$", entry)
                if generic_match:
                    serial = generic_match.group(1)
                    logger.debug(f"Extracted serial using generic pattern: {serial}")
                else:
                    logger.warning(f"Could not extract serial from {entry}, skipping")
                    continue

        video_index = _resolve_video_index(BY_ID_PATH, entry)
        if video_index is None:
            continue

        if serial in cameras:
            logger.warning(
                f"Duplicate serial '{serial}' in by-id (video{cameras[serial]} and "
                f"video{video_index}). This camera will be identified by USB path instead."
            )
        cameras[serial] = video_index
        logger.info(
            f"Found camera (by-id): serial={serial}, video_index={video_index}, device={entry}"
        )

    logger.info(f"by-id scan found {len(cameras)} camera(s)")
    return cameras


def scan_by_path():
    """
    Scan /dev/v4l/by-path/ for cameras and return a mapping of path
    identifiers to video indices.

    The identifier is the by-path symlink name with "-video-index0" stripped.
    For example:
        platform-3610000.usb-usb-0:2:1.0-video-index0
            → "platform-3610000.usb-usb-0:2:1.0"
        platform-3610000.usb-usb-0:3.1:1.0-video-index0
            → "platform-3610000.usb-usb-0:3.1:1.0"

    To find the config key for a camera: ls /dev/v4l/by-path/, find the
    -video-index0 entry, and remove the "-video-index0" suffix.

    Returns:
        dict: Mapping of {path_identifier: video_index}.
    """
    if not os.path.exists(BY_PATH_PATH):
        logger.info(f"{BY_PATH_PATH} does not exist, skipping by-path scan")
        return {}

    logger.debug(f"Scanning directory: {BY_PATH_PATH}")
    cameras = {}
    entries = os.listdir(BY_PATH_PATH)
    logger.info(f"Found {len(entries)} entries in {BY_PATH_PATH}")

    VIDEO_INDEX0_SUFFIX = "-video-index0"

    for entry in entries:
        if not entry.endswith(VIDEO_INDEX0_SUFFIX):
            continue

        path_id = entry[: -len(VIDEO_INDEX0_SUFFIX)]

        video_index = _resolve_video_index(BY_PATH_PATH, entry)
        if video_index is None:
            continue

        cameras[path_id] = video_index
        logger.info(
            f"Found camera (by-path): id={path_id}, video_index={video_index}"
        )

    logger.info(f"by-path scan found {len(cameras)} camera(s)")
    return cameras


def scan_for_cameras():
    """
    Scan for cameras using both /dev/v4l/by-id/ and /dev/v4l/by-path/.

    Cameras with unique serial numbers are identified by serial (backward
    compatible). Cameras that share a serial number (only one entry in by-id
    but multiple physical devices) are identified by their USB port shorthand
    from by-path.

    Returns:
        dict: Mapping of {identifier: video_index} where identifier is either
            a serial number string or a by-path identifier
            (e.g. "platform-3610000.usb-usb-0:2:1.0").
    """
    logger.info("Starting camera scan...")

    by_id_cameras = scan_by_id()
    by_path_cameras = scan_by_path()

    # Start with all by-id cameras
    result = dict(by_id_cameras)
    covered_indices = set(by_id_cameras.values())

    # Add any by-path cameras whose video index isn't already covered by by-id.
    # These are cameras that share a serial number with another camera, so
    # by-id only has one entry for them.
    for port, video_index in by_path_cameras.items():
        if video_index not in covered_indices:
            result[port] = video_index
            covered_indices.add(video_index)
            logger.info(
                f"Camera at video{video_index} not found in by-id, "
                f"using USB port identifier: {port}"
            )

    if not result:
        error_msg = "\n\nError: No camera devices found!\n\n"
        error_msg += "Scanned both /dev/v4l/by-id/ and /dev/v4l/by-path/.\n"
        if os.path.exists(BY_ID_PATH):
            error_msg += f"by-id devices: {list(os.listdir(BY_ID_PATH))}\n"
        else:
            error_msg += f"{BY_ID_PATH} does not exist.\n"
        if os.path.exists(BY_PATH_PATH):
            error_msg += f"by-path devices: {list(os.listdir(BY_PATH_PATH))}\n"
        else:
            error_msg += f"{BY_PATH_PATH} does not exist.\n"
        error_msg += "\nFor by-id detection, this utility looks for 'Camera' or 'camera' in the device filename.\n"
        error_msg += "With an arducam, try resetting the serial number: https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/"
        logger.error(error_msg)
        raise Exception(error_msg)

    logger.info(f"Camera scan completed. Found {len(result)} camera(s): {result}")
    return result


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
        cameras_by_serial_id (dict): Mapping of camera identifiers (serial numbers
            or USB port shorthands) to video indices (int).

    Returns:
        dict: Mapping of camera identifiers (str) to their camera locations from config.

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
    logger.debug(
        f"Available camera positions in config: {list(camera_mounted_positions.keys())}"
    )

    result = {}
    for k, v in cameras_by_serial_id.items():
        if k not in camera_mounted_positions:
            logger.error(f"Camera identifier {k} not found in configuration")
            raise Exception(
                f"Camera identifier '{k}' not found in camera_mounted_positions configuration. "
                f"Available keys: {list(camera_mounted_positions.keys())}"
            )

        # Extract the location from the camera config object
        camera_config = camera_mounted_positions[k]
        if isinstance(camera_config, dict) and "location" in camera_config:
            # New format: camera config is an object with location field
            result[k] = camera_config["location"]
            logger.debug(
                f"Mapped camera {k} to location {camera_config['location']} (new format)"
            )
        elif isinstance(camera_config, str):
            # Old format: camera config is just a location string (backward compatibility)
            result[k] = camera_config
            logger.debug(
                f"Mapped camera {k} to location {camera_config} (legacy format)"
            )
        else:
            logger.error(
                f"Invalid camera config format for identifier {k}: {camera_config}"
            )
            raise Exception(
                f"Invalid camera config format for identifier {k}: expected dict with 'location' field or string"
            )

    logger.info(f"Configuration mapping completed for {len(result)} cameras")
    return result
