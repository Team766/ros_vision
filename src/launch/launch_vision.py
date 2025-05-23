import os
import re
import sys

from launch import LaunchDescription
from launch_ros.actions import Node


BY_ID_PATH = "/dev/v4l/by-id/"


def scan_for_cameras():

    if not os.path.exists(BY_ID_PATH):
        raise Exception(f"Error: {BY_ID_PATH} does not exist.")

    found = False
    results = []

    for entry in os.listdir(BY_ID_PATH):
        # Example: usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_766-video-index0
        if "Camera" not in entry:
            continue  # Not a camera device
        found = True

        # Extract serial number and index using regex
        match = re.search(r"Camera_(\d+).*index(\d+)", entry)
        if not match:
            print(f"Error: Could not parse serial/index from: {entry}", file=sys.stderr)
            sys.exit(1)

        serial = match.group(1)
        index = match.group(2)

        # Follow symlink to get actual /dev/video* path
        device_path = os.path.realpath(os.path.join(BY_ID_PATH, entry))

        results.append(
            {
                "serial": serial,
                "index": index,
                "video_device": device_path,
            }
        )

    if not found:
        msg = "\n\nError: No camera devices found in by-id entries!\n\n"
        msg += "This utility depends on finding the string 'Camera' somewhere in the filename.\n"
        msg += f"We found these devices: {list(os.listdir(BY_ID_PATH))}\n\n"
        msg += "To debug, try listing the contents of /dev/v4l/by-id and confirming that Camera is in the device filename.\n"
        msg += "With an arducam, try resetting the serial number so that the Device Name is 'Camera': https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/"
        raise Exception(msg)

    cameras_by_serial_id = {
        item["serial"]: int(item["video_device"][-1]) for item in results
    }
    return cameras_by_serial_id


def generate_launch_description():

    # First we scan for cameras.
    cameras_by_serial_id = scan_for_cameras()
    print(f"Found cameras: {cameras_by_serial_id}")

    # For each camera found, set up the image processing pipeline.
    nodes = []
    for serial_id, camera_idx in cameras_by_serial_id.items():
        nodes.append(
            Node(
                package="usb_camera",
                executable="usb_camera_node",
                name=f"camera_{serial_id}",
                parameters=[
                    {
                        "camera_idx": camera_idx,
                        "topic_name": f"cameras/camera_{serial_id}/image_raw",
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
                        "topic_name": f"cameras/camera_{serial_id}/image_raw",
                        "publish_to_topic": f"apriltags/camera_{serial_id}/images",
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
