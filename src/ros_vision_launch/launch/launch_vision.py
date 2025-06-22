import json
import os
import re
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ros_vision_launch.utils import scan_for_cameras, get_config_data


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
