from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="usb_camera",
                executable="usb_camera_node",
                name=f"camera_0",
                parameters=[
                    {"camera_idx": 0, "topic_name": "camera/image_raw"},
                ],
            ),
            Node(
                package="apriltags_cuda",
                executable="apriltags_cuda_node",
                name=f"apriltags",
                parameters=[
                    {"topic_name": "camera/image_raw"},
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
            ),
        ]
    )
