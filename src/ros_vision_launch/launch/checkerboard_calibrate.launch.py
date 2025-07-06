import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_vision_launch.utils import scan_for_cameras, get_config_data

def launch_setup(context, *args, **kwargs):
    serial = LaunchConfiguration('serial').perform(context)
    max_frames = int(LaunchConfiguration('max_frames').perform(context))
    board_cols = int(LaunchConfiguration('board_cols').perform(context))
    board_rows = int(LaunchConfiguration('board_rows').perform(context))
    square_length = float(LaunchConfiguration('square_length').perform(context))

    cameras_by_serial = scan_for_cameras()
    camera_positions = get_config_data(cameras_by_serial)

    if serial not in cameras_by_serial:
        raise Exception(f"Camera serial '{serial}' not found. Available: {list(cameras_by_serial.keys())}")

    camera_idx = cameras_by_serial[serial]
    cam_location = camera_positions.get(serial, "unknown")
    camera_topic = f"cameras/{cam_location}/image_raw"

    return [
        Node(
            package='usb_camera',
            executable='usb_camera_node',
            name='camera_node',
            parameters=[{
                "camera_idx": camera_idx,
                "camera_serial": serial,
                "topic_name": camera_topic,
                "approx_fps": 10,  # Run capture at a slower rate for calibration
            }]
        ),
        Node(
            package='checkerboard_camera_calibration',
            executable='checkerboard_camera_calibrator',
            name='checkerboard_camera_calibrator',
            parameters=[{
                "subscriber_topic": camera_topic,
                "publisher_topic": f"/calibration/{cam_location}/image_annotated",
                "camera_serial": serial,
                "max_frames": max_frames,
                "board_cols": board_cols,
                "board_rows": board_rows,
                "square_length": square_length
            }]
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial",
            description="Camera serial number to calibrate"
        ),
        DeclareLaunchArgument(
            "max_frames",
            description="Number of frames to collect before calibrating",
            default_value="30",
        ),
        DeclareLaunchArgument(
            "board_cols",
            description="Number of columns the checkerboard has",
            default_value="9",
        ),
        DeclareLaunchArgument(
            "board_rows",
            description="Number of rows the checkerboard has",
            default_value="6",
        ),
        DeclareLaunchArgument(
            "square_length",
            description="Physical size of the checkerboard squares",
            default_value="0.0254",
        ),
        OpaqueFunction(function=launch_setup)
    ])
