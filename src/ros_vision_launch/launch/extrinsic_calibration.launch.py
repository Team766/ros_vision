#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for extrinsic calibration data collection.
    
    Starts:
    - data_collector node (automatically finds cameras and publishes images)
    - foxglove_bridge (for visualization in Foxglove Studio)
    
    Launch arguments:
    - capture_length_seconds: How long to collect data (default: 60.0)
    - frame_rate_hz: Frame capture rate (default: 0.5)
    - output_dir: Where to save calibration data (default: "./calibration_data")
    """
    
    # Declare launch arguments
    capture_length_arg = DeclareLaunchArgument(
        'capture_length_seconds',
        default_value='60.0',
        description='How long to collect calibration data in seconds'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate_hz',
        default_value='2.0',
        description='Frame capture rate in Hz'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='./calibration_data',
        description='Directory where calibration data will be saved'
    )
    
    # Data collector node
    data_collector = Node(
        package='extrinsic_calibration',
        executable='data_collector',
        name='data_collector',
        output='screen',
        parameters=[
            {'capture_length_seconds': LaunchConfiguration('capture_length_seconds')},
            {'frame_rate_hz': LaunchConfiguration('frame_rate_hz')},
            {'output_dir': LaunchConfiguration('output_dir')}
        ]
    )
    
    # Foxglove bridge for visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            {'port': 8765},
            {'address': '0.0.0.0'},
            {'tls': False},
            {'topic_whitelist': ['.*']},
            {'send_buffer_limit': 10000000},
            {'use_compression': False}
        ]
    )
    
    return LaunchDescription([
        capture_length_arg,
        frame_rate_arg,
        output_dir_arg,
        data_collector,
        foxglove_bridge,
    ])