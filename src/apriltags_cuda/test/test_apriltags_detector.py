#!/usr/bin/env python3

import os
import json
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
import launch_testing
from launch_ros.actions import Node as LaunchNode
import threading
import time


def get_camera_serial_and_location():
    """Get a valid camera serial from system config for testing"""
    try:
        config_path = os.path.join(
            get_package_share_directory('vision_config_data'),
            'data', 'system_config.json'
        )
        with open(config_path, 'r') as f:
            config = json.load(f)
        # Use the first serial and its location in the config
        serial = next(iter(config['camera_mounted_positions'].keys()))
        location = config['camera_mounted_positions'][serial]['location']
        return serial, location
    except Exception:
        # Fallback to default values for testing
        return "766", "right_front"


def get_test_image_path():
    """Get path to test image containing AprilTags"""
    return os.path.join(
        get_package_share_directory('apriltags_cuda'),
        'test', 'data', 'colorimage.jpg'
    )


def generate_test_description():
    """Launch configuration for the integration test"""
    camera_serial, location = get_camera_serial_and_location()
    image_path = get_test_image_path()
    input_topic = f'/cameras/{location}/image_raw'
    output_topic = f'/apriltags/{location}/images'

    # Launch the apriltags detector node
    apriltags_node = LaunchNode(
        package='apriltags_cuda',
        executable='apriltags_cuda_node',
        name='apriltags_cuda_node',
        parameters=[{
            'topic_name': input_topic,
            'camera_serial': f'"{camera_serial}"',
            'publish_to_topic': output_topic,
        }],
        output='screen',
    )
    
    # Launch the test image publisher
    image_pub_node = LaunchNode(
        package='apriltags_cuda',
        executable='test_image_publisher.py',
        name='test_image_publisher',
        parameters=[{
            'image_path': image_path,
            'topic_name': input_topic,
            'publish_rate': 2.0,  # Publish at 2 Hz
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        apriltags_node,
        image_pub_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestApriltagsDetector:
    @pytest.mark.launch_test
    def test_apriltags_integration(self, launch_service, proc_info, proc_output):
        """Integration test for AprilTag detection pipeline"""
        
        rclpy.init()
        
        try:
            # Get test configuration
            camera_serial, location = get_camera_serial_and_location()
            output_topic = f'/apriltags/{location}/images'
            
            received_images = []
            test_complete = threading.Event()
            
            class DetectionListener(Node):
                def __init__(self):
                    super().__init__('detection_listener')
                    self.subscription = self.create_subscription(
                        Image,
                        output_topic,
                        self.image_callback,
                        10
                    )
                    self.get_logger().info(f'Listening for detection results on: {output_topic}')
                
                def image_callback(self, msg):
                    """Callback for processed detection images"""
                    self.get_logger().info(f'Received detection image: {msg.width}x{msg.height}')
                    received_images.append(msg)
                    
                    # Complete test after receiving first processed image
                    if len(received_images) >= 1:
                        test_complete.set()

            # Create listener node
            listener = DetectionListener()
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(listener)

            # Spin in a separate thread
            def spin_executor():
                while rclpy.ok() and not test_complete.is_set():
                    try:
                        executor.spin_once(timeout_sec=0.1)
                    except Exception:
                        break

            spin_thread = threading.Thread(target=spin_executor)
            spin_thread.start()

            # Wait for test completion (max 30 seconds for colcon)
            test_passed = test_complete.wait(timeout=30.0)
            
            # Clean up
            executor.remove_node(listener)
            listener.destroy_node()
            spin_thread.join(timeout=1.0)

            # Verify results
            print(f'Test completed. Received {len(received_images)} processed images.')
            
            assert test_passed, f'Timeout: No processed images received on {output_topic} within 30 seconds'
            assert len(received_images) > 0, f'No detection images received on {output_topic}'
            
            # Verify image properties
            first_image = received_images[0]
            assert first_image.width > 0, 'Received image has invalid width'
            assert first_image.height > 0, 'Received image has invalid height'
            assert first_image.encoding == 'bgr8', f'Expected bgr8 encoding, got {first_image.encoding}'
            
            print(f'✓ Integration test passed: AprilTag detector processed {len(received_images)} images')
            
        finally:
            if rclpy.ok():
                rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput:
    def test_exit_codes(self, proc_info):
        """Verify that all processes exit cleanly"""
        launch_testing.asserts.assertExitCodes(proc_info)
