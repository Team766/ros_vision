#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
import json
import os
import time
import yaml
import argparse
import re
import logging
from typing import Dict, List, Any

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DataCollectorNode(Node):

    def __init__(self):
        super().__init__('data_collector')
        
        # Initialize CV bridge for image publishing
        self.bridge = CvBridge()
        self.image_publishers = {}
        
        # Declare parameters
        self.declare_parameter('output_dir', './calibration_data')
        self.declare_parameter('capture_length_seconds', 30.0)
        self.declare_parameter('frame_rate_hz', 1.0)
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.capture_length_seconds = self.get_parameter('capture_length_seconds').get_parameter_value().double_value
        self.frame_rate_hz = self.get_parameter('frame_rate_hz').get_parameter_value().double_value
        
        # Scan for cameras to get video index mapping
        try:
            from ros_vision_launch.utils import scan_for_cameras
            self.cameras_by_serial = scan_for_cameras()
            self.serial_ids = list(self.cameras_by_serial.keys())
            
            if not self.cameras_by_serial:
                self.get_logger().error('No cameras found! Please check camera connections and /dev/v4l/by-id/')
                raise Exception('No cameras detected')
                
            self.get_logger().info(f'Found cameras: {self.cameras_by_serial}')
            self.get_logger().info(f'Using serial IDs: {self.serial_ids}')
        except Exception as e:
            self.get_logger().error(f'Failed to scan for cameras: {e}')
            return
        
        # Load system configuration
        try:
            self.camera_configs = self._load_system_config()
            self.get_logger().info(f'Loaded camera configurations for {len(self.camera_configs)} cameras')
            
            # Verify all found cameras have configurations
            missing_configs = []
            for serial_id in self.serial_ids:
                if serial_id not in self.camera_configs:
                    missing_configs.append(serial_id)
                    
            if missing_configs:
                self.get_logger().error(f'Missing system configuration for cameras: {missing_configs}')
                raise Exception(f'Camera configurations not found for: {missing_configs}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load system configuration: {e}')
            return
            
        # Initialize cameras
        self.cameras = []
        self._setup_cameras()
        
        # Verify at least one camera was successfully initialized
        if not self.cameras:
            self.get_logger().error('No cameras were successfully initialized!')
            return
            
        self.get_logger().info(f'Successfully initialized {len(self.cameras)} cameras')
        
        # Setup output directory
        self._setup_output_directory()
        
        # Start data collection
        self._collect_frames()

    def _read_yaml_file(self, file_path: str) -> Dict[str, Any]:
        """Read YAML configuration file."""
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
            
    def _load_system_config(self) -> Dict[str, Any]:
        """Load system configuration from vision_config_data package."""
        from ament_index_python.packages import get_package_share_directory
        
        data_path = os.path.join(
            get_package_share_directory("vision_config_data"), "data", "system_config.json"
        )
        
        if not os.path.exists(data_path):
            raise Exception(f"System config file not found at {data_path}")
            
        with open(data_path, "r") as f:
            config_data = json.load(f)
            
        camera_mounted_positions = config_data["camera_mounted_positions"]
        
        # Extract camera configs for our serial IDs
        result = {}
        for serial_id in self.serial_ids:
            if serial_id not in camera_mounted_positions:
                raise Exception(f"Camera serial {serial_id} not found in system configuration")
            result[serial_id] = camera_mounted_positions[serial_id]
            
        return result

    def _setup_cameras(self):
        """Initialize camera capture devices using serial IDs."""
        for serial_id in self.serial_ids:
            if serial_id not in self.cameras_by_serial:
                raise Exception(f'Video index not found for camera {serial_id}')
                
            video_index = self.cameras_by_serial[serial_id]
            camera_config = self.camera_configs[serial_id]
            
            # Get camera properties from system config
            width = camera_config.get('width', 1280)
            height = camera_config.get('height', 800)
            
            # Get API preference if specified
            api_preference = cv2.CAP_ANY  # Default
            if 'api_preference' in camera_config:
                api_pref_str = camera_config['api_preference']
                api_preference_map = {
                    'V4L2': cv2.CAP_V4L2,
                    'GSTREAMER': cv2.CAP_GSTREAMER,
                    'FFMPEG': cv2.CAP_FFMPEG,
                    'ANY': cv2.CAP_ANY
                }
                if api_pref_str in api_preference_map:
                    api_preference = api_preference_map[api_pref_str]
                    self.get_logger().info(f'Using API preference: {api_pref_str}')
                else:
                    self.get_logger().warning(f'Unknown API preference: {api_pref_str}, using default')
            
            self.get_logger().info(f'Opening camera {serial_id} at video index {video_index} ({width}x{height})...')
            cap = cv2.VideoCapture(video_index, api_preference)
            
            if not cap.isOpened():
                raise Exception(f'Cannot open camera {serial_id} at video index {video_index}')
                
            # Set camera properties from system config
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # Set additional properties if available
            if 'format' in camera_config:
                fourcc = cv2.VideoWriter_fourcc(*camera_config['format'])
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            
            if 'frame_rate' in camera_config:
                cap.set(cv2.CAP_PROP_FPS, camera_config['frame_rate'])
            
            # Get camera location for topic name
            location = camera_config.get('location', serial_id)
            
            # Create image publisher for this camera
            topic_name = f'/cameras/{location}/image_raw'
            publisher = self.create_publisher(Image, topic_name, 10)
            self.image_publishers[serial_id] = publisher
            
            self.cameras.append({
                "name": serial_id,
                "serial_id": serial_id,
                "cap_device": cap,
                "config": camera_config,
                "location": location,
                "publisher": publisher
            })
            
            self.get_logger().info(f'Camera {serial_id} initialized successfully, publishing to {topic_name}')

    def _setup_output_directory(self):
        """Create output directory with timestamp."""
        from datetime import datetime
        
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.output_dir = os.path.join(self.output_dir, f"calibration_data_{timestamp}")
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            self.get_logger().info(f'Created output directory: {self.output_dir}')

    def _collect_frames(self):
        """Main frame collection loop."""
        start_time = time.perf_counter()
        end_time = start_time + self.capture_length_seconds
        image_sets = []
        image_cap_idx = 1
        
        self.get_logger().info('Starting frame collection...')
        
        while True:
            frame_cap_time = time.perf_counter()
            
            # Grab frames from all cameras simultaneously
            for cam in self.cameras:
                self.get_logger().info(f'Capturing {cam["name"]} frame {image_cap_idx}')
                success = cam["cap_device"].grab()
                if not success:
                    raise Exception(f'Unable to grab frame from camera {cam["name"]}')
            
            # Retrieve and save frames
            frames_in_set = {"frames": {}}
            for cam in self.cameras:
                success, frame = cam["cap_device"].retrieve()
                if not success:
                    raise Exception(f'Unable to retrieve frame from camera {cam["name"]}')
                
                # Publish image to ROS topic
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = f'camera_{cam["location"]}'
                    cam["publisher"].publish(ros_image)
                except Exception as e:
                    self.get_logger().warning(f'Failed to publish image for {cam["name"]}: {e}')
                    
                output_path = os.path.join(
                    self.output_dir, 
                    f'frame_{image_cap_idx}_{cam["name"]}.png'
                )
                
                if not cv2.imwrite(output_path, frame):
                    raise Exception(f'Error writing file {output_path}')
                    
                frames_in_set["frames"][cam["name"]] = output_path
                
            frames_in_set["cap_idx"] = image_cap_idx
            image_sets.append(frames_in_set)
            
            # Wait for next frame according to frame rate
            while True:
                now = time.perf_counter()
                if now - frame_cap_time >= 1 / self.frame_rate_hz:
                    break
                    
            image_cap_idx += 1
            
            # Check if capture duration has elapsed
            now = time.perf_counter()
            if now >= end_time:
                break
                
        self.get_logger().info('Frame collection completed!')
        
        # Release cameras
        for cam in self.cameras:
            cam["cap_device"].release()
            
        # Save frame set metadata
        output_path = os.path.join(self.output_dir, "framesets.json")
        with open(output_path, "w") as f:
            json.dump(image_sets, f, indent=4)
            
        self.get_logger().info(f'Saved frame metadata to {output_path}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DataCollectorNode()
        # Node completes its work in __init__, so just spin briefly
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()