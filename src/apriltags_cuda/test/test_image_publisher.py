#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory


class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        
        # Declare parameters
        self.declare_parameter('image_path', '')
        self.declare_parameter('topic_name', '/cameras/test/image_raw')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        # Get parameters
        self.image_path = self.get_parameter('image_path').value
        self.topic_name = self.get_parameter('topic_name').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Set up publisher
        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Set up CV bridge
        self.bridge = CvBridge()
        
        # Load the test image
        self.load_test_image()
        
        self.get_logger().info(f'Publishing test image to: {self.topic_name}')
        self.get_logger().info(f'Image path: {self.image_path}')
        
    def load_test_image(self):
        """Load the test image from the specified path"""
        if not self.image_path:
            # Use default test image if no path specified
            self.image_path = os.path.join(
                get_package_share_directory('apriltags_cuda'),
                'test', 'data', 'colorimage.jpg'
            )
        
        if not os.path.exists(self.image_path):
            self.get_logger().error(f'Test image not found at: {self.image_path}')
            self.image = None
            return
            
        # Load image
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            self.get_logger().error(f'Failed to load image from: {self.image_path}')
        else:
            height, width = self.image.shape[:2]
            self.get_logger().info(f'Loaded test image: {width}x{height}')
    
    def timer_callback(self):
        """Publish the test image"""
        if self.image is None:
            self.get_logger().warn('No test image loaded, skipping publish')
            return
            
        # Convert OpenCV image to ROS message
        try:
            msg = self.bridge.cv2_to_imgmsg(self.image, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'test_camera'
            
            self.publisher.publish(msg)
            self.get_logger().debug('Published test image')
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors


if __name__ == '__main__':
    main()