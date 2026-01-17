#!/usr/bin/env python3
"""
Utility to extract images from ROS2 bag files.

This script reads ROS2 bag files and extracts image messages to disk as PNG files.
Supports both compressed and raw image topics.
"""

import argparse
import os
import sys
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def deserialize_image(msg) -> Optional[np.ndarray]:
    """
    Deserialize a ROS Image message to numpy array.
    
    Args:
        msg: ROS Image message
        
    Returns:
        numpy array or None if deserialization fails
    """
    try:
        # Get image properties
        height = msg.height
        width = msg.width
        encoding = msg.encoding
        
        # Convert data to numpy array
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        
        # Handle different encodings
        if encoding == 'bgr8':
            img = img_data.reshape((height, width, 3))
        elif encoding == 'rgb8':
            img = img_data.reshape((height, width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif encoding == 'mono8' or encoding == '8UC1':
            img = img_data.reshape((height, width))
        elif encoding == 'bgra8':
            img = img_data.reshape((height, width, 4))
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        elif encoding == 'rgba8':
            img = img_data.reshape((height, width, 4))
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif encoding == '16UC1':
            img = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
            # Convert to 8-bit for saving as PNG
            img = (img / 256).astype(np.uint8)
        else:
            print(f"Warning: Unsupported encoding '{encoding}', skipping")
            return None
            
        return img
    except Exception as e:
        print(f"Error deserializing image: {e}")
        return None


def create_metadata_if_missing(bag_path: str):
    """
    Create a minimal metadata.yaml file if it's missing from a bag directory.
    This allows opening bags that may have lost their metadata.
    
    Args:
        bag_path: Path to the ROS2 bag directory
    """
    import glob
    import yaml
    
    metadata_path = Path(bag_path) / "metadata.yaml"
    
    if metadata_path.exists():
        return  # Metadata already exists
    
    print("Warning: metadata.yaml not found, creating minimal metadata file...")
    
    # Find all db3 files in the directory
    db3_files = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
    
    if not db3_files:
        raise FileNotFoundError(f"No db3 files found in {bag_path}")
    
    # Create minimal metadata
    relative_paths = [os.path.basename(f) for f in db3_files]
    
    metadata = {
        'rosbag2_bagfile_information': {
            'version': 5,
            'storage_identifier': 'sqlite3',
            'relative_file_paths': relative_paths,
            'duration': {
                'nanoseconds': 0
            },
            'starting_time': {
                'nanoseconds_since_epoch': 0
            },
            'message_count': 0,
            'topics_with_message_count': [],
            'compression_format': '',
            'compression_mode': ''
        }
    }
    
    # Write metadata file
    with open(metadata_path, 'w') as f:
        yaml.dump(metadata, f, default_flow_style=False)
    
    print(f"Created metadata file: {metadata_path}")


def extract_images_from_bag(
    bag_path: str,
    output_dir: str,
    topic_filter: Optional[str] = None,
    max_images: Optional[int] = None,
    skip_frames: int = 1,
):
    """
    Extract images from a ROS2 bag file.
    
    Args:
        bag_path: Path to the ROS2 bag directory
        output_dir: Directory to save extracted images
        topic_filter: Only extract from topics matching this string (e.g., 'camera', 'image_raw')
        max_images: Maximum number of images to extract per topic
        skip_frames: Extract every Nth frame (1 = all frames, 2 = every other frame, etc.)
    """
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Create metadata if missing
    create_metadata_if_missing(bag_path)
    
    # Create typestore for ROS2 message types
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    print(f"Opening bag: {bag_path}")
    
    # Open bag and get topic information
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        # Get all connections (topics)
        connections = [x for x in reader.connections]
        
        # Filter for image topics
        image_topics = [
            conn for conn in connections
            if 'sensor_msgs/msg/Image' in conn.msgtype or 
               'sensor_msgs/msg/CompressedImage' in conn.msgtype
        ]
        
        # Apply topic filter if specified
        if topic_filter:
            image_topics = [
                conn for conn in image_topics
                if topic_filter in conn.topic
            ]
        
        if not image_topics:
            print("No image topics found in bag!")
            if topic_filter:
                print(f"Filter: '{topic_filter}'")
            print("\nAvailable topics:")
            for conn in connections:
                print(f"  - {conn.topic} ({conn.msgtype})")
            return
        
        print(f"\nFound {len(image_topics)} image topic(s):")
        for conn in image_topics:
            print(f"  - {conn.topic} ({conn.msgtype})")
        
        # Track statistics per topic
        stats = {conn.topic: {'extracted': 0, 'skipped': 0, 'errors': 0} 
                for conn in image_topics}
        
        # Extract images
        print(f"\nExtracting images (skip_frames={skip_frames})...")
        
        frame_counters = {conn.topic: 0 for conn in image_topics}
        
        for connection, timestamp, rawdata in reader.messages(
            connections=image_topics
        ):
            topic = connection.topic
            
            # Check if we should skip this frame
            frame_counters[topic] += 1
            if frame_counters[topic] % skip_frames != 0:
                stats[topic]['skipped'] += 1
                continue
            
            # Check if we've reached the max for this topic
            if max_images and stats[topic]['extracted'] >= max_images:
                continue
            
            try:
                # Deserialize message
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                
                # Create topic-specific subdirectory
                topic_safe = topic.replace('/', '_').strip('_')
                topic_dir = output_path / topic_safe
                topic_dir.mkdir(exist_ok=True)
                
                # Handle compressed images
                if 'CompressedImage' in connection.msgtype:
                    # Decode compressed image
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if img is None:
                        print(f"Failed to decode compressed image from {topic}")
                        stats[topic]['errors'] += 1
                        continue
                else:
                    # Handle raw image
                    img = deserialize_image(msg)
                    if img is None:
                        stats[topic]['errors'] += 1
                        continue
                
                # Generate filename with timestamp
                timestamp_sec = timestamp / 1e9  # Convert nanoseconds to seconds
                filename = f"{topic_safe}_{timestamp_sec:.6f}.png"
                output_file = topic_dir / filename
                
                # Save image
                cv2.imwrite(str(output_file), img)
                stats[topic]['extracted'] += 1
                
                # Print progress
                if stats[topic]['extracted'] % 10 == 0:
                    print(f"  {topic}: {stats[topic]['extracted']} images extracted")
                    
            except Exception as e:
                print(f"Error processing message from {topic}: {e}")
                stats[topic]['errors'] += 1
    
    # Print summary
    print("\n" + "="*60)
    print("EXTRACTION COMPLETE")
    print("="*60)
    for topic, counts in stats.items():
        print(f"\n{topic}:")
        print(f"  Extracted: {counts['extracted']}")
        print(f"  Skipped:   {counts['skipped']}")
        print(f"  Errors:    {counts['errors']}")
    
    total_extracted = sum(s['extracted'] for s in stats.values())
    print(f"\nTotal images extracted: {total_extracted}")
    print(f"Output directory: {output_dir}")


def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description="Extract images from ROS2 bag files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Extract all images from a bag
  extract_images /path/to/bag/directory -o ./extracted_images
  
  # Extract only from topics containing 'center_front'
  extract_images /path/to/bag -o ./output -t center_front
  
  # Extract every 10th frame, max 100 images per topic
  extract_images /path/to/bag -o ./output -s 10 -m 100
  
  # Extract from specific topic pattern
  extract_images /path/to/bag -o ./output -t image_raw
        """
    )
    
    parser.add_argument(
        'bag_path',
        type=str,
        help='Path to ROS2 bag directory'
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default='./extracted_images',
        help='Output directory for extracted images (default: ./extracted_images)'
    )
    
    parser.add_argument(
        '-t', '--topic-filter',
        type=str,
        default=None,
        help='Only extract from topics containing this string'
    )
    
    parser.add_argument(
        '-m', '--max-images',
        type=int,
        default=None,
        help='Maximum number of images to extract per topic'
    )
    
    parser.add_argument(
        '-s', '--skip-frames',
        type=int,
        default=1,
        help='Extract every Nth frame (default: 1, meaning all frames)'
    )
    
    args = parser.parse_args()
    
    # Validate bag path
    if not os.path.exists(args.bag_path):
        print(f"Error: Bag path does not exist: {args.bag_path}")
        sys.exit(1)
    
    if not os.path.isdir(args.bag_path):
        print(f"Error: Bag path is not a directory: {args.bag_path}")
        sys.exit(1)
    
    # Extract images
    try:
        extract_images_from_bag(
            bag_path=args.bag_path,
            output_dir=args.output,
            topic_filter=args.topic_filter,
            max_images=args.max_images,
            skip_frames=args.skip_frames,
        )
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
