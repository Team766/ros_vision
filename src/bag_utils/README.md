# Bag Utils

Utilities for working with ROS2 bag files.

## Tools

### extract_images

Extract images from ROS2 bag files to PNG format.

#### Installation

```bash
# Build the package
colcon build --packages-select bag_utils

# Source the environment
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### Usage

```bash
# Extract all images from a bag
extract_images /path/to/bag/directory -o ./extracted_images

# Extract only from topics containing 'center_front'
extract_images /path/to/bag -o ./output -t center_front

# Extract every 10th frame, max 100 images per topic
extract_images /path/to/bag -o ./output -s 10 -m 100

# Extract from specific topic pattern
extract_images /path/to/bag -o ./output -t image_raw
```

#### Options

- `-o, --output`: Output directory for extracted images (default: ./extracted_images)
- `-t, --topic-filter`: Only extract from topics containing this string
- `-m, --max-images`: Maximum number of images to extract per topic
- `-s, --skip-frames`: Extract every Nth frame (default: 1, all frames)

#### Output Structure

Images are organized by topic:
```
extracted_images/
├── cameras_center_front_image_raw/
│   ├── cameras_center_front_image_raw_1234567890.123456.png
│   ├── cameras_center_front_image_raw_1234567890.456789.png
│   └── ...
├── cameras_left_side_image_raw/
│   └── ...
└── ...
```

Filenames include the topic name and timestamp for easy tracking.
