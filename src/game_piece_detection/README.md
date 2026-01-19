# Game Piece Detection

YOLOv11-based game piece detection using TensorRT for FRC robotics. Supports dynamic model configuration for different detection tasks.

## Prerequisites

- TensorRT engine file (`.engine`) generated from a trained YOLOv11 model
- CUDA 11.8 and TensorRT 10.6+
- Python 3.8+ with `ultralytics` package (for model conversion)

## Configuration

The detection utilities read model settings from `system_config.json`. Add a `game_piece_detection` section:

```json
{
    "game_piece_detection": {
        "engine_file": "/path/to/model.engine",
        "input_channels": 3,
        "class_names": ["ball"]
    }
}
```

**Configuration fields:**
- `engine_file` - Path to the TensorRT engine file
- `input_channels` - Number of input channels (3 for RGB, 1 for grayscale)
- `class_names` - Array of class labels matching your model's training classes

The utilities automatically extract tensor dimensions (input size, number of predictions, number of classes) from the engine file at runtime.

## Model Conversion Pipeline

### 1. Convert PyTorch to ONNX

Use the Ultralytics library to export your trained YOLOv11 model to ONNX format:

```bash
# Install ultralytics if needed
pip install ultralytics

# Export to ONNX
python -c "
from ultralytics import YOLO
model = YOLO('your-model.pt')
model.export(format='onnx', imgsz=640, opset=12)
"
```

Or use the command-line interface:

```bash
yolo export model=your-model.pt format=onnx imgsz=640 opset=12
```

**Export options:**
- `imgsz` - Input image size (default: 640)
- `opset` - ONNX opset version (12 recommended for TensorRT compatibility)
- `simplify` - Simplify ONNX model (default: True)
- `dynamic` - Enable dynamic input shapes (not recommended for TensorRT)

### 2. Convert ONNX to TensorRT Engine

Convert your ONNX model to a TensorRT engine using `trtexec`:

```bash
# FP16 precision (recommended for speed/accuracy balance)
/usr/src/tensorrt/bin/trtexec \
    --onnx=your-model.onnx \
    --saveEngine=your-model.engine \
    --fp16

# FP32 precision (highest accuracy)
/usr/src/tensorrt/bin/trtexec \
    --onnx=your-model.onnx \
    --saveEngine=your-model.engine

# INT8 precision (fastest, requires calibration data)
/usr/src/tensorrt/bin/trtexec \
    --onnx=your-model.onnx \
    --saveEngine=your-model.engine \
    --int8 \
    --calib=calibration_data/
```

**Important:** Engine files are not portable between different CUDA/TensorRT versions or GPU architectures. Regenerate the engine on each target system.

## Running the Detection Test

Run inference on a directory of test images:

```bash
./install/game_piece_detection/lib/game_piece_detection/detection_test \
    --config <config_file> \
    --images <image_directory> \
    [options]
```

**Required arguments:**
- `--config FILE` - Path to system_config.json
- `--images DIR` - Directory containing test images (jpg, jpeg, png, bmp)

**Optional arguments:**
- `--output DIR` - Directory to save annotated images
- `--engine FILE` - Override engine file from config

**Example:**
```bash
./install/game_piece_detection/lib/game_piece_detection/detection_test \
    --config src/vision_config_data/data/system_config.json \
    --images /home/user/test_images \
    --output /tmp/detection_output
```

**Output:**
```
Loading engine from: /path/to/model.engine
Engine loaded in 1234 ms
=== TensorRT Engine Info ===
Input shape: [1, 3, 640, 640]
  Channels: 3
  Height: 640
  Width: 640
Output shape: [1, 5, 8400]
  Num classes: 1
  Num predictions: 8400
Class names: [ball]

Found 10 images to process

Processing: image001.jpg
  Size: 1920x1080, Inference: 3.45 ms, Detections: 2
    class: ball (0), conf: 0.92, bbox: [...]
    class: ball (0), conf: 0.87, bbox: [...]
...
```

## Running the Inference Benchmark

Benchmark inference latency with a single image:

```bash
./install/game_piece_detection/lib/game_piece_detection/inference_benchmark \
    --config <config_file> \
    --image <image_file> \
    [options]
```

**Required arguments:**
- `--config FILE` - Path to system_config.json
- `--image FILE` - Path to test image

**Optional arguments:**
- `--engine FILE` - Override engine file from config
- `--warmup N` - Number of warmup iterations (default: 50)
- `--iterations N` - Number of benchmark iterations (default: 1000)
- `--output FILE` - Output CSV file (default: benchmark_results.csv)

**Example:**
```bash
./install/game_piece_detection/lib/game_piece_detection/inference_benchmark \
    --config src/vision_config_data/data/system_config.json \
    --image test_image.jpg \
    --warmup 100 \
    --iterations 500 \
    --output results.csv
```

**Output:**
```
=== Inference Benchmark ===
Config: system_config.json
Engine: /path/to/model.engine
...

=== Results ===

Inference Time (ms):
  Mean:   3.456
  Std:    0.123
  Min:    3.210
  Max:    4.567
  Median: 3.445
  P95:    3.678
  P99:    3.890

Post-processing Time (ms):
  Mean:   0.045
  ...

Total Time (ms):
  Mean:   3.501
  ...

Throughput: 285.63 FPS
```

**CSV output columns:** `iteration`, `inference_ms`, `postprocess_ms`, `total_ms`

## Model Information

The detection system supports any YOLOv11 model with standard tensor layouts:

**Input tensor:** `[batch, channels, height, width]`
- Typically `[1, 3, 640, 640]` for RGB models

**Output tensor:** `[batch, 4+num_classes, num_predictions]`
- 4 bbox coordinates: x_center, y_center, width, height
- Followed by class confidence scores
- Example: `[1, 5, 8400]` for 1 class, `[1, 6, 8400]` for 2 classes

The number of classes and predictions are automatically extracted from the engine file. Class names are provided via the config file.
