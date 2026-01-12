# Game Piece Detection

YOLOv11-based game piece detection using TensorRT for FRC Reefscape. Detects algae and coral game pieces.

## Prerequisites

- TensorRT engine file (`.engine`) generated from a trained YOLOv11 model
- CUDA 11.8 and TensorRT 10.6+

## Converting ONNX to TensorRT Engine

Convert your ONNX model to a TensorRT engine using `trtexec`:

```bash
# FP16 precision (recommended)
/usr/src/tensorrt/bin/trtexec --onnx=reefscape-yolov11-weights.onnx --saveEngine=reefscape-yolov11-weights.engine --fp16

# FP32 precision
/usr/src/tensorrt/bin/trtexec --onnx=reefscape-yolov11-weights.onnx --saveEngine=reefscape-yolov11-weights.engine
```

Note: Engine files are not portable between different CUDA/TensorRT versions or GPU architectures. Regenerate the engine on each target system.

## Running the Detection Test

Run inference on a directory of test images:

```bash
./install/game_piece_detection/lib/game_piece_detection/detection_test \
    <engine_file> \
    <image_directory> \
    [output_directory]
```

**Arguments:**
- `engine_file` - Path to TensorRT engine file (`.engine`)
- `image_directory` - Directory containing test images (jpg, png, bmp)
- `output_directory` - Optional: Directory to save annotated images

**Example:**
```bash
./install/game_piece_detection/lib/game_piece_detection/detection_test \
    reefscape-yolov11-weights.engine \
    /home/cpadwick/data/reefscape/test/images \
    /tmp/detection_output
```

## Running the Inference Benchmark

Benchmark inference latency with a single image loaded into memory:

```bash
./install/game_piece_detection/lib/game_piece_detection/inference_benchmark \
    <engine_file> \
    <image_file> \
    [options]
```

**Arguments:**
- `engine_file` - Path to TensorRT engine file (`.engine`)
- `image_file` - Path to test image

**Options:**
- `--warmup N` - Number of warmup iterations (default: 50)
- `--iterations N` - Number of benchmark iterations (default: 1000)
- `--output FILE` - Output CSV file (default: benchmark_results.csv)

**Example:**
```bash
./install/game_piece_detection/lib/game_piece_detection/inference_benchmark \
    reefscape-yolov11-weights.engine \
    /home/cpadwick/data/reefscape/test/images/frame_004048_png.rf.042d89d8d13336512cf942c4863dcd63.jpg \
    --warmup 50 \
    --iterations 1000 \
    --output benchmark_results.csv
```

**Output:**
- Console: Statistics (mean, std dev, min, max, median, P95, P99) and throughput (FPS)
- CSV: Per-iteration timings with columns `iteration`, `inference_ms`, `postprocess_ms`, `total_ms`

## Model Information

- Input: 640x640 RGB image
- Output: `[1, 6, 8400]` tensor (4 bbox coords + 2 class scores per prediction)
- Classes: `{0: "algae", 1: "coral"}`
