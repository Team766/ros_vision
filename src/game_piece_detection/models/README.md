# Model Zoo

Download and manage YOLO model files from the team's Google Drive.

## Quick Start

```bash
# Setup virtual environment (one time)
./setup_env.sh

# Download all models to this directory
./run.sh ./download_models.sh

# Or activate the venv manually
source .venv/bin/activate
./download_models.sh
```

## Google Drive Location

Models are stored at:
https://drive.google.com/drive/folders/1uNAqcHgcLXGH9LrHOLznUkBBRRb1tyAw

**Note:** The folder must be shared with "Anyone with the link" for automated download to work.

## Converting Models

After downloading, convert to TensorRT engine format:

```bash
# Convert ONNX to TensorRT (FP16)
/usr/src/tensorrt/bin/trtexec \
    --onnx=your-model.onnx \
    --saveEngine=your-model.engine \
    --fp16
```

## Directory Structure

After running the download script:

```
game_piece_detection/models/
├── download_models.sh
├── setup_env.sh
├── run.sh
├── requirements.txt
├── README.md
├── .gitignore
└── 2026_rebuilt/
    ├── model.pt
    ├── model.onnx
    └── ...
```

## Adding Models to the Zoo

1. Train your model using Ultralytics YOLO
2. Export to ONNX: `yolo export model=best.pt format=onnx`
3. Upload both `.pt` and `.onnx` files to the Google Drive folder
4. Team members can then run `./download_models.sh` to get the latest models
