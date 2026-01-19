#!/bin/bash
#
# Download model files from Google Drive modelzoo
#
# Usage: ./download_modelzoo.sh [output_directory]
#
# Prerequisites:
#   pip install gdown
#
# The Google Drive folder must be shared with "Anyone with the link" for this to work.
#

set -e

# Google Drive folder ID and name (from the share URL)
FOLDER_ID="1uNAqcHgcLXGH9LrHOLznUkBBRRb1tyAw"
FOLDER_NAME="2026_rebuilt"

# Default output directory (this script's location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${1:-${SCRIPT_DIR}/${FOLDER_NAME}}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Model Zoo Downloader ===${NC}"
echo "Google Drive Folder ID: ${FOLDER_ID}"
echo "Output directory: ${OUTPUT_DIR}"
echo ""

# Check if gdown is installed
if ! command -v gdown &> /dev/null; then
    echo -e "${YELLOW}gdown is not installed. Installing...${NC}"
    pip install gdown
fi

# Check gdown version (folder download requires >= 4.4.0)
GDOWN_VERSION=$(gdown --version 2>&1 | grep -oP '\d+\.\d+\.\d+' || echo "0.0.0")
echo "gdown version: ${GDOWN_VERSION}"

# Create output directory if it doesn't exist
mkdir -p "${OUTPUT_DIR}"

echo ""
echo -e "${GREEN}Downloading model zoo...${NC}"
echo ""

# Download the entire folder
# --folder: download entire folder
# --remaining-ok: continue even if some files fail
# -O: output directory
gdown --folder "https://drive.google.com/drive/folders/${FOLDER_ID}" \
    -O "${OUTPUT_DIR}" \
    --remaining-ok

echo ""
echo -e "${GREEN}Download complete!${NC}"
echo ""
echo "Models downloaded to: ${OUTPUT_DIR}/"
ls -la "${OUTPUT_DIR}/"

echo ""
echo -e "${YELLOW}Note: To convert models to TensorRT engine files, run:${NC}"
echo "  /usr/src/tensorrt/bin/trtexec --onnx=<model>.onnx --saveEngine=<model>.engine --fp16"
