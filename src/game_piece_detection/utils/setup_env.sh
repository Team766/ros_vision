#!/bin/bash
# Setup script for detection_tools virtual environment
# Run this once to create the environment, or after updating requirements.txt

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

echo "Setting up detection_tools environment..."

# Create virtual environment if it doesn't exist
if [ ! -d "${VENV_DIR}" ]; then
    echo "Creating virtual environment at ${VENV_DIR}"
    python3 -m venv "${VENV_DIR}"
fi

# Activate and install
source "${VENV_DIR}/bin/activate"

echo "Installing dependencies..."
pip install --upgrade pip
pip install -r "${SCRIPT_DIR}/requirements.txt"
pip install -e "${SCRIPT_DIR}"

echo ""
echo "Setup complete!"
echo ""
echo "To use the tools, either:"
echo "  1. Activate the environment: source ${VENV_DIR}/bin/activate"
echo "  2. Use the run wrapper:      ${SCRIPT_DIR}/run.sh <command> [args]"
echo ""
echo "Available commands:"
echo "  convert_to_onnx <weights.pt>  - Convert YOLO weights to ONNX"
