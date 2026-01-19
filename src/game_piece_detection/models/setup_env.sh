#!/bin/bash
# Setup script for modelzoo virtual environment
# Run this once to create the environment, then use download_models.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

echo "Setting up modelzoo environment..."

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

echo ""
echo "Setup complete!"
echo ""
echo "To download models:"
echo "  1. Activate the environment: source ${VENV_DIR}/bin/activate"
echo "  2. Run: ./download_models.sh"
echo ""
echo "Or use the run wrapper: ./run.sh download_models.sh"
