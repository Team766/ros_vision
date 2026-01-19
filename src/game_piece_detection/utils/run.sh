#!/bin/bash
# Wrapper script to run detection_tools commands in the virtual environment

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

if [ ! -d "${VENV_DIR}" ]; then
    echo "Error: Virtual environment not found at ${VENV_DIR}"
    echo "Run setup_env.sh first: ${SCRIPT_DIR}/setup_env.sh"
    exit 1
fi

# Run the command in the virtual environment
source "${VENV_DIR}/bin/activate"
exec "$@"
