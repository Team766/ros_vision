#!/bin/bash
# Run wrapper that activates the virtual environment before executing commands
# Usage: ./run.sh <script> [args...]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

if [ ! -d "${VENV_DIR}" ]; then
    echo "Virtual environment not found. Run setup_env.sh first:"
    echo "  ${SCRIPT_DIR}/setup_env.sh"
    exit 1
fi

source "${VENV_DIR}/bin/activate"
exec "$@"
