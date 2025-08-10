#!/bin/bash

# Script to create a versioned bundle tarball of the install directory
# Handles broken symlinks gracefully and uses workspace semantic versioning

INSTALL_DIR="${1:-install}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Get workspace version
get_workspace_version() {
    if [[ -f "$SCRIPT_DIR/VERSION" ]]; then
        cat "$SCRIPT_DIR/VERSION"
    else
        echo "0.1.0"
    fi
}

# Generate output filename with version
WORKSPACE_VERSION=$(get_workspace_version)
OUTPUT_FILE="${2:-ros_vision_bundle-v${WORKSPACE_VERSION}.tar.gz}"

echo "Creating bundle from install directory: $INSTALL_DIR"
echo "Workspace version: $WORKSPACE_VERSION"
echo "Output file: $OUTPUT_FILE"

# Check if install directory exists
if [ ! -d "$INSTALL_DIR" ]; then
    echo "Error: Install directory '$INSTALL_DIR' not found. Run 'colcon build' first."
    exit 1
fi

# Create tarball with dereferenced symlinks, ignoring broken symlinks
echo "Creating tarball with dereferenced symlinks..."
if tar -h --ignore-failed-read -czf "$OUTPUT_FILE" -C "$INSTALL_DIR" . 2>/dev/null; then
    echo "Bundle created successfully: $OUTPUT_FILE"
    # Show size of created bundle
    ls -lh "$OUTPUT_FILE"
    exit 0
else
    echo "Bundle created with warnings: $OUTPUT_FILE"
    # Show size even if there were warnings
    if [ -f "$OUTPUT_FILE" ]; then
        ls -lh "$OUTPUT_FILE"
        exit 0
    else
        echo "Failed to create bundle"
        exit 1
    fi
fi