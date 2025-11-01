#!/bin/bash

# Convenient release script for ROS Vision workspace
# Combines version management with bundle creation

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }
print_note() { echo -e "${BLUE}[NOTE]${NC} $1"; }

# Function to build and create bundle
build_and_bundle() {
    print_info "Building workspace..."
    if ! (source /opt/ros/humble/setup.bash && source ./build_env_vars.sh && colcon build); then
        print_error "Build failed!"
        return 1
    fi
    
    bundle_only
}

# Function to create bundle without building
bundle_only() {
    print_info "Creating versioned bundle..."
    if ! ./create_bundle.sh; then
        print_error "Bundle creation failed!"
        return 1
    fi
    
    # Show the created bundle
    local version=$(./version_manager.sh get)
    local bundle_file="ros_vision_bundle-v${version}.tar.gz"
    if [[ -f "$bundle_file" ]]; then
        print_info "Bundle created successfully:"
        ls -lh "$bundle_file"
    fi
}

# Function to perform a release
do_release() {
    local bump_type="$1"
    
    print_info "Starting $bump_type release process..."
    
    # Check for uncommitted changes
    if ! git diff-index --quiet HEAD --; then
        print_warn "You have uncommitted changes!"
        git status --porcelain
        read -p "Continue with release? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_info "Release cancelled"
            return 1
        fi
    fi
    
    # Bump version
    local new_version=$(./version_manager.sh bump "$bump_type")
    if [[ $? -ne 0 ]]; then
        print_error "Version bump failed!"
        return 1
    fi
    
    print_info "Bumped to version: $new_version"
    
    # Build and create bundle
    if ! build_and_bundle; then
        print_error "Build and bundle failed for version $new_version"
        return 1
    fi
    
    print_info "Release $new_version completed successfully!"
    print_note "Don't forget to push the tag: git push origin v$new_version"
}

# Main script logic
case "$1" in
    "patch"|"minor"|"major")
        do_release "$1"
        ;;
    "bundle")
        build_and_bundle
        ;;
    "bundle-only")
        bundle_only
        ;;
    "version")
        ./version_manager.sh show
        ;;
    *)
        echo "ROS Vision Release Manager"
        echo "Usage: $0 {patch|minor|major|bundle|bundle-only|version}"
        echo ""
        echo "Release Commands:"
        echo "  patch             Bump patch version and create release bundle"
        echo "  minor             Bump minor version and create release bundle"
        echo "  major             Bump major version and create release bundle"
        echo ""
        echo "Utility Commands:"
        echo "  bundle            Build workspace and create versioned bundle"
        echo "  bundle-only       Create versioned bundle without building"
        echo "  version           Show current version information"
        echo ""
        echo "Examples:"
        echo "  $0 patch          # 1.0.0 -> 1.0.1, build, and bundle"
        echo "  $0 minor          # 1.0.1 -> 1.1.0, build, and bundle"
        echo "  $0 bundle         # Build workspace and bundle current version"
        echo "  $0 bundle-only    # Bundle current version (skip build)"
        echo ""
        print_note "This script manages workspace-level versioning only"
        print_note "Package.xml versions remain independent"
        exit 1
        ;;
esac