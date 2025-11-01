# Developer Guide: ROS Vision Workspace

This guide explains the development workflows, versioning system, and utilities available for the ROS Vision workspace.

## Table of Contents
- [Quick Start](#quick-start)
- [Versioning System](#versioning-system)
- [Release Workflow](#release-workflow)
- [Development Utilities](#development-utilities)
- [Bundle Creation](#bundle-creation)
- [Git Integration](#git-integration)
- [Troubleshooting](#troubleshooting)

## Quick Start

### First Time Setup
```bash
# Clone and setup the workspace
git clone <repository-url>
cd ros_vision

# Initial build
source /opt/ros/humble/setup.bash
source ./build_env_vars.sh
./bootstrap.sh

# Regular build
colcon build
```

### Create Your First Release
```bash
# Create a patch release (e.g., 0.1.0 → 0.1.1)
./release.sh patch

# This will:
# 1. Bump the workspace version
# 2. Build the entire workspace  
# 3. Create a versioned bundle
# 4. Create a git tag

# Or just create a bundle from current build
./release.sh bundle-only              # Fastest - no build step
```

## Versioning System

### Two-Level Versioning Strategy

The workspace uses a **two-level versioning approach**:

1. **Workspace Version** (`VERSION` file) - Used for deployment bundles and releases
2. **Package Versions** (`package.xml` files) - Independent ROS2 package versions

```
ros_vision/
├── VERSION                           # 1.2.3 (workspace version)
├── src/apriltags_cuda/package.xml   # <version>0.5.2</version> (independent)
├── src/usb_camera/package.xml       # <version>1.0.1</version> (independent)
└── ...
```

### Why This Approach?

- ✅ **Follows ROS2 best practices** - Individual packages maintain semantic versioning
- ✅ **Avoids "version lock-step"** - No need to bump all package versions together
- ✅ **Clean deployment versioning** - Workspace releases are clearly versioned
- ✅ **Git-friendly** - Single version source of truth for tagging

## Release Workflow

### Complete Release Process

Use `release.sh` for the full automated workflow:

```bash
# Patch release: 1.0.0 → 1.0.1 (bug fixes)
./release.sh patch

# Minor release: 1.0.0 → 1.1.0 (new features, backward compatible)
./release.sh minor

# Major release: 1.0.0 → 2.0.0 (breaking changes)
./release.sh major
```

Each release command performs:
1. **Pre-flight checks** - Warns about uncommitted changes
2. **Version bump** - Updates VERSION file using semantic versioning
3. **Build workspace** - Full colcon build with all packages
4. **Create bundle** - Versioned tarball with all dependencies
5. **Git tagging** - Creates annotated tag (e.g., `v1.2.3`)

### Manual Release Steps

For more control, use individual commands:

```bash
# 1. Manual version management
./version_manager.sh bump minor     # 1.0.0 → 1.1.0
./version_manager.sh set 2.0.0-beta # Set specific version

# 2. Build and bundle
./release.sh bundle                  # Build + create bundle

# 3. Git operations (manual)
git add VERSION
git commit -m "Bump version to $(cat VERSION)"
git tag -a v$(cat VERSION) -m "Release version $(cat VERSION)"
git push origin v$(cat VERSION)
```

## Development Utilities

### Version Manager (`version_manager.sh`)

Core utility for workspace version management:

```bash
# Show current status
./version_manager.sh show
# Output:
# [INFO] Current workspace version: 1.2.3
# [INFO] Recent workspace tags: v1.0.0, v1.1.0, v1.2.0
# [INFO] Sample package versions (independent): ...

# Get current version (for scripting)
./version_manager.sh get              # Returns: 1.2.3

# Manual version bumps
./version_manager.sh bump patch       # 1.2.3 → 1.2.4
./version_manager.sh bump minor       # 1.2.3 → 1.3.0
./version_manager.sh bump major       # 1.2.3 → 2.0.0

# Set specific version
./version_manager.sh set 1.5.0        # Set to 1.5.0
./version_manager.sh set 2.0.0-rc1    # Pre-release version
```

### Release Manager (`release.sh`)

High-level release automation:

```bash
# Full release workflows
./release.sh patch                    # Automated patch release
./release.sh minor                    # Automated minor release  
./release.sh major                    # Automated major release

# Development utilities
./release.sh bundle                   # Build workspace + create bundle
./release.sh bundle-only              # Create bundle without building (fastest)
./release.sh version                  # Show version info (same as version_manager.sh show)
```

### Bundle Creator (`create_bundle.sh`)

Low-level bundle creation (usually called by release.sh):

```bash
# Create bundle with current version
./create_bundle.sh

# Create bundle with custom install directory
./create_bundle.sh /path/to/install

# Create bundle with custom filename
./create_bundle.sh install my_bundle-v1.0.0.tar.gz
```

## Bundle Creation

### What's in a Bundle?

Bundles contain the complete `install/` directory with:
- ✅ **All compiled binaries** (executables, libraries)
- ✅ **ROS2 setup scripts** (setup.bash, local_setup.bash, etc.)
- ✅ **Configuration files** (launch files, config JSONs)
- ✅ **Python packages** (with dependencies)
- ✅ **Dereferenced symlinks** (actual files, not broken links)

### Bundle Naming Convention

```
ros_vision_bundle-v{VERSION}.tar.gz

Examples:
ros_vision_bundle-v1.0.0.tar.gz      # Release version
ros_vision_bundle-v1.2.0-beta.tar.gz # Pre-release version
ros_vision_bundle-v2.0.0.tar.gz      # Major release
```

### Bundle Size and Content

Typical bundle size: ~50-60MB (includes OpenCV, CUDA libraries, etc.)

```bash
# Inspect bundle contents
tar -tzf ros_vision_bundle-v1.0.0.tar.gz | head -20

# Extract and test bundle
mkdir test_deployment
tar -xzf ros_vision_bundle-v1.0.0.tar.gz -C test_deployment
cd test_deployment
source setup.bash
ros2 launch ros_vision_launch launch_vision.py
```

## Git Integration

### Tagging Strategy

The version manager automatically creates **annotated git tags**:

```bash
# Tags are created in format: v{VERSION}
v0.1.0, v0.1.1, v1.0.0, v1.2.3, etc.

# View tags
git tag -l "v*" | sort -V

# Show tag details  
git show v1.0.0
```

### Push Tags to Remote

Tags are created locally and need manual pushing:

```bash
# Push specific tag
git push origin v1.0.0

# Push all tags
git push --tags

# Delete tag (if needed)
git tag -d v1.0.0                     # Delete local
git push origin --delete v1.0.0       # Delete remote
```

### Working with Releases

```bash
# Check out a specific release
git checkout v1.0.0

# Create branch from release
git checkout -b hotfix/v1.0.1 v1.0.0

# Compare releases
git diff v1.0.0..v1.1.0
```

## Troubleshooting

### Common Issues

#### 1. "There are uncommitted changes"
```bash
# The version manager warns about uncommitted changes before tagging
# Either commit your changes first, or use --force (not recommended)

git add .
git commit -m "Prepare for release"
./release.sh patch
```

#### 2. "Tag already exists"
```bash
# If you need to re-create a tag
git tag -d v1.0.0                     # Delete local tag
git push origin --delete v1.0.0       # Delete remote tag (if pushed)
./version_manager.sh set 1.0.0        # Re-create
```

#### 3. "Bundle creation failed"
```bash
# Usually due to build failures
# Check the build output and fix issues first
colcon build                          # Manual build to see errors
./release.sh bundle                   # Try bundle creation again
```

#### 4. "Broken symlinks in bundle"
```bash
# The bundle script handles broken symlinks gracefully
# "Bundle created with warnings" is normal and safe
# These are usually test files in dependencies that don't affect functionality
```

### Best Practices

#### Development Workflow
1. **Feature development** - Work on feature branches, don't bump versions
2. **Integration** - Merge to main/develop branch
3. **Testing** - Run full test suite: `colcon test`
4. **Release** - Use release.sh for version bumps and bundle creation
5. **Deployment** - Use the versioned bundle for deployment

#### Version Bumping Guidelines
- **Patch** (1.0.0 → 1.0.1): Bug fixes, no API changes
- **Minor** (1.0.0 → 1.1.0): New features, backward compatible
- **Major** (1.0.0 → 2.0.0): Breaking changes, API changes

#### Pre-release Versions
```bash
# Use pre-release versions for testing
./version_manager.sh set 1.1.0-alpha
./version_manager.sh set 1.1.0-beta
./version_manager.sh set 1.1.0-rc1

# Convert to release version
./version_manager.sh set 1.1.0
```

### Environment Requirements

Make sure you have:
- ROS2 Humble installed (`/opt/ros/humble/setup.bash`)
- Git configured for your user
- Bash shell (scripts use bash-specific features)
- tar with GNU extensions (for --dereference flag)

### Getting Help

```bash
# Show help for any script
./version_manager.sh                  # Shows usage
./release.sh                          # Shows usage

# Check current versions and status
./release.sh version                  # Comprehensive version info
```

---

## Summary

The ROS Vision workspace provides a complete versioning and release system that:

- ✅ **Separates concerns** - Workspace deployment vs package development
- ✅ **Automates releases** - One command for version → build → bundle → tag  
- ✅ **Follows semantic versioning** - Clear version progression
- ✅ **Integrates with Git** - Automatic tagging and version control
- ✅ **Creates deployment-ready bundles** - Self-contained, versioned releases

Use `./release.sh patch` for most releases, and refer to this guide for advanced workflows!