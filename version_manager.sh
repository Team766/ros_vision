#!/bin/bash

# Workspace-Level Semantic Version Manager for ROS Vision
# Manages workspace version for deployment bundles while keeping package.xml files independent

VERSION_FILE="VERSION"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }
print_note() { echo -e "${BLUE}[NOTE]${NC} $1"; }

# Function to validate semantic version format
validate_version() {
    local version="$1"
    if [[ ! $version =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[a-zA-Z0-9-]+)?(\+[a-zA-Z0-9-]+)?$ ]]; then
        print_error "Invalid semantic version format: $version"
        print_error "Expected format: MAJOR.MINOR.PATCH[-prerelease][+build]"
        return 1
    fi
    return 0
}

# Function to parse version components
parse_version() {
    local version="$1"
    # Extract major.minor.patch part (before any - or +)
    local base_version=$(echo "$version" | sed 's/[-+].*//')
    
    MAJOR=$(echo "$base_version" | cut -d. -f1)
    MINOR=$(echo "$base_version" | cut -d. -f2)
    PATCH=$(echo "$base_version" | cut -d. -f3)
}

# Function to get current workspace version
get_current_version() {
    if [[ -f "$VERSION_FILE" ]]; then
        cat "$VERSION_FILE"
    else
        echo "0.1.0"
    fi
}

# Function to update VERSION file
update_version_file() {
    local new_version="$1"
    echo "$new_version" > "$VERSION_FILE"
    print_info "Updated workspace VERSION file to $new_version"
}

# Function to create git tag
create_git_tag() {
    local version="$1"
    local tag="v$version"
    
    if git rev-parse "$tag" >/dev/null 2>&1; then
        print_warn "Tag $tag already exists"
        return 1
    fi
    
    # Check if there are uncommitted changes
    if ! git diff-index --quiet HEAD --; then
        print_warn "There are uncommitted changes. Consider committing before tagging."
        read -p "Continue with tagging? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_info "Tagging cancelled"
            return 1
        fi
    fi
    
    git tag -a "$tag" -m "Release workspace version $version"
    print_info "Created git tag: $tag"
    print_note "Push with: git push origin $tag"
    return 0
}

# Function to bump version
bump_version() {
    local bump_type="$1"
    local current_version=$(get_current_version)
    
    parse_version "$current_version"
    
    case "$bump_type" in
        "major")
            MAJOR=$((MAJOR + 1))
            MINOR=0
            PATCH=0
            ;;
        "minor")
            MINOR=$((MINOR + 1))
            PATCH=0
            ;;
        "patch")
            PATCH=$((PATCH + 1))
            ;;
        *)
            print_error "Invalid bump type: $bump_type (use major, minor, or patch)"
            return 1
            ;;
    esac
    
    local new_version="$MAJOR.$MINOR.$PATCH"
    
    print_info "Bumping workspace version from $current_version to $new_version"
    
    # Update version file
    update_version_file "$new_version"
    
    # Create git tag
    create_git_tag "$new_version"
    
    print_note "Package versions in package.xml files remain independent"
    print_note "Bundle will be created as: ros_vision_bundle-v$new_version.tar.gz"
    
    echo "$new_version"
}

# Function to set specific version
set_version() {
    local new_version="$1"
    
    if ! validate_version "$new_version"; then
        return 1
    fi
    
    local current_version=$(get_current_version)
    print_info "Setting workspace version from $current_version to $new_version"
    
    # Update version file
    update_version_file "$new_version"
    
    # Create git tag
    create_git_tag "$new_version"
    
    print_note "Package versions in package.xml files remain independent"
    print_note "Bundle will be created as: ros_vision_bundle-v$new_version.tar.gz"
    
    echo "$new_version"
}

# Function to show current version info
show_version() {
    local current_version=$(get_current_version)
    print_info "Current workspace version: $current_version"
    
    # Show git tags
    print_info "Recent workspace tags:"
    git tag -l "v*" | sort -V | tail -5 || echo "  No tags found"
    
    # Show package versions (sample)
    print_info "Sample package versions (independent):"
    find src -name "package.xml" -exec grep -H "<version>" {} \; 2>/dev/null | head -5 | while read line; do
        echo "  $line"
    done
    
    print_note "Workspace version is used for deployment bundles"
    print_note "Package versions remain independent for ROS2 compatibility"
}

# Main script logic
case "$1" in
    "show"|"current")
        show_version
        ;;
    "bump")
        if [[ -z "$2" ]]; then
            print_error "Usage: $0 bump <major|minor|patch>"
            exit 1
        fi
        bump_version "$2"
        ;;
    "set")
        if [[ -z "$2" ]]; then
            print_error "Usage: $0 set <version>"
            exit 1
        fi
        set_version "$2"
        ;;
    "get")
        get_current_version
        ;;
    *)
        echo "ROS Vision Workspace Version Manager"
        echo "Usage: $0 {show|get|bump|set}"
        echo ""
        echo "Commands:"
        echo "  show              Show current workspace version and git tags"
        echo "  get               Get current workspace version (for scripts)"
        echo "  bump <type>       Bump workspace version (major, minor, patch)"
        echo "  set <version>     Set specific workspace version"
        echo ""
        echo "Examples:"
        echo "  $0 show"
        echo "  $0 bump patch     # 1.0.0 -> 1.0.1"
        echo "  $0 bump minor     # 1.0.1 -> 1.1.0"
        echo "  $0 set 2.0.0-beta"
        echo ""
        print_note "Workspace versioning is independent of individual package.xml versions"
        print_note "Use for deployment bundles: ros_vision_bundle-v{version}.tar.gz"
        exit 1
        ;;
esac