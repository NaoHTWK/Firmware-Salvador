#!/usr/bin/env bash

# Script to build and push pre-built Docker images for cross-compilation toolchains
# This script should be run by maintainers when updating the toolchain

set -e

# Configuration - should match install.bash
DOCKER_IMAGE_VERSION="${DOCKER_IMAGE_VERSION:-$(hg id -i 2>/dev/null | cut -c1-12 || echo 'unknown')}"
SFTP_HOST="your"
SFTP_PORT="infra"
SFTP_USER="structur"
PUSH=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Write version to TOOLCHAIN_VERSION file (after functions are defined)
echo "$DOCKER_IMAGE_VERSION" > ../TOOLCHAIN_VERSION
print_status "Updated TOOLCHAIN_VERSION file with: $DOCKER_IMAGE_VERSION"

# Function to build and upload a single image
build_and_push_image() {
    local dockerfile="$1"
    local image_name="$2"
    local full_image_name="$image_name:$DOCKER_IMAGE_VERSION"
    local remote_filename="${image_name}_${DOCKER_IMAGE_VERSION}.tar.zst"

    print_status "Building $full_image_name from $dockerfile..."

    # Build the image
    if docker build --network host -f "toolchains/$dockerfile" -t "$full_image_name" .; then
        print_success "Built $full_image_name successfully"
    else
        print_error "Failed to build $full_image_name"
        return 1
    fi

    # Check if push is enabled
    if [ "$PUSH" = false ]; then
        print_warning "PUSH is disabled - skipping upload to SFTP"
        return 0
    fi

    # Upload to SFTP with bz2 compression
    print_status "Uploading $full_image_name to SFTP..."

    # Create temporary file for the tar archive (use directory with most available space)
    local temp_dir=""
    # Try directories in order of preference for available space
    for dir in "/var/tmp" "$HOME/tmp" "$HOME" "/tmp"; do
        if [ -w "$dir" ] && [ -d "$dir" ]; then
            # Check if directory has at least 20GB free (basic check)
            if df -k "$dir" | awk 'NR==2 {if ($4 > 40000000) print "ok"}' | grep -q ok 2>/dev/null; then
                temp_dir="$dir"
                break
            fi
        fi
    done

    if [ -z "$temp_dir" ]; then
        temp_dir="/tmp"  # fallback
        print_warning "Using /tmp as last resort - may run out of space!"
    fi

    print_status "Using temporary directory: $temp_dir"
    temp_tar="${temp_dir}/${remote_filename%.tar.zst}.tar"
    compressed_file="${temp_tar}.zst"

    # Step 1: Export Docker image filesystem (single layer, smaller size)
    print_status "Step 1/3: Exporting Docker image filesystem (single layer)..."
    # Create a temporary container to export just the final filesystem
    local container_id=$(docker create "$full_image_name")
    if [ -z "$container_id" ]; then
        print_error "Failed to create container from $full_image_name"
        return 1
    fi

    # Get the actual filesystem size by running du inside the container
    print_status "Calculating actual filesystem size..."
    local fs_size=$(docker run --rm "$full_image_name" du -sb / 2>/dev/null | awk '{print $1}' || echo "")
    if [ -n "$fs_size" ] && [ "$fs_size" -gt 0 ] 2>/dev/null; then
        local fs_size_human=$(numfmt --to=iec-i --suffix=B $fs_size 2>/dev/null || echo "${fs_size}B")
        print_status "Actual filesystem size: $fs_size_human"
        pv_cmd="pv -s $fs_size"
    else
        print_status "Could not determine filesystem size - progress bar will not show total size"
        pv_cmd="pv"
    fi

    if ! docker export "$container_id" | $pv_cmd > "$temp_tar"; then
        print_error "Failed to export filesystem from container $container_id"
        docker rm "$container_id" >/dev/null 2>&1 || true
        return 1
    fi

    # Clean up the temporary container
    docker rm "$container_id" >/dev/null 2>&1 || true

    local tar_size=$(stat -c%s "$temp_tar" 2>/dev/null || stat -f%z "$temp_tar" 2>/dev/null || echo "unknown")
    if [ "$tar_size" != "unknown" ]; then
        local tar_size_human=$(numfmt --to=iec-i --suffix=B $tar_size 2>/dev/null || echo "${tar_size}B")
        print_status "Actual exported tar size: $tar_size_human"
    fi

    # Step 2: Compress the tar file
    print_status "Step 2/3: Compressing tar file with zstd..."
    local tar_size=$(stat -c%s "$temp_tar" 2>/dev/null || stat -f%z "$temp_tar" 2>/dev/null || echo "0")
    if ! zstd --progress -T0 -o "$compressed_file" "$temp_tar"; then
        print_error "Failed to compress $temp_tar"
        rm -f "$temp_tar"
        return 1
    fi
    rm -f "$temp_tar"

    # Step 3: Upload to SFTP
    print_status "Step 3/3: Uploading compressed file to SFTP..."
    if ! scp -P "$SFTP_PORT" "$compressed_file" "$SFTP_USER@$SFTP_HOST:/toolchains/$remote_filename" 2>&1; then
        print_error "Failed to upload $compressed_file to SFTP"
        rm -f "$compressed_file"
        return 1
    fi

    # Success
    print_success "Uploaded $remote_filename to SFTP successfully"
    rm -f "$compressed_file"
}

# Check if docker is available
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed or not in PATH"
    exit 1
fi

# Check if user is logged in to registry
if ! docker info &> /dev/null; then
    print_error "Docker daemon is not running or you don't have permission to use it"
    exit 1
fi

print_status "Starting build and push process for firmware_6.0 toolchains"
print_status "Registry: $DOCKER_REGISTRY"
print_status "Tag: $DOCKER_IMAGE_TAG"

# Build and push images for each target
print_status "Building toolchain images..."

# Unified robot image (for both T1 and K1)
if build_and_push_image "Robot.Dockerfile" "compile-booster"; then
    print_success "Unified robot toolchain image processed successfully"
else
    print_error "Failed to process unified robot toolchain image"
    exit 1
fi

# Simulation image
if build_and_push_image "Simulation.Dockerfile" "compile-booster-sim"; then
    print_success "Simulation toolchain image processed successfully"
else
    print_error "Failed to process Simulation toolchain image"
    exit 1
fi

print_success "All toolchain images have been built and uploaded to SFTP!"
print_status "Images are now available on: $SFTP_HOST:$SFTP_PORT/toolchains/"
print_status "Developers can now use these pre-built images by running install.bash"
print_status "Images are automatically downloaded and cached in Docker when needed"
