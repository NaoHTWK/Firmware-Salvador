# Docker Toolchain Management

This document explains the new Docker-based cross-compilation workflow for the firmware project.

## Overview

The project uses pre-built Docker images for cross-compilation toolchains distributed via SFTP. Images are automatically downloaded from the NAS when needed and cached in Docker. No local builds or registry access required - images are versioned by git commit for reproducibility.

## For Developers

### Building the Project

Use the existing `install.bash` script as usual:

```bash
# Build for T1 robot
./install.bash --t1

# Build for K1 robot
./install.bash --k1

# Build for simulation
./install.bash --simulation
```

The script automatically:
1. **Checks for cached image** - uses existing Docker image if available
2. **Downloads from SFTP** - gets the correct versioned image from NAS if needed (3-step process with progress)
3. **Caches in Docker** - image stays available for future builds

### Version Control

Images are versioned using Mercurial changeset IDs. The `build_and_push_images.sh` script:
1. Gets the current Mercurial changeset ID (`hg id -i`)
2. Writes it to a `VERSION` file (which gets committed)
3. Builds images tagged with this version

The `install.bash` script reads the `VERSION` file and downloads the matching image. This ensures:
- Reproducible builds across different changesets
- Automatic cache invalidation when toolchains change
- Consistent builds across team members

### Offline Support

Once an image is downloaded, it stays cached in Docker. You can work offline as long as you have the correct image for your current commit.

### Configuration

You can customize the SFTP connection by setting environment variables:

```bash
export SFTP_HOST=your-nas-server.com
export SFTP_PORT=22
export SFTP_USER=yourusername
export DOCKER_IMAGE_VERSION=custom-version
./install.bash --t1
```

Default values:
- `SFTP_HOST=`
- `SFTP_PORT=`
- `SFTP_USER=`
- `DOCKER_IMAGE_VERSION=$(cat VERSION)` (automatically managed by build script)

## For Maintainers

### Building and Uploading Pre-built Images

When toolchain dependencies are updated, maintainers should build and upload new images:

```bash
./build_and_push_images.sh
```

This script will:
1. Build all three toolchain images (T1, K1, Simulation)
2. Upload them to SFTP with bz2 compression (3-step process with detailed progress)
3. Images are versioned by current Mercurial changeset ID

### SFTP Upload Process

The script compresses each image and uploads to:
- ``
- Files: `compile-booster-t1_{commit}.tar.bz2`

### Prerequisites

Before running the build script, ensure:
1. You have Docker installed and running
2. SFTP access to the NAS: ``
3. Mercurial repository is clean (for proper versioning)
4. Changes to VERSION file will be committed automatically

### Image Naming Convention

Images are named by Mercurial changeset ID (from VERSION file):
- `compile-booster-t1_{version}.tar.bz2` - T1 robot toolchain
- `compile-booster_{version}.tar.bz2` - K1 robot toolchain
- `compile-booster-sim_{version}.tar.bz2` - Simulation toolchain

## Troubleshooting

### SFTP Connection Issues

If you can't download images from SFTP:
- Check network connectivity to ``
- Verify SFTP credentials (uses your username by default)
- Ensure the image file exists for your current git commit
- Contact a maintainer to run `build_and_push_images.sh`

### Docker Image Not Found

The script checks for the correct versioned image using `docker image inspect "{image_name}:{version}"`. If not found:

**Version checking happens in install.bash:**
```bash
# Line 154: Check if versioned image exists
if ! docker image inspect "$VERSIONED_IMAGE_NAME" >/dev/null 2>&1; then
    # Download from SFTP...
```

**To troubleshoot:**
- Check your Mercurial status: `hg status`
- Verify the version in VERSION file: `cat VERSION`
- Ensure VERSION file matches your changeset: `hg id -i`
- The image filename should be: `{image_name}_{version}.tar.bz2`

### Offline Development

If you're working offline:
- You need the correct image cached in Docker first
- Run the build script while online to download the image
- Once cached, you can work offline with that commit

### Manual Image Management

To manually manage Docker images:
```bash
# List available images
docker images | grep compile-booster

# Remove old images
docker rmi compile-booster-t1:old-commit-hash

# Load image manually from file
bzcat compile-booster-t1_abc123.tar.bz2 | docker load
```

### Image Size Issues

The pre-built images are large due to the cross-compilation toolchains and dependencies. Docker automatically manages disk space, but you can:
- Clean old images: `docker image prune`
- List disk usage: `docker system df`
- Remove unused images: `docker image prune -a`

## Workflow Summary

1. **Maintainers** run `build_and_push_images.sh` to build and upload versioned images
2. **Developers** run `install.bash --t1/--k1/--simulation` - images download automatically
3. **Offline work** is supported once images are cached in Docker
4. **Version control** ensures reproducible builds across the team
