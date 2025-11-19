#!/bin/bash

# Script to build (if needed) and start the drobotics_shm_publisher

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="$SCRIPT_DIR/install"
BUILD_MARKER="$SCRIPT_DIR/build/.built"

echo "=== Drobotics SHM Bridge Startup Script ==="

# Check if already built
if [ ! -f "$BUILD_MARKER" ] || [ ! -d "$INSTALL_DIR" ]; then
    echo "Bridge not built yet or incomplete. Building now..."
    
    # Source ROS2 environment
    if [ -f "/opt/booster/BoosterRos2/install/setup.bash" ]; then
        source /opt/booster/BoosterRos2/install/setup.bash
        echo "✓ Sourced BoosterRos2 environment"
    else
        echo "✗ Error: BoosterRos2 environment not found!"
        exit 1
    fi
    
    # Build the package
    cd "$SCRIPT_DIR"
    echo "Building drobotics_shm_publisher..."
    colcon build --packages-select drobotics_shm_publisher
    
    if [ $? -eq 0 ]; then
        echo "✓ Build successful"
        mkdir -p "$(dirname "$BUILD_MARKER")"
        touch "$BUILD_MARKER"
    else
        echo "✗ Build failed!"
        exit 1
    fi
else
    echo "✓ Bridge already built"
fi

# Source the built package
if [ -f "$INSTALL_DIR/setup.bash" ]; then
    source /opt/booster/BoosterRos2/install/setup.bash
    source "$INSTALL_DIR/setup.bash"
    echo "✓ Sourced drobotics_shm_publisher environment"
else
    echo "✗ Error: Install directory not found!"
    exit 1
fi

# Start the publisher
echo "Starting drobotics_shm_publisher..."
echo "Press Ctrl+C to stop"
echo "================================"

ros2 run drobotics_shm_publisher publisher
