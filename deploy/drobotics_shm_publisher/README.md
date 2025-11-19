# Drobotics Shared Memory Bridge

This directory contains the ROS2 to Shared Memory bridge for the Droit Robotics camera.

## Location on Robot
After deployment, this directory is located at:
```
/home/booster/etc/drobotics_shm_publisher/
```

## Usage

### Start the bridge:
```bash
cd /home/booster/etc/drobotics_shm_publisher
./start_shm_bridge.sh
```

The script will:
1. Check if the package is already built
2. Build it if needed (using colcon)
3. Start the publisher node

### Manual build:
```bash
cd /home/booster/etc/drobotics_shm_publisher
source /opt/booster/BoosterRos2/install/setup.bash
colcon build --packages-select drobotics_shm_publisher
```

### Manual start:
```bash
cd /home/booster/etc/drobotics_shm_publisher
source /opt/booster/BoosterRos2/install/setup.bash
source install/setup.bash
ros2 run drobotics_shm_publisher publisher
```

## What it does

The bridge subscribes to:
- `/booster_camera_bridge/StereoNetNode/rectified_image` (RGB image in NV12 format)
- `/booster_camera_bridge/StereoNetNode/stereonet_depth` (Depth image in mono16 format)

And writes them to shared memory:
- `/drobotics_rgb_shm` with semaphore `/drobotics_rgb_sem`
- `/drobotics_depth_shm` with semaphore `/drobotics_depth_sem`

The firmware reads from these shared memory segments using the `Drobotics` camera connector.

## Workflow

1. Camera is started automatically by firmware's `Drobotics::start_ros()`
2. This bridge must be started manually (or via systemd service)
3. Firmware reads images from shared memory via `Drobotics::get_image()`
