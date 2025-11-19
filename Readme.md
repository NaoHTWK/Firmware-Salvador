# HTWK Robots Firmware 6.0

This is the firmware for HTWK Robots.
This firmware was successfully used at the RoboCup Salvador 2025, Beijing Masters 2025, and RCAP Abu Dhabi 2025.

**⚠️ Notice:** This firmware comes without any warranty. Use at your own risk. In particular, using the setup scripts can easily lock you out of a robot.

---

## 📋 Table of Contents

- [Supported Robots](#supported-robots)
- [Installation & Build](#installation--build)
- [Deployment](#deployment)
- [License](#license)

---

## 🤖 Supported Robots

### T1
- **Camera:** Intel RealSense
- **Build Options:** `--t1`, `--t1_robocup_version`
- **Deploy Directory:** `build-t1/deploy`

### K1
- **Camera Options:**
  - ZED2: `--k1z`
  - Drobotics: `--k1d`
- **Deploy Directories:** 
  - `build-k1z/deploy` (ZED2)
  - `build-k1d/deploy` (Drobotics)
    using the Droidbotics Camera is highly untestet. It might only work in 1 out of 5 cases. 

---

### Needed Software
- **scp** (SFTP client)
- **zstd** (compression)
- **pv** (progress display)

## 🔧 Installation & Build

### 1. Prepare Toolchain Images

```bash
./toolchains/build_and_push_images.sh
```

This creates the Docker images for cross-compilation. This needs to be executed once. It may take a while and requires internet access.

### 2. Build Firmware

#### For T1 (Standard RoboCup Version)
```bash
./install.bash --t1_robocup_version
```

#### For T1 (Standard)
```bash
./install.bash --t1
```

#### For K1 with ZED2 Camera
```bash
./install.bash --k1z
```

#### For K1 with Drobotics Camera
```bash
./install.bash --k1d
```


### 3. Build Process

The build process runs automatically in a Docker container:
1. The toolchain image must be built or a file server must be specified.
2. Cross-compilation with CMake.
3. Binaries and libraries are compiled.
4. Deployment files are created in `build-*/deploy/`.

---

## 🚀 Deployment

### Build and Deploy to Single Robot

```bash
./install.bash --t1_robocup_version <ROBOT_IP>
```

Example:
```bash
./install.bash --t1_robocup_version 10.0.44.15
```

### Deploy Only (without Build)

```bash
./install.bash --deploy --t1 <ROBOT_IP>
```

### Deployment Process

In addition to the firmware, various setup scripts are included.
Using the scripts is at your own risk. Incorrect usage can easily lock you out of a robot.

### Run

Execute on robot
```bash
./bin/fw_salvador
```

---


## 📝 License

See license information in the repository.

Special license conditions apply for RoboCup teams.

---

**Last Updated:** November 2025


