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
  - Booster Camera: `--k1bc`
- **Deploy Directories:** 
  - `build-k1z/deploy` (ZED2)
  - `build-k1bc/deploy` (Booster Camera)
    using the Booster Camera is highly untestet. It might only work in 1 out of 5 cases. 

---

### Needed Software
- **scp** (SFTP client)
- **zstd** (compression)
- **pv** (progress display)

## 🔧 Installation, Build & Deployment

### 1. Prepare Toolchain Images
This command creates the Docker images for cross-compilation. Run this on the mashine (not the robot) that will build the firmware . This needs to be executed once. 
It may take a while and requires internet access. You can copy the docker image on other mashines so it needs to be run only once per team.
```bash
./toolchains/build_and_push_images.sh
```



### 2. Build Firmware
This command will build our firmware. The Docker image from step 1 needs to be aviable.
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

#### For K1 with Booster Camera Camera
```bash
./install.bash --k1bc
```

### 3.1. Build and Deploy to Single Robot

This command will build and deploy our firmware to a robot. Please note that the robot model needs to be specified.

```bash
./install.bash --t1_robocup_version <ROBOT_IP>
```

Example:
```bash
./install.bash --t1_robocup_version 10.0.44.15
```

### 3.2. Deploy Only (without Build)

The --deploy flag will skip the building. It only works if the last build was sucessful.

```bash
./install.bash --deploy --t1_robocup_version <ROBOT_IP>
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


