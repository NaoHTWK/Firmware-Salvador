#!/usr/bin/env bash

# Function to check if command succeeded
check_success() {
  if [ $? -ne 0 ]; then
      echo "❌ Error: $1"
      docker run --rm --network host -v $TOOLCHAIN_VOLUME:/l4t -v $(pwd):$(pwd) $CONTAINER_NAME bash -c "
         chown -R $USER_ID:$GROUP_ID /workspaces/firmware_6.0
       "
    exit 1
  else
    echo "✅ Success: $1"
  fi
}

# Function to set the right permissions to SSH files
fix_ssh_key_file_permissions() {
	for fix_this_file in deploy-helpers/your.ssh.key.file deploy-helpers/your.ssh.key.file.pub deploy-helpers/.ssh deploy-helpers/.ssh/authorized_keys ; do
		# this will be 0 if perms is ok, nonzero if ssh would complain about key permissions
		local key_group_perms=$(( ( $(${stat_cmd} -c %a ${fix_this_file}) % 100 ) / 10 ))
		local key_other_perms=$(( $(${stat_cmd} -c %a ${fix_this_file}) % 10 ))

		if [[ ${key_group_perms} -ne 0 || ${key_other_perms} -ne 0 ]] ; then
			echo Tweaking SSH priv key file permissions.
			${chmod_cmd} g-rwx,o-rwx -- ${fix_this_file}
		fi
	done
}

LOCAL_DIR=$(pwd)

# Docker image configuration
DOCKER_IMAGE_VERSION="release1_0"
SFTP_HOST="${SFTP_HOST:-your}"
SFTP_PORT="${SFTP_PORT:-infrastructure}"
SFTP_USER="${SFTP_USER:-here}"

DOCKERFILE_PATH="toolchains/Robot.Dockerfile"
CONTAINER_NAME="htwk-robots:compile-booster"
IMAGE_BASE_NAME="compile-booster"
VERSIONED_IMAGE_NAME="${IMAGE_BASE_NAME}:${DOCKER_IMAGE_VERSION}"
ROBOT_MODEL="t1"
##default is ""
ROBOT_SUBMODEL="T1_robocup_version"
CAMERA_MODEL="realsense"

find_cmd=find
stat_cmd=stat
chmod_cmd=chmod
if [ "$(uname)" == "Darwin" ] ; then
    # macOS·
    find_cmd=gfind
    stat_cmd=gstat
    chmod_cmd=gchmod
fi

CMAKE_TOOLCHAIN_FILE="$LOCAL_DIR/toolchains.cmake"
BUILD_PATH="build-t1"

# Architecture the last build was made for (assume aarch64 if no build was found)
LAST_BUILD_ARCH="aarch64"
if [[ -f build/fw_salvador ]]; then
    LAST_BUILD_ARCH=$(file build/fw_salvador | tr ', ' '\n' | head -9 | tail -1)
fi

# Parse command line arguments
DEPLOY_IP=""
DEPLOY_ONLY=false
wifi_ssid=''
SIMULATION_MODE="OFF"
while [[ "$#" -gt 0 ]]; do
	case $1 in
        --ssid|-s) 
            wifi_ssid="${2}" 
            shift
            shift
            ;;
        --deploy) 
            echo "Deploy only"
            DEPLOY_ONLY=true
            shift
            ;;
        --simulation)
            echo "Simulation build detected"
            CONTAINER_IMAGE="ubuntu:22.04"
            TOOLCHAIN_VOLUME="sim-toolchain"
            DOCKERFILE_PATH="toolchains/Simulation.Dockerfile"
            CONTAINER_NAME="htwk-robots:compile-booster-sim"
            IMAGE_BASE_NAME="compile-booster-sim"
            VERSIONED_IMAGE_NAME="${IMAGE_BASE_NAME}:${DOCKER_IMAGE_VERSION}"
            CMAKE_TOOLCHAIN_FILE="$LOCAL_DIR/sim_toolchains.cmake"
            SIMULATION_MODE="ON"
            DEPLOY_PATH="deploy_sim"
            BUILD_PATH="build-sim"
            shift
            ;;
        --t1)
            echo "building for T1"
            DEPLOY_PATH="deploy_t1"
            BUILD_PATH="build-t1"
            ROBOT_MODEL="t1"
            CAMERA_MODEL="realsense"
            shift
            ;;
        --t1_robocup_version)
            echo "building for T1"
            DEPLOY_PATH="deploy_t1"
            BUILD_PATH="build-t1"
            ROBOT_MODEL="t1"
            ROBOT_SUBMODEL="T1_robocup_version"
            CAMERA_MODEL="realsense"
            shift
            ;;
        --k1z)
            echo "building for K1 with Zed2 camera"
            DEPLOY_PATH="deploy_k1z"
            BUILD_PATH="build-k1z"
            ROBOT_MODEL="k1"
            CAMERA_MODEL="zed2"
            shift
            ;;
        --k1bc)
            echo "building for K1 with Booster camera"
            DEPLOY_PATH="deploy_k1bc"
            BUILD_PATH="build-k1bc"
            ROBOT_MODEL="k1"
            CAMERA_MODEL="booster_camera"
            shift
            ;;
        -h|--help)
            echo "Usage: ./install.bash [OPTIONS] [ip of booster]"
            echo "     --deploy                         only deploy to robot and not build"
            echo "-s | --ssid [SSID]                TBD update the Wifi of the Robot"
            echo "     --simulation                     build for simulation"
            echo "     --t1                             build for T1"
            echo "     --k1                             build for K1 (default)"
            echo "-h | --help                           nobody really knows what this one does …"
            echo "     [ip of booster]                  ip address of the robot to deploy to, does not need to be ipv4"
            shift
            exit 0;;
		*)
            DEPLOY_IP="$1"
            echo "Deploy IP set to $DEPLOY_IP"
            shift
            ;;
esac; done

DEPLOY_PATH="${BUILD_PATH}/deploy"

if [ $DEPLOY_ONLY = false ] ; then
    echo $CONTAINER_NAME > .lsp-container
    echo $BUILD_PATH > .lsp-build-path

    echo "Building using toolchain $DOCKERFILE_PATH in $CONTAINER_NAME last build was for $LAST_BUILD_ARCH"
    echo "Deployment files will be written to ${DEPLOY_PATH}"

    file_exists_in_docker() {
        docker run --rm --network host -v $TOOLCHAIN_VOLUME:/l4t $CONTAINER_NAME bash -c "test -e $1"
    }

    #if [[ "$SIMULATION_MODE" = "OFF" ]] && [[ "$LAST_BUILD_ARCH" != "aarch64" ]]; then
    #    echo "Cleanup required"
    #    rm -rf build/
    #    check_success "Cleanup for real robot after last build was for simulation"
    #fi
    #
    #if [[ "$SIMULATION_MODE" = "ON" ]] && [[ "$LAST_BUILD_ARCH" == "aarch64" ]]; then
    #    echo "Cleanup required"
    #    rm -rf build/
    #    check_success "Cleanup for simulation after last build was for real robot"
    #fi

    # Get current user ID and group ID for docker to use
    USER_ID=$(id -u)
    GROUP_ID=$(id -g)

    # Check if Docker image exists, download from SFTP if needed
    if ! docker image inspect "$VERSIONED_IMAGE_NAME" >/dev/null 2>&1; then
        echo "Docker image $VERSIONED_IMAGE_NAME not found locally, downloading from SFTP..."

        IMAGE_FILENAME="${IMAGE_BASE_NAME}_${DOCKER_IMAGE_VERSION}"

        # Preflight: ensure required tools for download/decompression/import are available
        for required_cmd in scp zstd pv; do
            if ! command -v "$required_cmd" >/dev/null 2>&1; then
                echo "❌ Required tool '$required_cmd' is not installed or not in PATH. Please install it and rerun."
                exit 1
            fi
        done

        # Create temporary file for the compressed image (use directory with most available space)
        temp_dir=""
        # Try directories in order of preference for available space
        for dir in "/var/tmp" "$HOME/tmp" "$HOME" "/tmp"; do
            if [ -w "$dir" ] && [ -d "$dir" ]; then
                # Check if directory has at least 30GB free
                if df -k "$dir" | awk 'NR==2 {if ($4 > 30000000) print "ok"}' | grep -q ok 2>/dev/null; then
                    temp_dir="$dir"
                    break
                fi
            fi
        done

        if [ -z "$temp_dir" ]; then
            temp_dir="/tmp"  # fallback
            echo "⚠️ Using /tmp as last resort - may run out of space!"
        fi

        echo "Using temporary directory: $temp_dir"
        temp_compressed="${temp_dir}/${IMAGE_FILENAME}.tar.zst"

        echo "Downloading: $IMAGE_FILENAME (this may take several minutes)"

        # Step 1: Download compressed image from SFTP
        echo "Step 1/3: Downloading compressed image from SFTP..."
        if ! scp -P "$SFTP_PORT" "$SFTP_USER@$SFTP_HOST:/toolchains/$IMAGE_FILENAME.tar.zst" "$temp_compressed" 2>&1; then
            echo "❌ Failed to download image from SFTP"
            rm -f "$temp_compressed"
            exit 1
        fi

        # Step 2: Verify downloaded file integrity
        echo "Step 2/4: Verifying downloaded file..."
        if ! zstd -t "$temp_compressed"; then
            echo "❌ Downloaded file is corrupted or incomplete"
            rm -f "$temp_compressed"
            exit 1
        fi

        # Get compressed file size for progress indication
        compressed_size=$(stat -c%s "$temp_compressed" 2>/dev/null || stat -f%z "$temp_compressed" 2>/dev/null || echo "0")
        temp_uncompressed="${temp_compressed%.tar.zst}.tar"

        # Step 3: Decompress zstd archive with progress
        echo "Step 3/4: Decompressing zstd archive..."
        if ! zstd -dc "$temp_compressed" | pv -s "$compressed_size" > "$temp_uncompressed"; then
            echo "❌ Failed to decompress archive"
            rm -f "$temp_compressed" "$temp_uncompressed"
            exit 1
        fi

        # Get uncompressed file size for import progress
        uncompressed_size=$(stat -c%s "$temp_uncompressed" 2>/dev/null || stat -f%z "$temp_uncompressed" 2>/dev/null || echo "0")

        # Step 4: Import into Docker with progress
        echo "Step 4/4: Importing image into Docker..."
        if ! pv -s "$uncompressed_size" "$temp_uncompressed" | docker import - "$VERSIONED_IMAGE_NAME"; then
            echo "❌ Failed to import Docker image"
            rm -f "$temp_compressed" "$temp_uncompressed"
            exit 1
        fi

        # Clean up uncompressed file
        rm -f "$temp_uncompressed"

        # Success
        echo "✅ Successfully downloaded and loaded image from SFTP"
        rm -f "$temp_compressed"
    else
        echo "Using existing Docker image: $VERSIONED_IMAGE_NAME"
        # Still need to tag it as CONTAINER_NAME for the build process
        docker tag "$VERSIONED_IMAGE_NAME" "$CONTAINER_NAME" 2>/dev/null || true
    fi

    # Find the loaded image and tag it appropriately
    LOADED_IMAGE=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep "^${IMAGE_BASE_NAME}:" | head -1)
    if [ -n "$LOADED_IMAGE" ]; then
        docker tag "$LOADED_IMAGE" "$CONTAINER_NAME"
        echo "✅ Tagged image as: $CONTAINER_NAME"
    fi

    echo "Running the build in $CONTAINER_NAME ..."
    docker run --rm --network host -v $LOCAL_DIR:$LOCAL_DIR -v $(pwd)/$DEPLOY_PATH:/install $CONTAINER_NAME bash -c "  
        cd $LOCAL_DIR &&
        mkdir -p $BUILD_PATH &&
        cd $BUILD_PATH &&
        chown -R root:root ${LOCAL_DIR}/$BUILD_PATH &&

        # Native build without cross-compilation
         cmake --debug-trycompile -DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN_FILE -DCMAKE_COLOR_MAKEFILE=ON -DSIMULATION_MODE=$SIMULATION_MODE -DCMAKE_INSTALL_PREFIX=/install -DROBOT_MODEL=$ROBOT_MODEL -DROBOT_SUBMODEL=\"$ROBOT_SUBMODEL\" -DCAMERA_MODEL=$CAMERA_MODEL .. &&
        CLICOLOR_FORCE=1 make -j$(nproc) install &&

        # Copy shared libraries from baked sysroot into deployment (only for real robot builds)
        if [ \"$SIMULATION_MODE\" = \"OFF\" ] && [ -d \"/l4t/targetfs/usr/lib\" ]; then
            mkdir -p /install/lib &&
            shopt -s nullglob &&
            cp -a /l4t/targetfs/usr/lib/*.so* /install/lib/ 2>/dev/null || true &&
            if [ -d \"/l4t/targetfs/usr/lib/aarch64-linux-gnu\" ]; then
                cp -a /l4t/targetfs/usr/lib/aarch64-linux-gnu/*.so* /install/lib/ 2>/dev/null || true
            fi &&
            shopt -u nullglob
        fi
        # delete Zed2 libs because they dynamically load libsl_ai.so from /usr/lib, so for now we use the ones present on the bot.
        rm /install/lib/libsl_ai.so
        rm /install/lib/libsl_zed.so
    "
    check_success "Build process"

    docker run --rm --network host -v $LOCAL_DIR:$LOCAL_DIR $CONTAINER_NAME bash -c "
           chown -R $USER_ID:$GROUP_ID $LOCAL_DIR
    "

    echo "Build completed successfully!"
fi

if [ -n "$DEPLOY_IP" ]; then
    echo "Deploying to robot at $DEPLOY_IP ..."
    fix_ssh_key_file_permissions

    rsync -av --rsh="ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -l booster" ${DEPLOY_PATH}/bin/ booster@$DEPLOY_IP:/home/booster/bin/ --progress
    check_success "Uploaded binaries"
    rsync -av --rsh="ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -l booster" ${DEPLOY_PATH}/lib/ booster@$DEPLOY_IP:/home/booster/lib/ --progress
    check_success "Uploaded libraries"
    rsync -av --rsh="ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -l booster" deploy/ booster@$DEPLOY_IP:/home/booster/etc/ --progress
    check_success "Uploaded files"

    if [ "$wifi_ssid" ]; then
        echo "ToDo: Set Wifi SSID when installing on the robot"
    fi

    NTP_IP=$(./deploy-helpers/local_ipv4.py --no-fail)

    ssh -t -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file booster@$DEPLOY_IP \
    NTP_SERVER_IP="$NTP_IP" 'bash -s' << 'EOF'

echo "📡 Setting up NTP sync to $NTP_SERVER_IP ..."

echo 123456 | sudo -S bash -c "printf '[Time]\nNTP=$NTP_SERVER_IP\nFallbackNTP=\n' > /etc/systemd/timesyncd.conf"

# important to change different at cuntry
sudo timedatectl set-timezone Europe/Berlin

echo 123456 | sudo -S timedatectl set-ntp true

echo "⏱ Forcing immediate time sync..."
echo 123456 | sudo -S systemctl restart systemd-timesyncd

# Wait a moment for NTP to start
sleep 3

# Function to validate NTP configuration
validate_ntp_config() {
    local ntp_server_ip="$1"

    echo "🔍 Validating NTP configuration..."

    # Check if NTP service is active
    if systemctl is-active --quiet systemd-timesyncd; then
        echo "✅ NTP service is running"
    else
        echo "❌ NTP service is not running"
        return 1
    fi

    # Check if NTP is enabled
    if timedatectl show --property=NTP | grep -q "yes"; then
        echo "✅ NTP synchronization is enabled"
    else
        echo "❌ NTP synchronization is not enabled"
        return 1
    fi

    # Check time synchronization status
    if timedatectl show --property=NTPSynchronized | grep -q "yes"; then
        echo "✅ Time is synchronized with NTP server"
    else
        echo "⚠️  Time is not yet synchronized (this may take a few minutes)"
        echo "   Current time: $(date)"
        echo "   NTP server: $ntp_server_ip"
    fi

    # Check if we can reach the NTP server
    if ping -c 1 -W 3 $ntp_server_ip > /dev/null 2>&1; then
        echo "✅ NTP server $ntp_server_ip is reachable"
    else
        echo "❌ NTP server $ntp_server_ip is not reachable"
        return 1
    fi

    echo "📊 NTP Status Summary:"
    timedatectl status

    return 0
}

# Call the validation function
validate_ntp_config "$NTP_SERVER_IP"

EOF

    echo "Deployment completed successfully!"
else
    echo -e "\n\nIf you want to copy onto the robot:"
	echo "install.bash [robot ip]"
fi

# Deploy to robot if IP is specified
#if [ ! -z "$DEPLOY_IP" ]; then
#    echo "Deploying to robot at $DEPLOY_IP..."
#
#    # Create deployment directories on the robot
#    ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file booster@$DEPLOY_IP "mkdir -p /home/booster/bin"
#    ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file booster@$DEPLOY_IP "mkdir -p /home/booster/lib"
#    ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file booster@$DEPLOY_IP "mkdir -p /home/booster/etc"
#    check_success "Creating deployment directories on robot"
#
#    # Copy binary and libraries
#    scp -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -r ${DEPLOY_PATH}/bin/* booster@$DEPLOY_IP:/home/booster/bin
#    check_success "Copying binaries to robot"
#
#    scp -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -r ${DEPLOY_PATH}/lib/* booster@$DEPLOY_IP:/home/booster/lib/
#    check_success "Copying libraries to robot"
#
#    scp -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -r ${DEPLOY_PATH}/etc/* booster@$DEPLOY_IP:/home/booster/etc/
#    check_success "Copying libraries to robot"
#
#    echo "Deployment completed successfully!"
#fi
