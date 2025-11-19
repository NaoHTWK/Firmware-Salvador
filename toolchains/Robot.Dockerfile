# Unified cross-compilation toolchain for T1 and K1 robots
FROM nvcr.io/nvidia/jetpack-linux-aarch64-crosscompile-x86:6.1

LABEL maintainer="HTWK Robots"
LABEL description="Cross-compilation toolchain for T1 and K1 robots"
LABEL version="6.0"
LABEL target="t1,k1"

SHELL ["/bin/bash", "-c"]

RUN cd /l4t/ && cat targetfs.tbz2.* > targetfs.tbz2 && rm -f targetfs.tbz2.* && tar -I lbzip2 -xf targetfs.tbz2 && rm targetfs.tbz2 && \
    mkdir -p toolchain && tar -C toolchain -xf toolchain.tar.bz2 && rm toolchain.tar.bz2 && \
    rm -f /l4t/legal.tbz2 && cd / && \
    export DEBIAN_FRONTEND=noninteractive && \
    dpkg --add-architecture arm64 && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse" > /etc/apt/sources.list.d/ubuntu-ports.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list.d/ubuntu-ports.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted universe multiverse" >> /etc/apt/sources.list.d/ubuntu-ports.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list.d/ubuntu-ports.list && \
    mkdir -p /tmp/apt-sources-backup && \
    mv /etc/apt/sources.list /tmp/apt-sources-backup/sources.list.original || true && \
    apt-get update && \
    apt-get download libusb-1.0-0-dev:arm64 libudev-dev:arm64 && \
    for deb in *.deb; do dpkg -x "$deb" /l4t/targetfs; done && \
    rm *.deb && \
    mv /tmp/apt-sources-backup/sources.list.original /etc/apt/sources.list || true && \
    rm -f /etc/apt/sources.list.d/ubuntu-ports.list && \
    dpkg --remove-architecture arm64 && \
    rm -rf /tmp/apt-sources-backup && \
    apt-get update && apt-get remove -y libboost-* && apt-get install -y \
    clangd-15 clang-format-15 flatbuffers-compiler python3 python-is-python3 zstd unzip wget git curl ca-certificates && \
    rm -f /l4t/targetfs/usr/lib/aarch64-linux-gnu/libboost*.1.74.* && \
    rm -rf /l4t/targetfs/usr/lib/aarch64-linux-gnu/cmake/Boost-1.74.0/ && \
    rm -rf /l4t/targetfs/usr/lib/aarch64-linux-gnu/cmake/boost_headers-1.74.0/ && \
    rm -f /l4t/targetfs/usr/lib/aarch64-linux-gnu/cmake/BoostDetectToolset-1.74.0.cmake

COPY toolchains/toolchains.cmake /toolchains/toolchains.cmake
COPY third_party /third_party

# Set up cross-compilation environment
ENV TARGET_FS=/l4t/targetfs

# Build and install Boost 1.89.0 for arm64
RUN wget -nv -O /tmp/boost.tar.xz https://github.com/boostorg/boost/releases/download/boost-1.89.0/boost-1.89.0-cmake.tar.xz && \
    cd /third_party && tar -xf /tmp/boost.tar.xz && rm /tmp/boost.tar.xz
RUN cd /third_party/boost-1.89.0 && \
    ./bootstrap.sh --prefix=${TARGET_FS}/usr && \
    echo "using gcc : arm64 : /l4t/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-g++ ;" > user-config.jam && \
    ./b2 --user-config=user-config.jam \
        toolset=gcc-arm64 \
        target-os=linux \
        architecture=arm \
        abi=aapcs \
        binary-format=elf \
        address-model=64 \
        threading=multi \
        link=shared \
        runtime-link=shared \
        --prefix=${TARGET_FS}/usr \
        --layout=system \
        --build-type=minimal \
        --with-system \
        --with-atomic \
        --with-filesystem \
        --with-asio \
        --with-serialization \
        --with-program_options \
        -j$(nproc) \
        install && \
    rm -rf /third_party/boost-1.89.0/build

# Build and install loguru for arm64
RUN git clone https://github.com/emilk/loguru.git /third_party/loguru && \
    cd /third_party/loguru && git checkout 4adaa18
RUN cd /third_party/loguru && \
    sed -i 's/-Werror//' CMakeLists.txt && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=/toolchains/toolchains.cmake \
        -DLOGURU_WITH_STREAMS=TRUE \
        -DCMAKE_INSTALL_PREFIX=${TARGET_FS}/usr \
        .. && \
    make -j$(nproc) install

# Build and install TensorFlow Lite for arm64
RUN git clone https://github.com/tensorflow/tensorflow.git /third_party/tensorflow && \
    cd /third_party/tensorflow && git checkout v2.10.0 && git submodule update --init --recursive --depth 1 tensorflow/lite
RUN cd /third_party/tensorflow/tensorflow/lite && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=/toolchains/toolchains.cmake \
        -DCMAKE_INSTALL_PREFIX=${TARGET_FS}/usr \
        -DCMAKE_INSTALL_LIBDIR=lib \
        -DCMAKE_INSTALL_INCLUDEDIR=include \
        -DTFLITE_ENABLE_INSTALL=ON \
        -DTFLITE_BUILD_SHARED_LIBS=OFF \
        -DTFLITE_ENABLE_XNNPACK=ON \
        -DTFLITE_KERNEL_TEST=OFF \
        -DTFLITE_HOST_TOOLS_DIR=/usr/bin/flatc \
        -DTFLITE_ENABLE_GPU=ON \
        -DTFLITE_BUILD_BENCHMARK=OFF \
        -DTFLITE_ENABLE_BENCHMARKS=OFF \
        -DTFLITE_BUILD_TESTS=OFF \
        -DTFLITE_PYTHON_WRAPPER_BUILD_CMAKE=OFF \
        -DTFLITE_BUILD_PYTHON=OFF \
        .. && \
    make -j$(nproc) install && \
    mkdir -p ${TARGET_FS}/usr/include/tensorflow && \
    cp -a /third_party/tensorflow/tensorflow/lite ${TARGET_FS}/usr/include/tensorflow/ && \
    cp -a \
      libtensorflow-lite.a \
      _deps/xnnpack-build/libXNNPACK.a \
      pthreadpool/libpthreadpool.a \
      _deps/cpuinfo-build/libcpuinfo.a \
      _deps/flatbuffers-build/libflatbuffers.a \
      _deps/farmhash-build/libfarmhash.a \
      _deps/clog-build/libclog.a \
      _deps/fft2d-build/libfft2d_fftsg*.a \
      _deps/ruy-build/ruy/*.a \
      _deps/ruy-build/ruy/profiler/*.a \
      ${TARGET_FS}/usr/lib/

# Build and install Rerun C++ SDK for arm64
# Install a recent Rust toolchain (for Cargo lockfile v4 support)
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y --profile minimal --default-toolchain 1.82.0
ENV PATH=/root/.cargo/bin:${PATH}
ENV RUSTUP_TOOLCHAIN=1.84.0
ENV CARGO=/root/.cargo/bin/cargo
ENV RUSTC=/root/.cargo/bin/rustc
RUN /root/.cargo/bin/rustup toolchain install 1.84.0 --target aarch64-unknown-linux-gnu && \
    /root/.cargo/bin/rustup component add rust-std --toolchain 1.84.0 --target aarch64-unknown-linux-gnu && \
    mkdir -p /root/.cargo && \
    printf "[target.aarch64-unknown-linux-gnu]\nlinker = \"/l4t/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-gcc\"\nar = \"/l4t/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-ar\"\n" > /root/.cargo/config.toml
RUN git clone --depth 1 --branch 0.23.3 https://github.com/rerun-io/rerun.git /third_party/rerun && \
    cd /third_party/rerun && \
    # Disable building rerun_c, examples and tests to avoid extra deps
    sed -i -E 's/^[[:space:]]*add_subdirectory\(([^)]*examples[^)]*)\)/# add_subdirectory(\1)/I' CMakeLists.txt && \
    sed -i -E 's/^[[:space:]]*add_subdirectory\(([^)]*test[^)]*)\)/# add_subdirectory(\1)/I' CMakeLists.txt && \
    sed -i -E 's/^[[:space:]]*add_subdirectory\(([^)]*docs[^)]*)\)/# add_subdirectory(\1)/I' CMakeLists.txt && \
    rm -rf rerun_cpp/tests && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=/toolchains/toolchains.cmake \
        -DCMAKE_INSTALL_PREFIX=${TARGET_FS}/usr \
        .. && \
    CARGO_BUILD_TARGET=aarch64-unknown-linux-gnu \
    CC_aarch64_unknown_linux_gnu=/l4t/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-gcc \
    AR_aarch64_unknown_linux_gnu=/l4t/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-ar \
    make -j$(nproc) && \
    shopt -s nullglob && \
    for lib in $(find /third_party/rerun -maxdepth 8 -type f \( -name "libarrow*.a" -o -name "libparquet*.a" -o -name "libarrow*.so*" -o -name "libparquet*.so*" \) 2>/dev/null); do \
      echo "COPY: $lib -> ${TARGET_FS}/usr/lib/"; \
      cp -a "$lib" ${TARGET_FS}/usr/lib/ || true; \
    done && \
    shopt -u nullglob && \
    cp -a /third_party/rerun/build/rerun_cpp/librerun_sdk.a ${TARGET_FS}/usr/lib/ && \
    cp -a /third_party/rerun/target/aarch64-unknown-linux-gnu/release/librerun_c.a ${TARGET_FS}/usr/lib/ && \
    mkdir -p ${TARGET_FS}/usr/include && \
    cp -a /third_party/rerun/rerun_cpp/src/rerun.hpp ${TARGET_FS}/usr/include/ && \
    cp -a /third_party/rerun/rerun_cpp/src/rerun ${TARGET_FS}/usr/include/

# Clone and build booster SDK for arm64
# Upstream currently has no 'install' target; build and copy artifacts manually
RUN git clone https://github.com/BoosterRobotics/booster_robotics_sdk.git /tmp/booster-sdk && \
    cd /tmp/booster-sdk && \
    git checkout 75e38b19f98becfcbf298b1c1a04171c2e7d8b8f && \
    mkdir -p ${TARGET_FS}/usr/lib ${TARGET_FS}/usr/include && \
    cp -a /tmp/booster-sdk/third_party/include/. ${TARGET_FS}/usr/include/ && \
    cp -a /tmp/booster-sdk/third_party/lib/aarch64/libfastcdr.so* ${TARGET_FS}/usr/lib/ && \
    cp -a /tmp/booster-sdk/third_party/lib/aarch64/libfastrtps.so* ${TARGET_FS}/usr/lib/ && \
    cp -a /tmp/booster-sdk/third_party/lib/aarch64/libfoonathan_memory-0.7.3.a ${TARGET_FS}/usr/lib/ && \
    cp -a /tmp/booster-sdk/lib/aarch64/libbooster_robotics_sdk.a ${TARGET_FS}/usr/lib/ && \
    cp -a /tmp/booster-sdk/include/. ${TARGET_FS}/usr/include/ && \
    rm -rf /tmp/booster-sdk

# Patch FindCUDA to skip validation in cross-compilation
RUN sed -i '1264,1273d' /usr/share/cmake-3.22/Modules/FindCUDA.cmake && \
    sed -i '1263a\set(CUDA_FOUND TRUE)' /usr/share/cmake-3.22/Modules/FindCUDA.cmake

# Build and install librealsense2 for arm64
RUN git clone --depth 1 --branch v2.55.1 https://github.com/IntelRealSense/librealsense.git /third_party/librealsense && \
    cd /third_party/librealsense && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=/toolchains/toolchains.cmake \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_WITH_CUDA=ON \
        -DBUILD_WITH_OPENMP=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DFORCE_LIBUVC=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6 \
        -DCUDA_CUDART_LIBRARY=/usr/local/cuda-12.6/targets/aarch64-linux/lib/libcudart.so \
        -DCUDA_cudart_static_LIBRARY=/usr/local/cuda-12.6/targets/aarch64-linux/lib/libcudart_static.a \
        -DCUDA_cudadevrt_LIBRARY=/usr/local/cuda-12.6/targets/aarch64-linux/lib/libcudadevrt.a \
        -DCUDA_INCLUDE_DIRS=/usr/local/cuda-12.6/targets/aarch64-linux/include \
        -DCUDA_HOST_COMPILER=/l4t/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-g++ \
        -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc \
        -DCMAKE_CUDA_ARCHITECTURES=87 \
        -DCMAKE_INSTALL_PREFIX=${TARGET_FS}/usr \
        .. && \
    make -j$(nproc) install

# Install ZED SDK
RUN mkdir -p /zed2  && \
    wget -nv "https://download.stereolabs.com/zedsdk/5.0/l4t36.4/jetsons" -O /zed2/ZED_SDK_Tegra_L4T36.3_v5.0.1.zstd.run && \
    chmod +x /zed2/ZED_SDK_Tegra_L4T36.3_v5.0.1.zstd.run && \
    /zed2/ZED_SDK_Tegra_L4T36.3_v5.0.1.zstd.run --noexec --target /l4t/targetfs/usr/ && \
    rm -rf /zed2 && \
    rm -rf /var/lib/apt/lists/*

RUN rm -rf /tmp/* /third_party /opt/nvidia /root/.cargo /root/.rustup \
    /root/.rustup.sh /usr/local/cuda-12.6/targets/x86_64-linux \
    /l4t/targetfs/var/l4t-cuda-tegra-repo-ubuntu2204-12-6-local \
    /l4t/targetfs/var/cudnn-local-tegra-repo-ubuntu2204-9.3.0 \
    /l4t/targetfs/var/nv-tensorrt-local-tegra-repo-ubuntu2204-10.3.0-cuda-12.5 \
    /l4t/targetfs/opt/nvidia /l4t/targetfs/usr/lib/firmware \
    /l4t/targetfs/usr/lib/aarch64-linux-gnu/dri \
    /l4t/targetfs/usr/lib/libreoffice /l4t/targetfs/usr/lib/thunderbird \
    /l4t/targetfs/usr/lib/python3 /l4t/targetfs/opt \
    /l4t/targetfs/usr/include/tensorflow/lite/build \
    /l4t/targetfs/usr/share/AAVMF /l4t/targetfs/usr/share/icons \
    /l4t/targetfs/usr/share/fonts /l4t/targetfs/usr/share/help \
    /l4t/targetfs/var/lib/dpkg /l4t/targetfs/var/cache \
    /l4t/targetfs/boot /var/lib/apt

