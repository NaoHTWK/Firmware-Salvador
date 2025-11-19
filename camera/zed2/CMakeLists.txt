cmake_minimum_required(VERSION 3.5)
project(camera_zed2 LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(ZED_DIR "/l4t/targetfs/usr/")

find_package(CUDAToolkit REQUIRED) # Find CUDA toolkit

find_package(OpenCV REQUIRED)

link_directories("/l4t/targetfs/usr/lib/")
include_directories("/l4t/targetfs/usr/include/")

add_library(camera_zed2
    zed2.cpp
    zed2.h
)

target_link_directories(camera_zed2 PUBLIC 
    "/l4t/targetfs/usr/lib/aarch64-linux-gnu/nvidia/"
    "/l4t/targetfs/usr/lib/gcc/aarch64-linux-gnu/11"
)

target_include_directories(camera_zed2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS} 
)
target_link_libraries(camera_zed2
    PUBLIC 
    utils 
    fluxlog_vision
    fluxlog
    sl_zed
    CUDA::cudart_static
    ${OpenCV_LIBS}
)

target_compile_features(camera_zed2 INTERFACE cxx_std_17)
target_compile_options(camera_zed2 PRIVATE -std=c++17)

set_target_properties(camera_zed2 PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED 17
  CUDA_STANDARD 12 # Set CUDA standard
  CUDA_SEPARABLE_COMPILATION ON # Enable separable compilation for CUDA
  CUDA_RUNTIME_LIBRARY Shared # Use shared runtime library for CUDA
  POSITION_INDEPENDENT_CODE ON # Enable position-independent code for shared libraries
  CUDA_ARCHITECTURES 87
  CUDA_RESOLVE_DEVICE_SYMBOLS ON # Enable device symbol resolution
)

message(STATUS "C++ standard is ${CMAKE_CXX_STANDARD}")
