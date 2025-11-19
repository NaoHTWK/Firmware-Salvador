project(trt_detect LANGUAGES CUDA CXX )

find_package(CUDAToolkit REQUIRED) # Find CUDA toolkit
find_package(OpenCV REQUIRED)

add_library(trt_detect
  impl.h
  impl.cpp
)

target_include_directories(trt_detect PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} 
    /l4t/targetfs/usr/local/cuda/include
    #/usr/local/cuda/targets/aarch64-linux/include
    #/usr/include/aarch64-linux-gnu  # TensorRT headers
    ${OpenCV_INCLUDE_DIRS}
)

# target_link_directories(vision_booster_vision PUBLIC
#   /usr/local/cuda/targets/aarch64-linux/lib
# )

target_link_libraries(trt_detect PUBLIC
    # cuda # Link CUDA libraries
    CUDA::cudart # Link CUDA runtime library
    nvinfer  # TensorRT core library
    nvinfer_plugin  # TensorRT plugin library
    vision_pub_sub
    loguru
    time
    ${OpenCV_LIBS}
)

set_target_properties(trt_detect PROPERTIES
    CXX_STANDARD 17
    CUDA_STANDARD 17 # Set CUDA standard
    CUDA_SEPARABLE_COMPILATION ON # Enable separable compilation for CUDA
    CUDA_RUNTIME_LIBRARY Shared # Use shared runtime library for CUDA
    POSITION_INDEPENDENT_CODE ON # Enable position-independent code for shared libraries
    CUDA_ARCHITECTURES 87
    CUDA_RESOLVE_DEVICE_SYMBOLS ON # Enable device symbol resolution
)
