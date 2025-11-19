# Set CUDA as a language for projekt.
# NEEDS TO BE *BEFORE* CXX
project(cuda-test LANGUAGES CUDA CXX)

# Find libraries to link for C++
find_package(CUDAToolkit)

# just add the .cu files to the list of sources
add_executable(cuda-test
  test.cu
)

# Just link required cuda libraries here
target_link_libraries(cuda-test PUBLIC CUDA::cudart)

# Set to be separatly compiled, not required but possible
set_target_properties(cuda-test PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
