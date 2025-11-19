project(tflite-benchmark)

find_package(Boost CONFIG COMPONENTS program_options filesystem system REQUIRED QUIET)

add_executable(tflite-benchmark tflite-benchmark.cpp)

target_link_libraries(
    tflite-benchmark
    Vision
    ImageTools
    Boost::program_options
)

set_target_properties(tflite-benchmark PROPERTIES CXX_STANDARD 17)
