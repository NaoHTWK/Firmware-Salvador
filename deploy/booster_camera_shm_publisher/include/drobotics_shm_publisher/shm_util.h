#pragma once

#include <cstdint>

namespace booster_camera {

const char* const RGB_SHM_NAME = "/booster_camera_rgb_shm";
const char* const DEPTH_SHM_NAME = "/booster_camera_depth_shm";
const char* const RGB_SEM_NAME = "/booster_camera_rgb_sem";
const char* const DEPTH_SEM_NAME = "/booster_camera_depth_sem";

struct ShmHeader {
    uint64_t timestamp_sec;
    uint64_t timestamp_nanosec;
    uint32_t width;
    uint32_t height;
    uint32_t step;
    uint32_t encoding_len;
    // flexible array member for encoding string
    char encoding[];
};

}  // namespace booster_camera
