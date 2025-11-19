#pragma once
#include <cstdint>

struct Odometer {
    float x;
    float y;
    float theta;
    int64_t timestamp_system_us;
};