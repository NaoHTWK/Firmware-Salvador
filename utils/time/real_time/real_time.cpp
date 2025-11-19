#include "robot_time.h"

int64_t time_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
}