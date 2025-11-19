#pragma once
#include <chrono>
#include <string>
#include "logging.h"


namespace htwk {
struct Timer {
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
    using NanoSeconds = std::chrono::nanoseconds;
    
    TimePoint start_time;
    TimePoint end_time;
    bool is_running = false;
    bool is_started = false;
    std::string name;

    Timer(const std::string& timer_name = "nameless Timer") : name(timer_name) {}

    void start() {
        start_time = std::chrono::high_resolution_clock::now();
        is_running = true;
        is_started = true;
    }

    void stop() {
        end_time = std::chrono::high_resolution_clock::now();
        is_running = false;
    }

    void log(){
        if (!is_started) {
            LOG_F(ERROR, "Timer '%s' has not been started. Please start it before logging.", name.c_str());
            return;
        }
        if (is_running) {
            LOG_F(ERROR, "Timer '%s' is not running. Please start it before logging.", name.c_str());
            return;
        }
        LOG_F(INFO, "'%s': elapsed time: %ld ns", name.c_str(),
              std::chrono::duration_cast<NanoSeconds>(end_time - start_time).count()
        );
    }
}
}