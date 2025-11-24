#pragma once

#include <semaphore.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <thread>

#include "../camera_connector.h"
#include "ipc/booster_camera_shm.h"

namespace booster_camera {
struct ShmHeader;
}

class Booster_Camera : public htwk::CameraConnector {
public:
    Booster_Camera();
    ~Booster_Camera() override;

    void start() override;
    void stop() override;
    void start_ros();

private:
    std::shared_ptr<Image> get_image() override;

    void rgb_thread_func();
    void depth_thread_func();

    struct ShmReader {
        ShmReader(const char* shm_name, const char* sem_name);
        ~ShmReader();
        void* ptr = nullptr;
        int fd = -1;
        sem_t* sem = SEM_FAILED;
        size_t size = 0;
    };

    std::unique_ptr<ShmReader> rgb_reader;
    std::unique_ptr<ShmReader> depth_reader;
    uint64_t frame_counter = 0;

    std::thread rgb_thread;
    std::thread depth_thread;
    std::atomic<bool> running{false};

    std::mutex mtx;
    std::condition_variable cv;

    // Buffer for latest images
    cv::Mat latest_rgb_yuv;
    std::unique_ptr<booster_camera::ShmHeader> latest_rgb_header;

    cv::Mat latest_depth;
    std::unique_ptr<booster_camera::ShmHeader> latest_depth_header;
    bool new_frame = false;
};
