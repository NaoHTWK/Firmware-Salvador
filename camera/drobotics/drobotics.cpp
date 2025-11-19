#include "drobotics.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "image.h"
#include "ipc/drobotics_shm.h"
#include "logging.h"
#include "robot_time.h"

Drobotics::ShmReader::ShmReader(const char* shm_name, const char* sem_name) {
    fd = shm_open(shm_name, O_RDONLY, 0666);
    if (fd == -1) {
        throw std::runtime_error("shm_open failed: " + std::string(strerror(errno)));
    }

    struct stat shm_stat;
    if (fstat(fd, &shm_stat) == -1) {
        close(fd);
        throw std::runtime_error("fstat failed: " + std::string(strerror(errno)));
    }
    size = shm_stat.st_size;

    ptr = mmap(0, size, PROT_READ, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        close(fd);
        throw std::runtime_error("mmap failed: " + std::string(strerror(errno)));
    }

    sem = sem_open(sem_name, 0);
    if (sem == SEM_FAILED) {
        munmap(ptr, size);
        close(fd);
        throw std::runtime_error("sem_open failed: " + std::string(strerror(errno)));
    }
}

Drobotics::ShmReader::~ShmReader() {
    if (ptr)
        munmap(ptr, size);
    if (fd != -1)
        close(fd);
    if (sem != SEM_FAILED)
        sem_close(sem);
}

Drobotics::Drobotics() = default;

Drobotics::~Drobotics() {
    stop();
}

void Drobotics::rgb_thread_func() {
    while (running) {
        sem_wait(rgb_reader->sem);
        if (!running)
            break;

        drobotics::ShmHeader* header = static_cast<drobotics::ShmHeader*>(rgb_reader->ptr);
        char* encoding_ptr = header->encoding;
        uint8_t* data_ptr = reinterpret_cast<uint8_t*>(encoding_ptr + header->encoding_len);

        {
            std::lock_guard<std::mutex> lock(mtx);
            latest_rgb_header = std::make_unique<drobotics::ShmHeader>(*header);
            latest_rgb_yuv = cv::Mat(header->height * 1.5, header->width, CV_8UC1, data_ptr).clone();
            new_frame = true;
            cv.notify_one();
        }
    }
}

void Drobotics::depth_thread_func() {
    while (running) {
        sem_wait(depth_reader->sem);
        if (!running)
            break;

        drobotics::ShmHeader* header = static_cast<drobotics::ShmHeader*>(depth_reader->ptr);
        char* encoding_ptr = header->encoding;
        uint8_t* data_ptr = reinterpret_cast<uint8_t*>(encoding_ptr + header->encoding_len);

        {
            std::lock_guard<std::mutex> lock(mtx);
            latest_depth_header = std::make_unique<drobotics::ShmHeader>(*header);
            latest_depth = cv::Mat(header->height, header->width, CV_16UC1, data_ptr).clone();
            new_frame = true;
            cv.notify_one();
        }
    }
}

void Drobotics::start_ros() {
    LOG_F(INFO, "Starting camera service via restart_camera script...");

    // Execute the restart_camera binary with sudo (password: 123456)
    const char* cmd = "echo '123456' | sudo -S /home/booster/Workspace/restart_camera 2>&1";

    int result = system(cmd);

    if (result == 0) {
        LOG_F(INFO, "Camera restart command completed successfully.");
    } else {
        LOG_F(WARNING, "Camera restart failed (exit code: %d). Continuing anyway...", result);
    }

    // Start the ROS2 to Shared Memory bridge
    LOG_F(INFO, "Starting drobotics_shm_publisher bridge...");
    const char* bridge_cmd =
            "cd /home/booster/etc/drobotics_shm_publisher && ./start_shm_bridge.sh >/tmp/shm_bridge.log 2>&1 &";

    result = system(bridge_cmd);

    if (result == 0) {
        LOG_F(INFO, "Bridge start command executed. Check /tmp/shm_bridge.log for details.");
    } else {
        LOG_F(WARNING, "Bridge start command failed (exit code: %d). Continuing anyway...", result);
    }
}

void Drobotics::start() {
    start_ros();

    // Wait for bridge to create shared memory segments (retry for up to 10 seconds)
    const int max_retries = 20;
    const int retry_delay_ms = 500;
    int retry_count = 0;
    bool success = false;

    while (retry_count < max_retries && !success) {
        try {
            rgb_reader = std::make_unique<ShmReader>(drobotics::RGB_SHM_NAME, drobotics::RGB_SEM_NAME);
            depth_reader = std::make_unique<ShmReader>(drobotics::DEPTH_SHM_NAME, drobotics::DEPTH_SEM_NAME);
            LOG_F(INFO, "Drobotics camera connector started.");
            success = true;
        } catch (const std::runtime_error& e) {
            retry_count++;
            if (retry_count < max_retries) {
                LOG_F(INFO, "Waiting for shared memory to be ready... (attempt %d/%d)", retry_count, max_retries);
                usleep(retry_delay_ms * 1000); // Convert ms to microseconds
            } else {
                LOG_F(ERROR, "Failed to start Drobotics camera connector after %d attempts: %s", max_retries, e.what());
                exit(EXIT_FAILURE);
            }
        }
    }

    running = true;
    rgb_thread = std::thread(&Drobotics::rgb_thread_func, this);
    depth_thread = std::thread(&Drobotics::depth_thread_func, this);
}

void Drobotics::stop() {
    if (running) {
        running = false;
        // Unblock threads waiting on semaphores
        if (rgb_reader && rgb_reader->sem != SEM_FAILED)
            sem_post(rgb_reader->sem);
        if (depth_reader && depth_reader->sem != SEM_FAILED)
            sem_post(depth_reader->sem);

        cv.notify_all();
        if (rgb_thread.joinable()) {
            rgb_thread.join();
        }
        if (depth_thread.joinable()) {
            depth_thread.join();
        }
    }

    rgb_reader.reset();
    depth_reader.reset();
    LOG_F(INFO, "Drobotics camera connector stopped.");
}

std::shared_ptr<Image> Drobotics::get_image() {

    std::unique_lock<std::mutex> lock(mtx);
    // Frame matching logic
    bool good = false;
    while (running) {
        cv.wait(lock, [this] { return new_frame; });
        new_frame = false;
        if (latest_rgb_header && latest_depth_header &&
             latest_rgb_header->timestamp_sec == latest_depth_header->timestamp_sec &&
             latest_rgb_header->timestamp_nanosec == latest_depth_header->timestamp_nanosec) {
            good = true;
            break;
        }
    }

    // At this point, we have a matching pair
    cv::Mat bgr;
    cv::cvtColor(latest_rgb_yuv, bgr, cv::COLOR_YUV2BGR_NV12);

    auto image = std::make_shared<Image>();

    // Set timestamp
    image->timestamp_us =
            latest_rgb_header->timestamp_sec * 1000000 + latest_rgb_header->timestamp_nanosec / 1000;
    image->timestamp_camera = 0;
    image->last_image_time = image->timestamp_us;
    image->frame_id = frame_counter++;

    image->fx = 206;
    image->fy = 206;
    image->cx = 272;
    image->cy = 224;

    // Convert BGR to BGRA and store in data_bgra
    if (bgr.rows == Image::HEIGHT && bgr.cols == Image::WIDTH) {
        cv::Mat bgra(Image::HEIGHT, Image::WIDTH, CV_8UC4);
        cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
        std::memcpy(image->data_bgra.data(), bgra.data, Image::BGRA_SIZE);

        // Convert BGRA to YUV444
        Image::BGR2YUV444(image->data_yuv.data(), image->data_bgra.data(), Image::WIDTH * Image::HEIGHT, 4);
    } else {
        LOG_F(WARNING, "RGB image size mismatch: got %dx%d, expected %dx%d", bgr.cols, bgr.rows, Image::WIDTH,
              Image::HEIGHT);
    }

    // Store depth data
    image->depth_width = latest_depth.cols;
    image->depth_height = latest_depth.rows;
    image->depth_data.resize(latest_depth.cols * latest_depth.rows);
    std::memcpy(image->depth_data.data(), latest_depth.data, latest_depth.cols * latest_depth.rows * sizeof(uint16_t));

    // Convert depth to meters (assuming input is in millimeters)
    if (latest_depth.cols == Image::DEPTH_WIDTH && latest_depth.rows == Image::DEPTH_HEIGHT) {
        const uint16_t* depth_ptr = reinterpret_cast<const uint16_t*>(latest_depth.data);
        for (int i = 0; i < Image::DEPTH_SIZE; i++) {
            image->data_depth_scaled[i] = depth_ptr[i];
            image->data_depth_m[i] = depth_ptr[i] * Image::DEPTH_SCALE;
        }
    } else {
        LOG_F(WARNING, "Depth image size mismatch: got %dx%d, expected %dx%d", latest_depth.cols, latest_depth.rows,
              Image::DEPTH_WIDTH, Image::DEPTH_HEIGHT);
    }

    new_frame = false;

    image->log_image_ = false;
    image->log_depth_image_ = false;

    return image;
}
