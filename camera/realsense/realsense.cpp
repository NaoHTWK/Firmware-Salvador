#include "realsense.h"

#include <chrono>
#include <iostream>

#include "logging.h"
#include "named_thread.h"
#include "robot_time.h"

Realsense::Realsense()
    : running(false),
      align_to_color(rs2::align(RS2_STREAM_COLOR)),
      align_to_depth(rs2::align(RS2_STREAM_DEPTH)) {
    LOG_F(INFO, "Realsense constructor called");
}

Realsense::~Realsense() {
    LOG_F(INFO, "Realsense destructor called");
    stop();
}

void Realsense::start() {
    LOG_F(INFO, "RealSense SDK version: %s", RS2_API_VERSION_STR);
    if (running) {
        LOG_F(INFO, "Camera already running, skipping start");
        return;
    }

    LOG_F(INFO, "Logging depth image white is 0m, black is 25m");

    rs2::context ctx;
    auto devices = ctx.query_devices();
    LOG_F(INFO, "Found %d devices", devices.size());

    if (devices.size() == 0) {
        LOG_F(ERROR, "No RealSense devices found");
        return;
    }

    // Skip hardware reset - it often causes issues with power management
    LOG_F(INFO, "Skipping hardware reset to avoid power management issues");
    
    auto dev = devices[0];
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    config.enable_device(serial);
    LOG_F(INFO, "Using RealSense device with serial: %s", serial.c_str());
    
    config.enable_stream(RS2_STREAM_COLOR, CAMERA_WIDTH, CAMERA_HEIGHT, RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, DEPTH_CAMERA_WIDTH, DEPTH_CAMERA_HEIGHT, RS2_FORMAT_Z16, 30);
    LOG_F(INFO, "Enabled color and depth streams");
    
    try {
        auto profile = pipeline.start(config);

        // Get depth scale for converting raw depth values to meters
        auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
        depth_scale = depth_sensor.get_depth_scale();
        LOG_F(INFO, "Depth scale: %f meters per unit", depth_scale);

        running = true;
        LOG_F(INFO, "Realsense pipeline started");
    } catch (const rs2::error& e) {
        LOG_F(ERROR, "RealSense error: %s (function: %s)", e.what(),
              e.get_failed_function().c_str());
        return;
    }
}

void Realsense::stop() {
    LOG_F(INFO, "Stopping RealSense camera");

    running = false;
    pipeline.stop();
}

std::shared_ptr<Image> Realsense::get_image() {
    //prepare timing stats
    static int frame_count = 0; //used for fps calculation
    static uint64_t frame_id = 0;//used for frame counting
    static int64_t grab_time_acc = 0;
    static int64_t retrieve_data_time_acc = 0;
    static int64_t processing_time_acc = 0;
    static int64_t last_frame_log_time = time_us();

    int64_t start_time = time_us();

    //get image
    rs2::frameset frames = pipeline.wait_for_frames(1000);

    int64_t grab_time = time_us() - start_time;
    grab_time_acc += grab_time;

    start_time = time_us();

    frames = align_to_color.process(frames);
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame depth_frame = frames.get_depth_frame();
    if (!color_frame) {
        // Don't log warning during shutdown
        if (running) {
            LOG_F(ERROR, "No color frame available from realsense camera");
            return nullptr;
        }
    }
    if (!depth_frame) {
        // Don't log warning during shutdown
        if (running) {
            LOG_F(ERROR, "No depth frame available from realsense camera");
            return nullptr;
        }
    }
    int64_t retrieve_data_time = time_us() - start_time;
    retrieve_data_time_acc += retrieve_data_time;

    start_time = time_us();
    std::shared_ptr<Image> image = std::make_shared<Image>();
    rs2::video_frame video_frame = color_frame.as<rs2::video_frame>();

    //set constants
    image->frame_id = frame_id;
    frame_id++;
    
    rs2_intrinsics intrinsics =
            video_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    image->fx = intrinsics.fx;
    image->fy = intrinsics.fy;
    image->cx = intrinsics.ppx;
    image->cy = intrinsics.ppy;

    int width = video_frame.get_width();
    int height = video_frame.get_height();
    if (width != CAMERA_WIDTH || height != CAMERA_HEIGHT) {
        LOG_F(WARNING, "Unexpected color frame size: %dx%d (expected %dx%d)", width, height, CAMERA_WIDTH, CAMERA_HEIGHT);
        return nullptr;
    }

    image->timestamp_us = start_time;
    image->timestamp_camera = color_frame.get_timestamp() * 1'000;

    //set color image bgr
    const uint8_t* data = static_cast<const uint8_t*>(color_frame.get_data());
    size_t data_size = color_frame.get_data_size();
    if (Image::BGR_SIZE != data_size) {
        LOG_F(ERROR, "Unexpected color frame data size: %zu (expected %d)", data_size,
              Image::BGR_SIZE);
        return nullptr;
    }

    //set color image bgra / yuv
    {
        cv::Mat bgr = cv::Mat(height, width, CV_8UC3, (void*)data);
        cv::Mat bgra = cv::Mat(height, width, CV_8UC4, image->data_bgra.data());

        cv::cvtColor(cv::_InputArray(bgr), bgra, cv::COLOR_BGR2BGRA);
        bgr.u = nullptr;  // prevent cv::Mat from freeing the data
        bgra.u = nullptr;  // prevent cv::Mat from freeing the data
    }

    // set 16 unint depth data
    const uint16_t* depth_data = static_cast<const uint16_t*>(depth_frame.get_data());

    std::memcpy(image->data_depth_scaled.data(), depth_data,
                DEPTH_CAMERA_WIDTH * DEPTH_CAMERA_HEIGHT * sizeof(uint16_t));
    
    //set float depth data
    for (size_t i = 0; i < Image::DEPTH_SIZE; ++i) {
        if (depth_data[i] == 0) {
            // If depth data is zero, set to NaN
            image->data_depth_m[i] = std::numeric_limits<float>::quiet_NaN();
        } else {
            image->data_depth_m[i] = depth_data[i] * Image::DEPTH_SCALE;
        }
    }

    int64_t processing_time = time_us() - start_time;
    processing_time_acc += processing_time;

    frame_count++;
    int64_t now = time_us();
    if (now - last_frame_log_time > 1'000'000) {
        if (frame_count > 0) {
            LOG_F(1,
                  "realsense avg times (ms): grab: %.2f, retrieve_data: %.2f, processing: %.2f, "
                  "fps: %d",
                  grab_time_acc / (float)frame_count / 1000.f,
                  retrieve_data_time_acc / (float)frame_count / 1000.f,
                  processing_time_acc / (float)frame_count / 1000.f, frame_count);
        }
        last_frame_log_time += 1'000'000;
        frame_count = 0;
        grab_time_acc = 0;
        retrieve_data_time_acc = 0;
        processing_time_acc = 0;
    }

    return image;
}
