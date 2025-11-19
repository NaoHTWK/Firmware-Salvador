#include "zed2.h"

#include <opencv2/core.hpp>
#include <random>
#include <sl/Camera.hpp>
#include <vector>

#include "../pub_sub/image.h"
#include "logging.h"
#include "named_thread.h"
#include "odometer.h"
#include "quaternion.h"
#include "robot_time.h"

inline cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE::F32_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE::F32_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE::F32_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE::U8_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE::U8_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE::U8_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE::U8_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }

    return cv::Mat(input.getHeight(), input.getWidth(), cv_type,
                   input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

Zed2::Zed2() : zed(new sl::Camera()), default_image_size(new sl::Resolution()) {}

Zed2::~Zed2() {
    stop();
}

void Zed2::start() {
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution =
            sl::RESOLUTION::HD720;    // Use HD720 opr HD1200 video mode, depending on camera type.
    init_parameters.camera_fps = 30;  // Set fps at 30
    init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;  // check other modes
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

    init_parameters.async_grab_camera_recovery = true;

    // Open the camera
    auto returned_state = this->zed->open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        LOG_F(ERROR, "Camera Open failed with error code: %s",
              sl::toString(returned_state).c_str());
        exit(EXIT_FAILURE);
    }

    LOG_F(INFO, "Zed2 camera opened successfully");
    running = true;

    LOG_F(INFO, "Starting Zed2 camera thread");
    *default_image_size =
            zed->getRetrieveMeasureResolution(sl::Resolution(CAMERA_WIDTH, CAMERA_HEIGHT));
    auto calibration_parameters =
            zed->getCameraInformation().camera_configuration.calibration_parameters;
    fx = calibration_parameters.left_cam.fx;
    fy = calibration_parameters.left_cam.fy;
    cx = calibration_parameters.left_cam.cx;
    cy = calibration_parameters.left_cam.cy;
}

std::shared_ptr<Image> Zed2::get_image() {
    static int frame_count = 0; 
    static uint64_t frame_id = 0;//used for frame counting
    static int64_t grab_time_acc = 0;
    static int64_t retrieve_data_time_acc = 0;
    static int64_t processing_time_acc = 0;
    static int64_t last_frame_log_time = time_us();

    sl::Mat zed_image;
    sl::Mat zed_depth;
    // sl::Mat zed_depth_image;
    // sl::SensorsData sensors_data;
    // sl::SensorsData::IMUData imu_data;
    // sl::Pose zed_pose;
    // sl::Mat pc;

    int64_t start_time = time_us();
    sl::ERROR_CODE returned_state = zed->grab();
    int64_t grab_time = time_us() - start_time;
    grab_time_acc += grab_time;

    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        LOG_F(ERROR, "Zed2 grab failed with error code: %s", sl::toString(returned_state).c_str());
        return nullptr;
    }
    start_time = time_us();
    zed->retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU, *default_image_size);
    zed->retrieveMeasure(zed_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, *default_image_size);

    int64_t retrieve_data_time = time_us() - start_time;
    retrieve_data_time_acc += retrieve_data_time;

    start_time = time_us();
    std::shared_ptr<Image> image = std::make_shared<Image>();
    image->timestamp_us = time_us();

    //set constants
    image->frame_id = frame_id;
    frame_id++;
    image->fx = fx;
    image->fy = fy;
    image->cx = cx;
    image->cy = cy;

    int width = zed_image.getWidth();
    int height = zed_image.getHeight();
    if (width != CAMERA_WIDTH || height != CAMERA_HEIGHT) {
        LOG_F(ERROR, "Unexpected color frame size: %dx%d (expected %dx%d)", width, height, CAMERA_WIDTH, CAMERA_HEIGHT);
        return nullptr;
    }

    {
        cv::Mat bgra = slMat2cvMat(zed_image);
        if(bgra.channels() !=4) {
            LOG_F(ERROR, "Unexpected number of channels in color frame: %d (expected 4)", bgra.channels());
            return nullptr;
        }
        std::memcpy(image->data_bgra.data(), bgra.data, Image::BGRA_SIZE);
        bgra.u = nullptr;  // prevent cv::Mat from freeing the data
    }
    
    //set data depth m
    {
        cv::Mat depth = slMat2cvMat(zed_depth);
        if(depth.channels() !=1) {
            LOG_F(ERROR, "Unexpected number of channels in depth frame: %d (expected 1)", depth.channels());
            return nullptr;
        }
        std::memcpy(image->data_depth_m.data(), depth.data, Image::DEPTH_SIZE * sizeof(float));

        depth.u = nullptr;  // prevent cv::Mat from freeing the data
    }

    //set data depth scaled
    for (int i = 0; i < Image::DEPTH_SIZE; ++i) {
        //if NaN
        if (std::isnan(image->data_depth_m[i])) {
            image->data_depth_scaled[i] = 0;
        } else {
            image->data_depth_scaled[i] = static_cast<uint16_t>(image->data_depth_m[i] / Image::DEPTH_SCALE);
        }
    }

    int64_t processing_time = time_us() - start_time;
    processing_time_acc += processing_time;

    frame_count++;
    int64_t now = time_us();
    if (now - last_frame_log_time > 1'000'000) {
        if (frame_count > 0) {
            LOG_F(1,
                  "zed2 avg times (ms): grab: %.2f, retrieve_data: %.2f, processing: %.2f, "
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

    // LOG_F(INFO, "Odo: %f, %f, %f", odo.x, odo.y, odo.theta);

    return image;
}

void Zed2::stop() {
    LOG_F(INFO, "Stopping Zed2 camera");

    running = false;
    if (zed->isOpened()) {
        LOG_F(INFO, "Closing Zed2 camera");
        zed->close();
    } else {
        LOG_F(INFO, "Zed2 camera was not opened");
    }
}
