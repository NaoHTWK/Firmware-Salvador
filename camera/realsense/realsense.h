#pragma once

#include <librealsense2/rs.hpp>
#include <memory>
#include <thread>

#include "../camera_connector.h"
#include "camera_pub_sub.h"
#include "sensor_pub_sub.h"

class Realsense : public htwk::CameraConnector {
public:
    Realsense();
    ~Realsense();

    void start() override;
    void stop() override;

private:
    std::shared_ptr<Image> get_image() override;

    void publishFrame();

    rs2::pipeline pipeline;
    rs2::config config;
    bool running;
    std::thread cam_thread;
    rs2::align align_to_color;
    rs2::align align_to_depth;
    float depth_scale;
};
