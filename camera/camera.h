#pragma once

#include <sensor_pub_sub.h>

#include <memory>
#include <thread>

#include "cam_pose.h"
#include "camera_connector.h"
#include "pub_sub/image.h"
#include "robot_time.h"

namespace htwk {

/** Abstract camera class. wraps both zed2 an realsense

 */
class Camera {

public:
    template <class T, typename... Args,
              typename std::enable_if<std::is_base_of<CameraConnector, T>::value>::type* = nullptr>
    static std::unique_ptr<Camera> create_camera(Args... args) {
        std::unique_ptr<CameraConnector> connector = std::make_unique<T>(args...);
        return std::unique_ptr<Camera>(new Camera(std::move(connector)));
    }

    void start();
    void stop();

protected:
    void publishFrame();

    htwk::ChannelSubscriber<std::shared_ptr<IMUJointState>> imu_joint_states_subscriber =
            htwk::imu_joint_states_channel.create_subscriber();

private:
    Camera(std::unique_ptr<CameraConnector> connector);
    std::unique_ptr<CameraConnector> connector;

    std::thread cam_thread;
    bool running = false;

    CamPose last_cam_pose;
    int64_t last_image_time = time_us();

    int image_counter = 0;  // Counter for images, used for logging
};

std::unique_ptr<Camera> choose_camera();

}  // namespace htwk
