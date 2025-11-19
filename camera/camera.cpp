#include "camera.h"

#include "named_thread.h"
#include "pub_sub/camera_pub_sub.h"

#if defined(ZED2_CAMERA)

#include <zed2.h>

std::unique_ptr<htwk::Camera> htwk::choose_camera() {
    return htwk::Camera::create_camera<Zed2>();
}

#elif defined(REALSENSE_CAMERA)

#include <realsense.h>

std::unique_ptr<htwk::Camera> htwk::choose_camera() {
    return htwk::Camera::create_camera<Realsense>();
}

#elif defined(DROBOTICS_CAMERA)

#include <drobotics.h>

std::unique_ptr<htwk::Camera> htwk::choose_camera() {
    return htwk::Camera::create_camera<Drobotics>();
}

#else
#error "No valid camera selected"

std::unique_ptr<htwk::Camera> htwk::choose_camera() {
    return nullptr;
}

#endif

void htwk::Camera::start() {
    running = true;
    connector->start();

    cam_thread = named_thread("cam-thread", [this]() {
        LOG_F(INFO, "cam-thread started, initializing");
        while (running) {
            publishFrame();
        }
    });
}

htwk::Camera::Camera(std::unique_ptr<CameraConnector> connector)
    : connector(std::move(connector)) {}

void htwk::Camera::stop() {
    connector->stop();
    running = false;
}

void htwk::Camera::publishFrame() {
    auto img = connector->get_image();

    if (!img) {
        return;
    }

    auto imu_joint_states = imu_joint_states_subscriber.latest();
    if (!imu_joint_states) {
        LOG_F(WARNING, "no imu joint states");
        return;
    }

    MotorState head_yaw = imu_joint_states->serial[static_cast<size_t>(JointIndex::kHeadYaw)];
    MotorState head_pitch = imu_joint_states->serial[static_cast<size_t>(JointIndex::kHeadPitch)];
#if defined(ROBOT_MODEL_K1)
    img->cam_pose_ptr = std::make_shared<CamPose>(0.435f, imu_joint_states->imu.toOld().gyr,
                                                  YawPitch(head_yaw.q, head_pitch.q), img->HEIGHT,
                                                  img->fx, img->fy, img->cx, img->cy);
    float yaw_diff = head_yaw.q - last_cam_pose.head_angles.yaw;
    last_cam_pose = *img->cam_pose_ptr;
#elif defined(ROBOT_MODEL_T1)
    CamPose cam_pose(0.47f, imu_joint_states->imu.toOld().gyr, YawPitch(head_yaw.q, head_pitch.q),
                     img->HEIGHT, img->fx, img->fy, img->cx, img->cy);
    img->cam_pose_ptr = std::make_shared<CamPose>(cam_pose);

    float yaw_diff = cam_pose.head_angles.yaw - last_cam_pose.head_angles.yaw;
    last_cam_pose = cam_pose;
#endif
    Image::BGR2YUV444(img->data_yuv.data(), img->data_bgra.data(), CAMERA_WIDTH*CAMERA_HEIGHT, 4);
    img->last_image_time = last_image_time;
    last_image_time = img->timestamp_us;

    // decide if we want to log the image
    if (image_counter % 2 == 0) {
        img->log_image_ = true;
    } else {
        img->log_image_ = false;
    }

    if (image_counter % 30 == 0) {
        img->log_depth_image_ = true;
    } else {
        img->log_depth_image_ = false;
    }

    image_counter++;
    images.publish(img);
}
