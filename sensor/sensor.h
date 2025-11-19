#pragma once
#include <booster/idl/b1/FallDownState.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/Odometer.h>

#include <booster/robot/channel/channel_subscriber.hpp>

#include "sensor_pub_sub.h"

namespace htwk {

class Sensor {
public:
    Sensor();
    ~Sensor();

private:
    void odoHandler_from_imu(const IMUJointState& imu_joint_state);
    float last_odo_yaw = 0.0f;

    void odoHandler(const void *msg);
    void lowHandler(const void *msg);
    void update_fall_down_state_booster(const void *msg);

    void update_fall_down_state(IMUJointState imu_joint_state);

    bool is_fallen(IMU_new imu) {
        return std::fabs(imu.rpy[0]) > 0.5f || std::fabs(imu.rpy[1]) > 0.5f;
    }

    htwk::FallDownSide get_fall_down_side(IMU_new imu) {
        return imu.rpy[1] > 0.f ? htwk::FallDownSide::FRONT : htwk::FallDownSide::BACK;
    }

    std::unique_ptr<booster::robot::ChannelSubscriber<booster_interface::msg::LowState>>
            channel_subscriber_low;
    std::unique_ptr<booster::robot::ChannelSubscriber<booster_interface::msg::Odometer>>
            channel_subscriber_odo;

    std::unique_ptr<booster::robot::ChannelSubscriber<booster_interface::msg::FallDownState>>
            channel_subscriber_fall_down;

    htwk::ChannelSubscriber<htwk::FallDownState> fallen_subscriber =
            htwk::fallen_channel.create_subscriber();

    int64_t last_getup_started = 0;

    int64_t front_get_up_time = 6.5_s;
    int64_t back_get_up_time = 6.5_s;
    float back_torque_limit = 3.f;
    float front_torque_limit = 3.f;

#ifdef ROBOT_MODEL_K1
    int joint_cnt = 22;
#else
    int joint_cnt = 23;
#endif
};

}  // namespace htwk