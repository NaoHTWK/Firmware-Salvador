#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <vector>

#include "motion_pub_sub.h"
#include "position.h"
#include "sensor_pub_sub.h"

namespace htwk {

class OdometerProcessor {
public:
    OdometerProcessor();

    // Main processing method that handles all odometry calculations
    void proceed();

    // Get current pose
    const Position& getCurrentPose() const {
        return pose;
    }

    // Get pose history for visualization
    const std::vector<std::array<float, 2>>& getPoseHistory() const {
        return pose_history;
    }

    // Reset odometry
    void reset();

private:
    // pub subs
    htwk::ChannelSubscriber<std::shared_ptr<IMUJointState>> imu_joint_state_sub =
            imu_joint_states_channel.create_subscriber();
    htwk::ChannelSubscriber<MotionCommand> motion_command_sub =
            motion_command_channel.create_subscriber();

    // Calculate normalized yaw difference
    float calculateYawDiff(float current_yaw);

    // Process motion command and update pose
    void processMotionCommand(MotionCommand mc, float yaw_diff, float actual_dt);

    // Member variables
    float last_yaw = 0;
    bool last_yaw_init = false;
    htwk::Position pose{0, 0, 0};
    std::vector<std::array<float, 2>> pose_history;

    // Current actual velocities (for acceleration calculation)
    float current_vx = 0.0f;
    float current_vy = 0.0f;

    // Time tracking for accurate dt calculation
    std::chrono::high_resolution_clock::time_point last_time_point;
    bool first_call = true;

    // Configuration
    static constexpr float dt = 1.f / 50.f;  // 50 FPS (fallback value)
    static constexpr float ax = .3f;         // acceleration x (m/s^2)
    static constexpr float ay = .3f;         // acceleration y (m/s^2)
    static constexpr float decel_x = .8f;    // deceleration x (m/s^2) - faster than acceleration
    static constexpr float decel_y = .8f;    // deceleration y (m/s^2) - faster than acceleration
};

}  // namespace htwk