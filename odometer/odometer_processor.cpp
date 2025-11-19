#include "odometer_processor.h"

#include <logging.h>
#include <robot_time.h>

#include <chrono>
#include <cmath>
#include <rerun.hpp>

namespace htwk {

OdometerProcessor::OdometerProcessor() {
    // Constructor - initialize default values
}

void OdometerProcessor::proceed() {
    std::shared_ptr<IMUJointState> imu_joint_states = imu_joint_state_sub.latest();
    MotionCommand motion_command = motion_command_sub.latest();

    if (!imu_joint_states) {
        return;
    }

    // Get current timestamp and calculate actual dt
    auto current_time = std::chrono::high_resolution_clock::now();
    float actual_dt = dt;  // Use fallback dt for first call

    if (!first_call) {
        // Calculate actual time difference in seconds
        auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(current_time -
                                                                               last_time_point);
        actual_dt = time_diff.count() / 1000000.0f;
    } else {
        first_call = false;
    }

    last_time_point = current_time;

    // Initialize last_yaw on first call
    if (!last_yaw_init) {
        last_yaw = imu_joint_states->imu.rpy[2];
        last_yaw_init = true;
    }

    // Calculate normalized yaw difference
    // we don´t track the yaw_diff here because it is to slow to be tracked here
    // float yaw_diff = calculateYawDiff(imu_joint_states->imu.rpy[2]);

    // Update last_yaw for next iteration
    // last_yaw = imu_joint_states->imu.rpy[2];

    processMotionCommand(motion_command, 0, actual_dt);
}

float OdometerProcessor::calculateYawDiff(float current_yaw) {
    float yaw_diff = current_yaw - last_yaw;

    // Normalize angle difference to [-π, π]
    while (yaw_diff > M_PI) {
        yaw_diff -= 2.0 * M_PI;
    }
    while (yaw_diff < -M_PI) {
        yaw_diff += 2.0 * M_PI;
    }

    return yaw_diff;
}

void OdometerProcessor::processMotionCommand(MotionCommand mc, float yaw_diff, float actual_dt) {
    if (mc.type == MotionCommand::Type::WALK) {
        auto wr = mc.walk_request;

        // Apply speed limits to target velocities: max 1.5 m/s for x, max 1.2 m/s for y
        float target_vx = std::clamp(wr.dx, -1.5f, 1.5f);
        float target_vy = std::clamp(wr.dy, -1.2f, 1.2f);

        // Calculate velocity differences
        float vel_diff_x = target_vx - current_vx;
        float vel_diff_y = target_vy - current_vy;

        // Determine if we're decelerating or accelerating for each axis
        bool decelerating_x = (std::abs(target_vx) < std::abs(current_vx)) ||
                              (current_vx != 0 && (current_vx * vel_diff_x < 0));
        bool decelerating_y = (std::abs(target_vy) < std::abs(current_vy)) ||
                              (current_vy != 0 && (current_vy * vel_diff_y < 0));

        // Apply acceleration limits to velocity changes
        float max_vel_change_x = (decelerating_x ? decel_x : ax) * actual_dt;
        float max_vel_change_y = (decelerating_y ? decel_y : ay) * actual_dt;

        // Clamp velocity changes to acceleration limits
        float vel_change_x = std::clamp(vel_diff_x, -max_vel_change_x, max_vel_change_x);
        float vel_change_y = std::clamp(vel_diff_y, -max_vel_change_y, max_vel_change_y);

        // Update current velocities
        current_vx += vel_change_x;
        current_vy += vel_change_y;

        // Calculate displacement using current actual velocity
        float dx = current_vx * actual_dt;
        float dy = current_vy * actual_dt;

        if (odometer_history.size() > 300) {
            odometer_history.clear_last_n(30);
        }
        odometer_history.put(time_us(), Odometer{dx, dy, yaw_diff});
    } else {
        odometer_history.put(time_us(), Odometer{0.f, 0.f, yaw_diff});
    }
}

void OdometerProcessor::reset() {
    last_yaw = 0;
    last_yaw_init = false;
    pose = htwk::Position{0, 0, 0};
    pose_history.clear();

    // Reset current velocities
    current_vx = 0.0f;
    current_vy = 0.0f;

    // Reset time tracking
    first_call = true;
}

}  // namespace htwk