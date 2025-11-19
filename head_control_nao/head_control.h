#pragma once

#include <atomic>
#include <mutex>

#include "cam_constants.h"
#include "gc_pub_sub.h"
#include "head_focus.h"
#include "imu.h"
#include "joints.h"
#include "localization_pub_sub.h"
#include "motion_command.h"
#include "sensor_pub_sub.h"
#include "team_strategy_pub_sub.h"
#include "vision_pub_sub.h"

namespace htwk {

class HeadControl {
public:
    struct sta_settings {
        float w_fac = 0;
        float t = 0;
        float gain = 0;
        sta_settings() {}
        sta_settings(float w_fac, float t, float gain) : w_fac(w_fac), t(t), gain(gain) {}
    };

    HeadControl() {
        cam_ball_callback.registerCallback(std::bind_front(&HeadControl::updateDetectedBall, this));
    }

    YawPitch proceed(const MotionCommand& motion_command);

    void updateDetectedBall(const point_2d& ball, YawPitch head_angles, int64_t timestamp_us,
                            int cx, int cy, int fx, int fy) {
        std::lock_guard<std::mutex> lck(ball_detection_mtx);
        ball_pos = YawPitch{head_angles.yaw - std::atan((ball.x - cx) / fx),
                            head_angles.pitch + std::atan((ball.y - cy) / fy)};
        last_ball_percept = timestamp_us;
    }
    static float smoothTriAng(float w, sta_settings s);
    static float smoothTriAngStartTime(YawPitch head_pos, sta_settings sta_yaw,
                                       sta_settings sta_pitch = sta_settings());

private:
    htwk::ChannelSubscriber<GCState> gc_state_sub = gc_state.create_subscriber();
    htwk::ChannelSubscriber<LocPosition> loc_position_sub =
            loc_position_channel.create_subscriber();
    htwk::ChannelSubscriber<std::shared_ptr<IMUJointState>> imu_joint_states_sub =
            imu_joint_states_channel.create_subscriber();
    htwk::ChannelSubscriber<std::optional<TeamComData>> striker_sub =
            striker_channel.create_subscriber();
    std::mutex ball_detection_mtx;
    int64_t last_ball_percept = 0;
    YawPitch ball_pos;
    HeadFocus last_focus = HeadFocus::NOTHING;
    int64_t last_time = 0;
    float time_step = 0;
    // Initialized with true to force time_step calculation in Focus::BALL.
    bool ball_found = true;
    float pitch_base_value = 48_deg;
    YawPitch head_pos{0, 0};
};

}  // namespace htwk