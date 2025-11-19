#include "head_control.h"

#include <async.h>
#include <logging.h>

#include "cam_pose.h"
#include "camera/pub_sub/camera_pub_sub.h"
#include "motion_connector/pub_sub/motion_pub_sub.h"
#include "multi_target_tracker/pub_sub/multi_target_tracker_pub_sub.h"

#define sign(x) (x < 0 ? -1 : (x > 0 ? 1 : 0))

htwk::HeadFocusInternal htwk::HeadFocusInternal::nothing() {
    return {.type = NOTHING};
}

htwk::HeadFocusInternal htwk::HeadFocusInternal::ball() {
    return {.type = BALL};
}

htwk::HeadFocusInternal htwk::HeadFocusInternal::ball_search_left() {
    return {.type = BALL_SEARCH_LEFT};
}

htwk::HeadFocusInternal htwk::HeadFocusInternal::ball_search_right() {
    return {.type = BALL_SEARCH_RIGHT};
}

htwk::HeadFocusInternal htwk::HeadFocusInternal::localize() {
    return {.type = LOCALIZE};
}

htwk::HeadFocusInternal htwk::HeadFocusInternal::fixed_relative(const point_2d &relative_postion) {
    return {.type = FIXED_REL, .argument = relative_postion};
}

htwk::HeadControl::HeadControl()
    : ball_subscriber(rel_ball_channel.create_subscriber()),
      image_subscriber(images.create_subscriber()),
      may_move(true),
      focus({HeadFocusInternal::NOTHING}) {
    current = {0, 0};
    target = {0, 0};
    previous = {0, 0};
    current_velocity = {0, 0};
    previous_velocity = {0, 0};
}

htwk::HeadControl::YawPitch htwk::HeadControl::look_at_ball() {

    using namespace std::chrono_literals;

    point_2d relative_ball_position;
    bool striker_ball = false;

    auto ball_detection = ball_subscriber.latest();
    if (ball_detection.has_value()) {
        // Fetch information about ball from vision
        const auto ball_info = ball_detection;
        relative_ball_position = ball_info->pos_rel;
        last_ball_percept = std::chrono::high_resolution_clock::now();
    }

    auto now = std::chrono::high_resolution_clock::now();

    // A ball was seen in the last two seconds
    if (std::chrono::duration<float>(now - last_ball_percept).count() < 2.0f) {
        // We look at the ball we have just seen
        return look_at_ball_relative(relative_ball_position);
    } else if (focus.type == HeadFocusInternal::BALL_GOALIE && striker_ball) {

        // TODO: fetch possible ball location from other players

    } else {
        sta_settings sta_yaw{1, 0.1f, YAW_MAX};
        float t = std::chrono::duration<float>(now - pattern_start_time).count();

        return {.yaw = smoothTriAng(t + pattern_time_offset, sta_yaw), .pitch = current.pitch};
    }

    // Look in the same direction as before
    return current;
}

htwk::HeadControl::YawPitch htwk::HeadControl::look_at_position_relative(
        const point_2d &relative_position) {

    auto image = image_subscriber.latestIfExists();
#if defined(ROBOT_MODEL_T1)
    float height_above_ground = 1.13f;
#else
    float height_above_ground = 0.89f;
#endif
    if (image.has_value()) {
        height_above_ground = (*image)->cam_pose_ptr->get_translation().z;
    }

    float yaw = atan2f(relative_position.y, relative_position.x);
    float pitch = atan2f(height_above_ground, relative_position.norm());

    return {yaw, pitch};
}

htwk::HeadControl::YawPitch htwk::HeadControl::look_at_ball_relative(
        const point_2d &relative_position) {

    auto image = image_subscriber.latestIfExists();
#if defined(ROBOT_MODEL_T1)
    float height_above_ground = 1.13f;
#else
    float height_above_ground = 0.89f;
#endif
    if (image.has_value()) {
        height_above_ground = (*image)->cam_pose_ptr->get_translation().z;
    }

    float head_stabilization_factor = std::min(1.f, relative_position.norm());
    float smoothing_factor=0.5f;
    
    float yaw = atan2f(relative_position.y, relative_position.x) * head_stabilization_factor*smoothing_factor;
    float pitch = atan2f(height_above_ground, relative_position.norm())*smoothing_factor;

    return {yaw, pitch};
}

htwk::HeadControl::YawPitch htwk::HeadControl::look_search_pattern() {

    const float yaw_factor = 10.0 / 7;
    const float pitch_factor = 1.5;

    const float min_pitch = PITCH_MIN / 2;
    const float max_pitch = PITCH_MIN;
    const float mid_pitch = (min_pitch + max_pitch) / 2;

    auto now = std::chrono::high_resolution_clock::now();

    float t = std::chrono::duration<float>(now - pattern_start_time).count();

    // TODO: Change this to a triang function
    // float yaw = YAW_MAX * sinf(yaw_factor * t);
    float yaw = smoothTriAng(yaw_factor * t + M_PIf / 2, sta_settings(1.0, 0.1, YAW_MAX));
    float pitch = ((max_pitch - min_pitch) * sinf(pitch_factor * t) - mid_pitch) / 2;

    return {yaw, pitch};
}

/*
htwk::HeadControl::YawPitch htwk::HeadControl::look_search_for_ball_left() {

    static int pattern_count = 0;
    const float yaw_factor = 5.0;

    auto now = std::chrono::high_resolution_clock::now();

    float t = std::chrono::duration<float>(now - pattern_start_time).count();

    float yaw =
            smoothTriAng(2 * M_PI / yaw_factor * t + M_PIf / 2, sta_settings(1.0, 0.1, YAW_MAX));
    // float yaw = YAW_MAX * sinf(2 * M_PI / yaw_factor * t);

    if (t > yaw_factor) {
        pattern_count++;
        pattern_count %= 3;
        pattern_start_time = std::chrono::high_resolution_clock::now();
    }

    const std::array<float, 3> pitches = {PITCH_MAX, 2 * PITCH_MAX / 3, PITCH_MAX / 3};

    return {yaw, pitches[pattern_count]};
}
*/

htwk::HeadControl::YawPitch htwk::HeadControl::look_search_for_ball_left() {
    auto now = std::chrono::high_resolution_clock::now();
    float t = std::chrono::duration<float>(now - pattern_start_time).count();
    return {.yaw = 0.4f + smoothTriAng(t + pattern_time_offset, BALL_SEARCH_LEFT_YAW_STA),
            .pitch = 0.25f + smoothTriAng(t + pattern_time_offset, BALL_SEARCH_LEFT_PITCH_STA)};
}

htwk::HeadControl::YawPitch htwk::HeadControl::look_search_for_ball_right() {
    auto now = std::chrono::high_resolution_clock::now();
    float t = std::chrono::duration<float>(now - pattern_start_time).count();
    return {.yaw = -0.4f + smoothTriAng(t + pattern_time_offset, BALL_SEARCH_RIGHT_YAW_STA),
            .pitch = 0.25f + smoothTriAng(t + pattern_time_offset, BALL_SEARCH_RIGHT_PITCH_STA)};
}

htwk::HeadControl::YawPitch htwk::HeadControl::look_obstacles() {

    auto now = std::chrono::high_resolution_clock::now();
    float t = std::chrono::duration<float>(now - pattern_start_time).count();

    return {.yaw = 0.4f + smoothTriAng(t + pattern_time_offset, OBSTACLES_YAW_STA), .pitch = 0.2f};
}

htwk::HeadControl::YawPitch htwk::HeadControl::look_search_localize() {
    auto now = std::chrono::high_resolution_clock::now();
    float t = std::chrono::duration<float>(now - pattern_start_time).count();

    return {.yaw = smoothTriAng(t + pattern_time_offset, LOCALIZE_YAW_STA), .pitch = 0.2f};
}

htwk::HeadControl::YawPitch htwk::HeadControl::proceed(const MotionCommand &motion_command) {

    set_focus(motion_command.focus);

    if (!may_move) {
        // TODO make head stationary. Do this by getting the current angles and using them as
        // targets.
        return current;
    }

    switch (focus.type) {

        case HeadFocusInternal::NOTHING:
            break;

        case HeadFocusInternal::BALL_GOALIE:
            [[fallthrough]];
        case HeadFocusInternal::BALL:
            target = look_at_ball();
            break;

        case HeadFocusInternal::BALL_SEARCH_LEFT:
            target = look_search_for_ball_left();
            break;

        case HeadFocusInternal::BALL_SEARCH_RIGHT:
            target = look_search_for_ball_right();
            break;

        case HeadFocusInternal::FIXED_REL:
            target = look_at_position_relative(std::get<point_2d>(*(focus.argument)));
            break;

        case HeadFocusInternal::LOCALIZE:
            target = look_search_localize();
            break;

        case HeadFocusInternal::OBSTACLES:
            target = look_obstacles();
            break;
    }

    target.yaw = std::clamp(target.yaw, YAW_MIN, YAW_MAX);
    target.pitch = std::clamp(target.pitch, PITCH_MIN, PITCH_MAX);

    // Store previous values for velocity and acceleration calculation
    previous = current;
    previous_velocity = current_velocity;
    
    // Apply acceleration limits to the target movement
    current = apply_acceleration_limits(target);
    
    current.yaw = std::clamp(current.yaw, YAW_MIN, YAW_MAX);
    current.pitch = std::clamp(current.pitch, PITCH_MIN, PITCH_MAX);

    return current;
}

void htwk::HeadControl::set_focus(const HeadFocusInternal &f) {

    // If we do not change focus type, we do nothing.
    if (f.type == focus.type) {
        return;
    }

    focus = f;
    LOG_F(INFO, "HeadFocus has type %d", (int)focus.type);

    switch (f.type) {

        case HeadFocusInternal::BALL:
            [[fallthrough]];
        case HeadFocusInternal::BALL_GOALIE:
            pattern_start_time = std::chrono::high_resolution_clock::now();
            pattern_time_offset = smoothTriAngStartTime(current, sta_settings(1, 0.1f, 1.68f));
            break;

        case HeadFocusInternal::BALL_SEARCH_LEFT:
            pattern_start_time = std::chrono::high_resolution_clock::now();
            pattern_time_offset = smoothTriAngStartTime(
                    {.yaw = current.yaw - 0.4f, .pitch = current.pitch - 0.25f},
                    BALL_SEARCH_LEFT_YAW_STA, BALL_SEARCH_LEFT_PITCH_STA);
            break;

        case HeadFocusInternal::BALL_SEARCH_RIGHT:
            pattern_start_time = std::chrono::high_resolution_clock::now();
            pattern_time_offset = smoothTriAngStartTime(
                    {.yaw = current.yaw - 0.4f, .pitch = current.pitch - 0.25f},
                    BALL_SEARCH_RIGHT_YAW_STA, BALL_SEARCH_RIGHT_PITCH_STA);
            break;

        case HeadFocusInternal::LOCALIZE:
            pattern_start_time = std::chrono::high_resolution_clock::now();
            pattern_time_offset = smoothTriAngStartTime(current, LOCALIZE_YAW_STA);
            break;

        case HeadFocusInternal::OBSTACLES:
            pattern_start_time = std::chrono::high_resolution_clock::now();
            pattern_time_offset = smoothTriAngStartTime(current, OBSTACLES_YAW_STA);
            break;

        default:
            break;
    }
}

/**
 * @param w current position
 * @param t slowdown (higher value slows more)
 * @param gain maximum yaw-angle of head
 */
float htwk::HeadControl::smoothTriAng(float w, sta_settings s) {
    // TODO: Replace this monster with something understandable. Do we even want to break? Why not
    // linear acceleration/deceleration?
    w *= s.w_fac;
    while (w < 0)
        w += M_PIf * 2.f;
    while (w > M_PIf * 2.f)
        w -= M_PIf * 2.f;
    if ((w > s.t && w < M_PIf - s.t) || (w > M_PIf + s.t && w < M_PIf * 2.f - s.t)) {
        if (w < M_PIf) {
            return s.gain * (w / M_PIf * 2.f - 1);
        } else {
            return s.gain * ((M_PIf * 2.f - w) / M_PIf * 2.f - 1.f);
        }
    } else {
        float r = s.t;
        float a = 1.f / 2.f / r;
        float b = -r / 2.f + s.t;
        if (w <= s.t) {
            float x = w;
            return s.gain * ((a * x * x + b) / M_PIf * 2.f - 1.f);
        } else if (w <= M_PIf) {
            float x = M_PIf - w;
            return s.gain * ((1.f - (a * x * x + b) / M_PIf) * 2.f - 1.f);
        } else if (w >= M_PIf * 2.f - s.t) {
            float x = w - M_PIf * 2.f;
            return s.gain * ((a * x * x + b) / M_PIf * 2.f - 1.f);
        } else if (w > M_PIf) {
            float x = w - M_PIf;
            return s.gain * ((1.f - (a * x * x + b) / M_PIf) * 2.f - 1.f);
        }
        return 0;
    }
}

float htwk::HeadControl::smoothTriAngStartTime(YawPitch head_pos, sta_settings sta_yaw,
                                               sta_settings sta_pitch) {
    float min_w = 0;
    float min_dist = std::numeric_limits<float>::infinity();
    for (float w = 0; w < M_PIf * 2.f / std::abs(sta_yaw.w_fac);
         w += 0.1f / std::abs(sta_yaw.w_fac)) {
        float dist = 0;
        if (sta_pitch.w_fac == 0) {
            dist = std::abs(smoothTriAng(w, sta_yaw) - head_pos.yaw);
        } else {
            // TODO: This doesn't work if abs(w_fac) is different in sta_yaw vs. sta_pitch.
            dist = (point_2d(smoothTriAng(w, sta_yaw), smoothTriAng(w, sta_pitch)) -
                    point_2d(head_pos.yaw, head_pos.pitch))
                           .magnitude();
        }
        if (dist < min_dist) {
            min_dist = dist;
            min_w = w;
        }
    }
    return min_w;
}

htwk::HeadControl::YawPitch htwk::HeadControl::apply_acceleration_limits(const YawPitch& target_angles) {
    // Calculate current velocity from position change
    current_velocity.yaw = (current.yaw - previous.yaw) / UPDATE_INTERVAL;
    current_velocity.pitch = (current.pitch - previous.pitch) / UPDATE_INTERVAL;
    
    // Calculate position error
    float yaw_error = target_angles.yaw - current.yaw;
    float pitch_error = target_angles.pitch - current.pitch;
    
    // Use a very aggressive proportional control for fast response
    float proportional_gain = 40.0f; // Doubled for twice as fast movement
    float desired_velocity_yaw = yaw_error * proportional_gain;
    float desired_velocity_pitch = pitch_error * proportional_gain;
    
    // Limit velocity to maximum allowed
    desired_velocity_yaw = std::clamp(desired_velocity_yaw, -max_ang_vel, max_ang_vel);
    desired_velocity_pitch = std::clamp(desired_velocity_pitch, -max_ang_vel, max_ang_vel);
    
    // Much more aggressive velocity change limiting
    float max_velocity_change = max_ang_accel * UPDATE_INTERVAL;  // = 500 * 0.002 = 1.0 rad/s
    float velocity_change_yaw = desired_velocity_yaw - current_velocity.yaw;
    float velocity_change_pitch = desired_velocity_pitch - current_velocity.pitch;
    
    velocity_change_yaw = std::clamp(velocity_change_yaw, -max_velocity_change, max_velocity_change);
    velocity_change_pitch = std::clamp(velocity_change_pitch, -max_velocity_change, max_velocity_change);
    
    float new_velocity_yaw = current_velocity.yaw + velocity_change_yaw;
    float new_velocity_pitch = current_velocity.pitch + velocity_change_pitch;
    
    // Calculate new position
    YawPitch new_position;
    new_position.yaw = current.yaw + new_velocity_yaw * UPDATE_INTERVAL;
    new_position.pitch = current.pitch + new_velocity_pitch * UPDATE_INTERVAL;
    
    return new_position;
}
