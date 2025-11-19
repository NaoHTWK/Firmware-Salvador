#include "head_control.h"

#include <loguru.hpp>

#include "gc_state.h"
#include "localization_utils.h"
#include "robot_time.h"
#include "soccerfield.h"
#include "stl_ext.h"

namespace htwk {

/**
 * @param w current position
 * @param t slowdown (higher value slows more)
 * @param gain maximum yaw-angle of head
 */
float HeadControl::smoothTriAng(float w, sta_settings s) {
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

float HeadControl::smoothTriAngStartTime(YawPitch head_pos, sta_settings sta_yaw,
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

YawPitch HeadControl::proceed(const MotionCommand& motion_command) {
#ifdef ROBOT_MODEL_T1
    float pitch_offset = 0;
    float cam_height = 1.05;
    float fps = 5000;
#else
    float pitch_offset = 14_deg;
    float cam_height = 0.9;
    float fps = 200;
#endif
    GCState gc_state = gc_state_sub.latest();
    bool may_move = true;
    if (gc_state.state == GameState::Initial || gc_state.state == GameState::Finished ||
        gc_state.state == GameState::Standby ||
        gc_state.my_team.players[gc_state.player_idx].is_penalized) {
        may_move = false;
    }
    if (!may_move)
        return YawPitch{0, 0};
    LocPosition loc_position = loc_position_sub.latest();
    HeadFocus cur_focus = motion_command.focus;
    int64_t time = time_us();
    if (cur_focus == HeadFocus::NOTHING)
        cur_focus = loc_position.quality > 0.7f ? HeadFocus::BALL : HeadFocus::LOC;
    time_step += (time - last_time) / 1'000'000.f;

#ifdef ROBOT_MODEL_K1
    if (std::shared_ptr<IMUJointState> imu_joint_state = imu_joint_states_sub.latest()) {
        head_pos = {imu_joint_state->serial[static_cast<size_t>(JointIndex::kHeadYaw)].q,
                    imu_joint_state->serial[static_cast<size_t>(JointIndex::kHeadPitch)].q};
    }
#elif ROBOT_MODEL_T1
// Booster doesn't set the head angles very well, so we just simulate it.
#else
    LOG_F(Error, "Unknown robot model in head control");
#endif

    if (cur_focus == HeadFocus::LOC) {
        sta_settings sta_yaw{2, 0.1f, 58_deg};
        if (cur_focus != last_focus)
            time_step = smoothTriAngStartTime(head_pos, sta_yaw);
        head_pos = {smoothTriAng(time_step, sta_yaw), 15_deg};
    } else if (cur_focus == HeadFocus::BALL || cur_focus == HeadFocus::BALL_GOALIE) {
        std::optional<TeamComData> striker = striker_sub.latest();
        std::lock_guard<std::mutex> lck(ball_detection_mtx);
        if (last_ball_percept > time - 2_s) {
            ball_found = true;
            // TODO: Rotate the last ball percept based on odometry since we last saw it.
            // float cur_ball_yaw = normalizeRotation(
            //         ball_pos.yaw -
            //         get<Position>(SensorData::instance().getOdometry(last_ball_percept,
            //         time)).a);
            float cur_ball_yaw = normalizeRotation(ball_pos.yaw);
            float target_yaw =
                    cur_ball_yaw;  // clamped_linear_interpolation(std::abs(cur_ball_yaw), 0.f,
                                   //                cur_ball_yaw, 10_deg, 20_deg);
            float target_pitch = ball_pos.pitch;  // clamped_linear_interpolation(48_deg -
                                                  // ball_pos.pitch, 48_deg,
                                                  //               ball_pos.pitch, 10_deg, 20_deg);
            head_pos.yaw = normalizeRotation(
                    head_pos.yaw + normalizeRotation(target_yaw - head_pos.yaw) * 40.f / fps);
            head_pos.pitch = clamp(head_pos.pitch + (target_pitch - head_pos.pitch) * 60.f / fps,
                                   10_deg, 48_deg);
        } else if ((cur_focus == HeadFocus::BALL_GOALIE || cur_focus == HeadFocus::BALL) &&
                   striker && striker->ball && striker->ball->ball_age_us < 1_s) {
            ball_found = true;
            point_2d rel_team_ball = LocalizationUtils::absToRel(
                    LocalizationUtils::relToAbs(striker->ball->pos_rel, striker->pos),
                    loc_position.position);
            // estimate yaw and pitch, this could be done with a relToCam if we have it.
            head_pos.yaw = normalizeRotation(
                    head_pos.yaw +
                    normalizeRotation(rel_team_ball.to_direction() - head_pos.yaw) * 40.f / fps);
            float target_pitch =
                    std::atan(cam_height / std::max(0.01f, rel_team_ball.magnitude())) -
                    pitch_offset;
            head_pos.pitch = clamp(head_pos.pitch + (target_pitch - head_pos.pitch) * 30.f / fps,
                                   10_deg, 48_deg);
        } else {
            sta_settings sta_yaw{3, 0.1f, 58_deg};
            if (cur_focus != last_focus || ball_found) {
                time_step = smoothTriAngStartTime(head_pos, sta_yaw);
                ball_found = false;
            }
            // TODO: use last seen to determine direction of this.
            // TODO: use pitch of striker ball
            head_pos.yaw = smoothTriAng(time_step, sta_yaw);
            if (striker && striker->ball) {
                point_2d rel_team_ball = LocalizationUtils::absToRel(
                        LocalizationUtils::relToAbs(striker->ball->pos_rel, striker->pos),
                        loc_position.position);
                float target_pitch =
                        std::atan(cam_height / std::max(0.01f, rel_team_ball.magnitude())) -
                        pitch_offset;
                head_pos.pitch =
                        clamp(head_pos.pitch + (target_pitch - head_pos.pitch) * 30.f / fps, 10_deg,
                              48_deg);
            } else {
                head_pos.pitch = 15_deg;
            }
        }
    } else if (cur_focus == HeadFocus::OBSTACLES) {
        sta_settings sta_yaw{2, 0.1f, 0.5f};
        if (cur_focus != last_focus)
            time_step = smoothTriAngStartTime(head_pos, sta_yaw);
        head_pos = {smoothTriAng(time_step, sta_yaw), 15_deg};
    } else if (cur_focus == HeadFocus::BALL_SEARCH_LEFT) {
        sta_settings sta_yaw{3, 0.1f, 30_deg};
        sta_settings sta_pitch{6, 0.1f, 19_deg};
        if (cur_focus != last_focus)
            time_step = smoothTriAngStartTime(YawPitch(head_pos.yaw - .4f, head_pos.pitch - 29_deg),
                                              sta_yaw, sta_pitch);
        head_pos = {.4f + smoothTriAng(time_step, sta_yaw),
                    smoothTriAng(time_step, sta_pitch) + 29_deg};
    } else if (cur_focus == HeadFocus::BALL_SEARCH_RIGHT) {
        sta_settings sta_yaw{-3, 0.1f, 30_deg};
        sta_settings sta_pitch{6, 0.1f, 19_deg};
        if (cur_focus != last_focus)
            time_step = smoothTriAngStartTime(YawPitch(head_pos.yaw + .4f, head_pos.pitch - 29_deg),
                                              sta_yaw, sta_pitch);
        head_pos = {-.4f + smoothTriAng(time_step, sta_yaw),
                    smoothTriAng(time_step, sta_pitch) + 29_deg};
    } else {
        head_pos = {0, 0};
    }
    last_focus = cur_focus;
    last_time = time;
    return head_pos;
}

}  // namespace htwk
