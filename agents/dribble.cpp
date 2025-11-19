#include "dribble.h"

#include <cstdlib>
#include <optional>

#include "localization_utils.h"
#include "logging.h"
#include "moveballgoalorder.h"
#include "moveballorder.h"
#include "near_obstacle_tracker_result.h"
#include "point_2d.h"
#include "point_3d.h"
#include "position.h"
#include "soccerfield.h"
#include "stl_ext.h"
#include "unordered_map"

static point_2d dribblingDirection(const point_2d& pos) {
    static float h_length = SoccerField::fieldLength() / 2.f;
    static float h_width = SoccerField::fieldWidth() / 2.f;
    static float q_length = SoccerField::fieldLength() / 4.f;
    static float q_width = SoccerField::fieldWidth() / 4.f;
    static float h_width_goal_posts = SoccerField::goalPostDistance() / 2.f;

    static std::unordered_map<point_2d /*field_coord*/, point_2d /*weighted_dir*/> vertices{
            {{-h_length, -h_width}, {2.f, 2.f}},   {{-h_length, -q_width}, {2.f, -2.f}},
            {{-h_length, 0.f}, {2.f, 0.f}},        {{-h_length, q_width}, {2.f, 2.f}},
            {{-h_length, h_width}, {2.f, -2.f}},

            {{0.f, -h_width}, {1.f, 0.5f}},        {{0.f, 0.f}, {1.f, 0.f}},
            {{0.f, h_width}, {1.f, -0.5f}},

            {{q_length, -h_width}, {1.f, 2.f}},    {{q_length, 0.f}, {5.f, 0.f}},
            {{q_length, h_width}, {1.f, -2.f}},

            {{h_length, -h_width}, {-1.5f, 3.5f}}, {{h_length, -q_width}, {-1.5f, 3.5f}},
            {{h_length, -1.633f}, {-0.5f, 2.f}},   {{h_length, -h_width_goal_posts}, {1.5f, 0.f}},
            {{h_length, 0.f}, {4.f, 0.f}},         {{h_length, h_width_goal_posts}, {1.5f, 0.f}},
            {{h_length, 1.633f}, {-0.5f, -2.f}},   {{h_length, q_width}, {-1.5f, -3.5f}},
            {{h_length, h_width}, {-1.5f, -3.5f}}};

    point_2d sum(0, 0);
    for (const auto& [v_pos, dir] : vertices) {
        sum += dir / (.1f + (pos - v_pos).norm_sqr());
    }
    return sum.normalized();
}

point_2d DribbleAgent::getSpreadTargetVector(point_2d target, float maxAngle) {
    if (abs(target.to_direction()) < maxAngle)
        return {1, 0};
    else if (target.to_direction() > 0)
        return target.rotated(-maxAngle);
    else
        return target.rotated(maxAngle);
}

float DribbleAgent::getSpreadFactor(point_2d robot_pos, float weight) {
    float fadingDistanceX = 3;
    float fadingDistanceY = 3;
    float goal_fading_distance = 1.3;  // updated from 0.6
    float borderX = 4.0;
    float borderY = 2.5;
    float xFactor = limit01(1.f - (robot_pos.x - borderX) / fadingDistanceX);
    float yFactor = limit01(1.f - (abs(robot_pos.y) - borderY) / fadingDistanceY);
    static float h_length = SoccerField::length() / 2.f;
    static float h_width = SoccerField::width() / 2.f;
    float goal_center_diff = limit01(
            1 - std::max(fabsf(robot_pos.x - h_length), fabsf(robot_pos.y)) / goal_fading_distance);
    return std::max(xFactor * yFactor, goal_center_diff) * weight;
}

MotionCommand DribbleAgent::proceed(std::shared_ptr<Order> order) {
    if (!isOrder<MoveBallGoalOrder, MoveBallOrder>(order))
        return MotionCommand::Nothing;

    std::shared_ptr<htwk::NearObstacleTrackerResult> near_obstacle_tracker_result =
            near_obstacle_tracker_result_sub.latest();
    if (!near_obstacle_tracker_result) {
        LOG_F(ERROR, "No near obstacle tracker result available.");
        exit(43);
        return MotionCommand::Nothing;
    }

    std::optional<RelBall> rel_ball = rel_ball_sub.latest();
    std::optional<LocPosition> loc_position = loc_position_sub.latestIfExists();

    if (!loc_position || loc_position->quality == 0) {
        return MotionCommand::Nothing;
    }

    if (!rel_ball) {
        return MotionCommand::Nothing;
    }
    if (rel_ball->pos_rel.norm() > 3.0f)  // change also in walkToPos!
        return MotionCommand::Nothing;

    htwk::Position robot_pos = loc_position->position;

    point_2d dribble_dir_rel;
    if (isOrder<MoveBallGoalOrder>(order)) {
        point_2d dribbling_dir = dribblingDirection(
                LocalizationUtils::relToAbs(rel_ball->pos_rel, loc_position->position));
        dribble_dir_rel = dribbling_dir.rotated(-loc_position->position.a);
    } else {
        point_2d ball_dest_abs = dynamic_cast<MoveBallOrder*>(order.get())->pos;
        point_2d ball_dest_rel = LocalizationUtils::absToRel(ball_dest_abs, loc_position->position);
        dribble_dir_rel = (ball_dest_rel - rel_ball->pos_rel).normalized();
    }

    float spreadFactor = getSpreadFactor(robot_pos.point(), 1.0f);

    bool limit_speed = near_obstacle_tracker_result->corridorBlocked(0.5f, 0.6f);

    if (false) {
        return MotionCommand::Walk({.dx = 0.f, .dy = 0.0f, .da = 0.0f}, HeadFocus::BALL);
    }
    float dribble_angle = dribble_dir_rel.to_direction();
    if (dribble_angle > 0.f) {
        dribble_angle -= spreadFactor * 20_deg;
        if (-dribble_angle > 0.f) {
            dribble_angle = 0.f;
        }
    }
    if (-dribble_angle > 0.f) {
        dribble_angle += spreadFactor * 20_deg;
        if (dribble_angle > 0.f) {
            dribble_angle = 0.f;
        }
    }
    dribble_dir_rel = point_2d(1, 0).rotated(dribble_angle);
    point_2d ball_pos_rel = rel_ball->pos_rel;
    float ball_feet_offset = 0.f;
    ball_pos_rel.y += ball_feet_offset;
    float ball_angle = ball_pos_rel.to_direction();
    float ball_distance = ball_pos_rel.norm();

    float distance_p = 1.0f;
    float min_angle_correction_distance = 0.3;
    float ball_far =
            clamp((ball_distance - min_angle_correction_distance) / min_angle_correction_distance,
                  0.5f, 1.0f);
    float angle_correction_p = 2.f * ball_far;
    float bturn_p = 0.60f;
    float dribble_forward_p = 1.f;
    float bturn_distance = 0.25f;
    float fast_dribble = spreadFactor;
    if (limit_speed) {
        fast_dribble = 1.0f;
    }
    fast_dribble = 0.f;
    float dribble_bturn_damp = 5.f * (1.f - fast_dribble) + fast_dribble * 2.7f;    // 2.2
    float dribble_ball_y_damp = 3.0f * (1.f - fast_dribble) + fast_dribble * 2.0f;  // 1.5
    float max_bturn_side_speed = -0.8f;
    float max_bturn_angle_speed = 0.9f;
    float side_y_correction_p = 1.2f;
    float side_correction_p = 1.f;
    float near_to_ball = clamp(1.5f - ball_distance, 0.f, 1.f);
    float somewhat_near_to_ball = clamp(2.5f - ball_distance, 0.f, 1.f);
    float dribble_angle_forward = std::clamp((90_deg - std::abs(dribble_angle)) / 15_deg, 0.f, 1.f);

    point_2d target_pos_rel = ball_pos_rel - dribble_dir_rel * bturn_distance * 2.0f;
    float target_angle = target_pos_rel.to_direction();
    ball_angle = target_angle * (1.f - near_to_ball) + ball_angle * near_to_ball;
    float bturn_factor = clamp(dribble_angle * bturn_p, -1.f, 1.f) * near_to_ball;
    float dribble_forward_factor = clamp(1.f - fabsf(bturn_factor) * dribble_bturn_damp -
                                                 fabsf(ball_pos_rel.y) * dribble_ball_y_damp,
                                         0.f, 1.f) *
                                   dribble_forward_p;
    bturn_factor *= clamp(1.f - dribble_forward_factor * 2.f, 0.f, 1.f);
    if (limit_speed) {  //
        dribble_forward_factor *= 0.7f;
    } else {
        float goal_fading_distance = 5.0;
        static float h_length = SoccerField::length() / 2.f;
        float goal_center_diff =
                limit01(1.f - std::max(fabsf(robot_pos.x - h_length), fabsf(robot_pos.y)) /
                                      goal_fading_distance);
        float goal_sideplay_damping = fabsf(sinf(loc_position->position.a) * goal_center_diff);
        float goal_sideplay_p = 0.9f;
        dribble_forward_factor *= 1.f - goal_sideplay_damping * goal_sideplay_p;
    }
    float forward_damping_factor = 1.f - clamp(fabsf(ball_angle / (M_PI / 3.f)), 0.f, 1.f);
    float bturn_distance_correction =
            clamp((ball_distance - bturn_distance) * distance_p, -1.0f, 1.0f) *
            forward_damping_factor;
    float wrong_target_angle = clamp(fabsf(dribble_angle * 0.7f), 0.6f, 1.f);
    float angle_correction =
            ball_angle * angle_correction_p * wrong_target_angle * somewhat_near_to_ball;
    float side_correction =
            target_pos_rel.y * side_correction_p * somewhat_near_to_ball * dribble_angle_forward +
            ball_pos_rel.y * side_y_correction_p * (1.f - wrong_target_angle);
    float non_linear_rotation_compensation = 0.3f;
    point_3d bturn_vec = {
            clamp(bturn_distance_correction + dribble_forward_factor, -1.f, 1.f),
            clamp(max_bturn_side_speed * bturn_factor + side_correction, -1.f, 1.f),
            clamp(max_bturn_angle_speed * bturn_factor + angle_correction, -1.f, 1.f)};

    return MotionCommand::Walk(
            {.dx = bturn_vec.x,
             .dy = bturn_vec.y,
             .da = powf(fabsf(bturn_vec.z), non_linear_rotation_compensation) * sgn(bturn_vec.z)},
            HeadFocus::BALL);
}
