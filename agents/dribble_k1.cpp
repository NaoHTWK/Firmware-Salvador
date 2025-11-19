#include <cstdlib>
#include <optional>

#include "dribble.h"
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
    static float h_length = SoccerField::length() / 2.f;
    static float h_width = SoccerField::width() / 2.f;
    static float q_length = SoccerField::length() / 4.f;
    static float q_width = SoccerField::width() / 4.f;
    static std::unordered_map<point_2d /*field_coord*/, point_2d /*weighted_dir*/> vertices{
            {{-h_length, -h_width}, {2.f, 2.f}},   {{-h_length, -q_width}, {2.f, -2.f}},
            {{-h_length, 0.f}, {2.f, 0.f}},        {{-h_length, q_width}, {2.f, 2.f}},
            {{-h_length, h_width}, {2.f, -2.f}},

            {{0.f, -h_width}, {1.f, 0.5f}},        {{0.f, 0.f}, {1.f, 0.f}},
            {{0, h_width}, {1.f, -0.5f}},

            {{q_length, -h_width}, {1.f, 2.f}},    {{q_length, 0.f}, {5.f, 0.f}},
            {{q_length, h_width}, {1.f, -2.f}},

            {{h_length, -h_width}, {-1.5f, 3.5f}}, {{h_length, -q_width}, {-0.7f, 1.0f}},
            {{h_length, 0.f}, {5.f, 0.f}},         {{h_length, q_width}, {-0.7f, -1.0f}},
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
    float fadingDistanceX = 1;
    float fadingDistanceY = 1;
    float goal_fading_distance = 1.3;  // updated from 0.6
    float borderX = 2.0;
    float borderY = 1.5;
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

    // float off_dx = param_tuner.getParameter("dx", 0.7f);
    // float off_dy = param_tuner.getParameter("dy", 0.6f);
    // float off_da = param_tuner.getParameter("da", 1.0f);

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

    // Apply spread filtering similar to firmware 5.0
    float spreadFactor = getSpreadFactor(robot_pos.point(), 1.0f);

    // TODO: Walk around obstacles.
    // TODO: Implement pew shot.

    bool limit_speed = near_obstacle_tracker_result->corridorBlocked(0.5f, 0.6f);
    /*if(limit_speed){
        LOG_F(INFO, "############### OBSTACLE ###############");
    }else{
        LOG_F(INFO, "---");
    }*/

    float dribble_angle = dribble_dir_rel.to_direction();
    point_2d ball_pos_rel = rel_ball->pos_rel;
    float ball_feet_offset = -0.05f;
    /*printf("%02f,\t%02f\n",ball_pos_rel.x,ball_pos_rel.y);
    return MotionCommand::Walk(
            {.dx = 0, .dy = 0, .da = 0},
            HeadFocus::BALL);*/
    ball_pos_rel.y += ball_feet_offset;
    float ball_angle = ball_pos_rel.to_direction();
    float ball_distance = ball_pos_rel.norm();

    float distance_p = 1.f;
    float min_angle_correction_distance = 0.3;
    float ball_far =
            clamp((ball_distance - min_angle_correction_distance) / min_angle_correction_distance,
                  0.5f, 1.0f);
    float angle_correction_p = 2.f * ball_far;
    float bturn_p = 0.35f;
    float dribble_forward_p = 1.f;
    float bturn_distance = 0.25f;
    float target_distance = 0.4f;
    float dribble_bturn_damp = 2.8f - spreadFactor;
    float dribble_ball_y_damp = 2.5f - spreadFactor;
    float max_bturn_side_speed = -0.7f;
    float max_bturn_angle_speed = 0.7f;
    float max_bturn_distance = 1.0;
    float side_correction_p = 0.6f;
    float side_y_correction_p = 1.0f;
    // angle_correction_p*=1.f-ball_near;
    // side_correction_p*=ball_near;

    point_2d target_pos_rel = ball_pos_rel + dribble_dir_rel * bturn_distance;
    float bturn_factor = clamp(dribble_angle * bturn_p, -1.f, 1.f);
    float dribble_forward_factor = clamp(1.f - fabsf(bturn_factor) * dribble_bturn_damp -
                                                 fabsf(ball_pos_rel.y) * dribble_ball_y_damp,
                                         0.f, 1.f) *
                                   dribble_forward_p;

    if (limit_speed) {
        dribble_forward_factor *= 0.3f;
    }
    float forward_damping_factor = 1.f - clamp(fabsf(ball_angle / (3.14f / 2.f)), 0.f, 1.f);
    float bturn_distance_correction =
            clamp((ball_distance - bturn_distance) * distance_p, -1.0f, 1.0f) *
            forward_damping_factor;
    float wrong_target_angle = clamp(fabsf(dribble_angle * 0.7f), 0.6f, 1.f);
    float angle_correction = ball_angle * angle_correction_p * wrong_target_angle;
    float side_correction = (ball_angle - clamp(ball_angle, -0.3f, 0.3f)) * side_correction_p +
                            ball_pos_rel.y * side_y_correction_p * (1.f - wrong_target_angle);
    point_3d bturn_vec = {bturn_distance_correction + dribble_forward_factor,
                          max_bturn_side_speed * bturn_factor + side_correction,
                          max_bturn_angle_speed * bturn_factor + angle_correction};
    /*point_3d bturn_vec = {bturn_distance_correction,
                        side_correction,
                        angle_correction};*/
    return MotionCommand::Walk({.dx = bturn_vec.x, .dy = bturn_vec.y, .da = bturn_vec.z},
                               HeadFocus::BALL);
}