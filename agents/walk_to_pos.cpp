#include "walk_to_pos.h"

#include <localization_utils.h>

#include "keepgoalorder.h"
#include "moveballgoalorder.h"
#include "walktopositionorder.h"

using namespace htwk;
using namespace std;

htwk::Position WalkToPositionAgent::handelHeadAngle(htwk::Position order_pos,
                                                    WalkToPositionOrder::Mode order_mode,
                                                    htwk::Position current_pos) {
    htwk::Position dest = order_pos;
    if (order_mode != WalkToPositionOrder::Mode::USE_A &&
        order_mode != WalkToPositionOrder::Mode::FOCUS_DIRECTION) {
        if (gc_state_sub.latest().state == GameState::Ready) {
            focus = HeadFocus::LOC;
            dest.a = current_pos.a;
        } else {
            focus = HeadFocus::BALL;
            std::optional<RelBall> rel_ball = rel_ball_sub.latest();
            std::optional<TeamComData> striker = striker_sub.latest();
            if (rel_ball) {
                dest.a = normalizeRotation(rel_ball->pos_rel.to_direction() + current_pos.a);
            } else if (striker && striker->ball) {
                dest.a = normalizeRotation(
                        LocalizationUtils::absToRel(
                                LocalizationUtils::relToAbs(striker->ball->pos_rel, striker->pos),
                                current_pos)
                                .to_direction() +
                        current_pos.a);
            } else {
                focus = HeadFocus::LOC;
                dest.a = current_pos.a;
            }
        }
    }
    return dest;
}

MotionCommand WalkToPositionAgent::proceed(std::shared_ptr<Order> order) {
    // TODO: Avoid obstacles from order
    if (!isOrder<WalkToPositionOrder, KeepGoalOrder, MoveBallGoalOrder>(order))
        return MotionCommand::Nothing;

    std::shared_ptr<htwk::NearObstacleTrackerResult> near_obstacle_tracker_result =
            near_obstacle_tracker_result_sub.latest();
    if (!near_obstacle_tracker_result) {
        LOG_F(ERROR, "No near obstacle tracker result available.");
        exit(43);
        return MotionCommand::Nothing;
    }

    htwk::Position dest;
    std::vector<vectorfield::Influencer> obstacles;
    std::vector<RobotDetection> rel_robots = rel_robots_sub.latest();
    std::optional<LocPosition> current_pos = pos_sub.latest();
    float omni_dist = 3.75f;
    if (!current_pos) {
        return MotionCommand::Nothing;
    }
    for (const auto& robot : rel_robots) {
        obstacles.push_back(vectorfield::Influencer{
                .position = LocalizationUtils::relToAbs(robot.pos_rel, current_pos->position),
                .radius = 1.5f,
                .deflection = 1.5f,
                .flee = false});
    }

    focus = HeadFocus::LOC;
    bool focus_a = false;

    if (isOrder<WalkToPositionOrder>(order)) {
        auto* wtp_order = dynamic_cast<WalkToPositionOrder*>(order.get());

        dest = handelHeadAngle(wtp_order->pos, wtp_order->mode, current_pos->position);
        obstacles.insert(obstacles.end(), wtp_order->obstacle_influencers.begin(),
                         wtp_order->obstacle_influencers.end());

        focus_a = wtp_order->mode == WalkToPositionOrder::Mode::FOCUS_DIRECTION;
        if (wtp_order->head_focus)
            focus = *wtp_order->head_focus;
    } else if (isOrder<MoveBallGoalOrder>(order)) {
        auto* mbg_order = dynamic_cast<MoveBallGoalOrder*>(order.get());
        std::optional<RelBall> rel_ball = rel_ball_sub.latest();
        dest = {LocalizationUtils::relToAbs(rel_ball->pos_rel, current_pos->position), 0};
        focus = HeadFocus::BALL;
        omni_dist = 3.0f;  // Change also in dribble.cpp!!
    } else {
        auto* kg_order = dynamic_cast<KeepGoalOrder*>(order.get());
        dest = handelHeadAngle(kg_order->pos, WalkToPositionOrder::Mode::SUPPORTER,
                               current_pos->position);
    }

    htwk::log_rerun("soccerfield/agents/walk_to_pos/dest",
                    rerun::LineStrips2D(
                            {{{{current_pos->position.point().x, current_pos->position.point().y},
                               {dest.point().x, dest.point().y}}}}));

    float accuracy = 0.3f;
    if (isOrder<KeepGoalOrder>(order))
        accuracy = 0.2f;
    if (isOrder<WalkToPositionOrder>(order) &&
        dynamic_cast<WalkToPositionOrder*>(order.get())->precise)
        accuracy = 0.1f;
    if ((dest - current_pos->position).weightedNorm(1.f) < accuracy) {
        return MotionCommand::Stand(focus);
    }

    point_2d dest_rel = LocalizationUtils::absToRel(dest.point(), current_pos->position);
    if (isOrder<WalkToPositionOrder>(order) &&
        dynamic_cast<WalkToPositionOrder*>(order.get())->mode ==
                WalkToPositionOrder::Mode::SUPPORTER &&
        gc_state_sub.latest().state == GameState::Playing && dest_rel.norm() >= 1.25f) {
        focus = HeadFocus::LOC;
    }

    // Ignore the whole "we-walk-around-stuff" if we are close to the target (1m) and didn't get an
    // explicit obstacle to avoid from the Order or if we're in FOCUS_DIRECTION mode.
    bool close_obstacle = false;
    for (const auto& obstacle : obstacles) {
        if (obstacle.position.dist(current_pos->position.point()) <= 1.7f) {
            close_obstacle = true;
            break;
        }
    }
    if (focus_a || dest_rel.norm() < omni_dist) {
        if (isOrder<KeepGoalOrder>(order) &&
            std::abs(normalizeRotation(dest.a - current_pos->position.a)) < 90_deg) {
            focus = HeadFocus::BALL_GOALIE;
        }
        return MotionCommand::Walk(
                {.dx = clamp(dest_rel.x * 1.f, -0.15f, 1.0f),
                 .dy = clamp(dest_rel.y * 1.f, -.75f, .75f),
                 .da = clamp(normalizeRotation(dest.a - current_pos->position.a) * 2.f, -1.f, 1.f)},
                focus);
    }

    // Walk around lying robots that we can't see using the far obstacle tracker.
    if (near_obstacle_tracker_result->corridorBlocked(0.75f, 0.4f) &&
        std::abs(current_pos->position.x) < SoccerField::length() / 2.f + 0.4f &&
        std::abs(current_pos->position.y) < SoccerField::width() / 2.f + 0.4f) {
        // TODO: Choose more wisely than just going towards the line between the ball and the own
        // goal.
        avoidance_side_decision_reset_time = time_us() + 3_s;
        if (avoidance_side_decision == 0) {
            for (const point_2d& gp : SoccerField::getGoalPosts()) {
                if (gp.dist(current_pos->position.point()) < 1.f) {
                    avoidance_side_decision = -sgn(Line(gp, gp + point_2d(2 * gp.x, gp.y))
                                                           .side(current_pos->position.point()));
                }
            }
            if (avoidance_side_decision == 0) {
                avoidance_side_decision =
                        sgn(Line(dest.point(), SoccerField::ownGoal())
                                    .angle_to(Line(dest.point(), current_pos->position.point())));
            }
        }
        float speed = 1.0f * avoidance_side_decision;
        bool far_blocked = near_obstacle_tracker_result->corridorBlocked(0.6f, 0.4f);
        bool near_blocked = near_obstacle_tracker_result->corridorBlocked(0.375f, 0.5f);
        if (near_blocked)
            return MotionCommand::Walk(
                    {.dx = -0.13f,
                     .dy = speed / 1.25f,
                     .da = -speed / 1.25f / dest_rel.magnitude() + dest_rel.to_direction()},
                    focus);
        else if (far_blocked)
            return MotionCommand::Walk(
                    {.dx = 0.f,
                     .dy = speed,
                     .da = -speed / dest_rel.magnitude() + dest_rel.to_direction()},
                    focus);
        return MotionCommand::Walk({.dx = 0.12f,
                                    .dy = speed,
                                    .da = -speed / dest_rel.magnitude() + dest_rel.to_direction()},
                                   focus);
    } else if (avoidance_side_decision != 0 && time_us() > avoidance_side_decision_reset_time) {
        avoidance_side_decision = 0;
    }

    // Compose the destination vector by passing the data through the vectorfield, add the target
    // vector (source->target) and transform the resulting vector from absolute to robot relative to
    // get the direction. Fun fact: if we forget the transformation to the relative frame, we start
    // to walk in circles ... because the direction vector does not change, duh!!!1!1
    point_2d dest_vec =
            vectorfield::calcDirection(current_pos->position, dest, obstacles).normalized();
    dest_vec += (dest.point() - current_pos->position.point()).normalized();
    float dest_angle = normalizeRotation(dest_vec.to_direction() - current_pos->position.a);
    float angle_goodness = 1.f - clamp(std::abs(dest_angle) / 90_deg, 0.f, 1.f);

    return MotionCommand::Walk(
            {.dx = 1.0f * angle_goodness,
             .dy = 0.f,
             .da = clamp(dest_angle * 2.f * (1.f - 0.7f * angle_goodness), -1.5f, 1.5f)},
            focus);
}
