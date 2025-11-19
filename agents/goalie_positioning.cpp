#include "goalie_positioning.h"

#include <keepgoalorder.h>
#include <line.h>
#include <localization_utils.h>
#include <soccerfield.h>

#include <cmath>
#include <iostream>
#include <optional>

using namespace htwk;
using namespace std;

MotionCommand GoaliePositioningAgent::proceed(std::shared_ptr<Order> order) {
    if (!isOrder<KeepGoalOrder>(order))
        return MotionCommand::Nothing;

    auto* keepGoalOrder = dynamic_cast<KeepGoalOrder*>(order.get());
    if(keepGoalOrder->isPenaltyShot) {
        return MotionCommand::Nothing;
    }
    focus = HeadFocus::BALL;

    std::optional<RelBall> rel_ball = rel_ball_sub.latest();
    std::optional<LocPosition> pos_opt = pos_sub.latest();
    std::optional<TeamComData> striker = striker_sub.latest();

    Position pos;
    if (!pos_opt) {
        return MotionCommand::Nothing;
    } else {
        pos = pos_opt->position;
    }

    if (pos.x > -SoccerField::length() / 2.f + SoccerField::goalBoxWidth() ||
        abs(pos.y) > SoccerField::goalBoxHeight() / 2.f)
        return keepGoalOrder->allowedToMove ? MotionCommand::Nothing : MotionCommand::Stand();

    // chose which ball to use
    if (rel_ball) {
        ball = LocalizationUtils::relToAbs(rel_ball->pos_rel, pos);
    } else if (striker && striker->ball) {
        ball = LocalizationUtils::relToAbs(striker->ball->pos_rel, striker->pos);
    } else {
        return keepGoalOrder->allowedToMove ? MotionCommand::Nothing : MotionCommand::Stand();
    }

    // Position between the ball and goal mid
    htwk::Line goal_ball_line(own_goal, ball);
    Position dest = {goal_ball_line.p1() + 1.5f * goal_ball_line.u().normalized(),
        goal_ball_line.u().to_direction()};
    // limits goalies distance from goal, to avoid being to far away or looking at the goal post during corner kick
    dest.x = clamp(dest.x, -SoccerField::length() / 2.f + 0.3f, -SoccerField::length() / 2 + 1.f);
    
    if ((dest - pos).weightedNorm(1.f) < .2f) {
        return MotionCommand::Stand(focus);
    }

    point_2d dest_rel = LocalizationUtils::absToRel(dest.point(), pos);

    float dest_angle = normalizeRotation(dest.a - pos.a);
    float angle_goodness = 1.f - clamp(std::abs(dest_angle) / 90_deg, 0.f, 1.f);

    return MotionCommand::Walk(
            {clamp(dest_rel.x, -1.f, 1.f), clamp(dest_rel.y, -1.f, 1.f),
             clamp(dest_angle * 2.f, -1.5f, 1.5f)},
            focus);
}