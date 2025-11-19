#include <goalie_dive.h>
#include <localization_utils.h>
#include <keepgoalorder.h>

#include "logging.h"

using namespace htwk;
using namespace std;

MotionCommand GoalieDiveAgent::proceed(std::shared_ptr<Order> order) {
    if (!isOrder<KeepGoalOrder>(order))
        return MotionCommand::Nothing;
    auto* keepGoalOrder = dynamic_cast<KeepGoalOrder*>(order.get());
    if(!keepGoalOrder->isPenaltyShot) {
        return MotionCommand::Nothing;
    }

    std::optional<RelBall> rel_ball = rel_ball_sub.latest();
    std::optional<LocPosition> pos = pos_sub.latest();

     if (!rel_ball || !pos) {
        return MotionCommand::Nothing;
    }

    dest.a = rel_ball->pos_rel.to_direction();
    if ((dest - pos->position).norm() < 0.05f) {
        float side_dist = .25f;
        float sign = d(gen) ? -1.f : 1.f;
        dest.y = dest.y == 0.f ? side_dist * sign : 0.f;
    }

    //keepGoalOrder->pos = dest;
    //return MotionCommand::Nothing;
    return MotionCommand::Stand();
}
