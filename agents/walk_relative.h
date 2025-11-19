#pragma once

#include "agent_base.h"
#include "robot_time.h"
#include "walkrelativeorder.h"

class WalkRelativeAgent : public AgentBase {
public:
    WalkRelativeAgent() : AgentBase("WalkRelative") {}
    MotionCommand proceed(std::shared_ptr<Order> order) override;

private:
    float avoidance_side_decision = 0;
    int64_t avoidance_side_decision_reset_time = time_us();
};
