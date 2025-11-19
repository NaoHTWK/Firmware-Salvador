#pragma once

#include <agent_base.h>
#include <soccerfield.h>

#include "localization_pub_sub.h"
#include "multi_target_tracker_pub_sub.h"
#include "team_strategy_pub_sub.h"

using namespace htwk;

class GoaliePositioningAgent : public AgentBase {
public:
    GoaliePositioningAgent() : AgentBase("GoaliePositioning") {}
    MotionCommand proceed(std::shared_ptr<Order> order) override;

private:
    const point_2d own_goal = point_2d(-SoccerField::length() / 2.f, 0.f);

    ChannelSubscriber<LocPosition> pos_sub = loc_position_channel.create_subscriber();
    ChannelSubscriber<std::optional<RelBall>> rel_ball_sub = rel_ball_channel.create_subscriber();

    ChannelSubscriber<std::optional<TeamComData>> striker_sub = striker_channel.create_subscriber();

    HeadFocus focus;
    point_2d ball;
};
