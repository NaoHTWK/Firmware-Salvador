#pragma once

#include <agent_base.h>
#include <soccerfield.h>

#include "localization_pub_sub.h"
#include "multi_target_tracker_pub_sub.h"

#include <random>

using namespace htwk;

class GoalieDiveAgent : public AgentBase {
public:
    GoalieDiveAgent() : AgentBase("GoalieDive"), rd(), gen(rd()), d(0.5) {}
    MotionCommand proceed(std::shared_ptr<Order> order) override;

private:
    const point_2d own_goal = point_2d(-SoccerField::length() / 2.f, 0.f);

    Position dest = {own_goal, 0.f};

    ChannelSubscriber<LocPosition> pos_sub = loc_position_channel.create_subscriber();
    ChannelSubscriber<std::optional<RelBall>> rel_ball_sub = rel_ball_channel.create_subscriber();

    HeadFocus focus;
    point_2d ball;


    std::random_device rd;
    std::mt19937 gen;
    std::bernoulli_distribution d;
};
