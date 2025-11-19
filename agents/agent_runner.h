#pragma once

#include <agent_base.h>
#include <async.h>
#include <localization_pub_sub.h>
#include <noorder.h>

#include <memory>
#include <shared_mutex>
#include <thread>
#include <tuple>
#include <vector>

#include "team_strategy_pub_sub.h"

class AgentRunner {
public:
    AgentRunner(std::string agent_selection);

    void proceed();

    static std::list<std::string> valid_agent_selections() {
        return {"competition", "walktuner", "ballIntercept"};
    };

private:
    std::vector<std::shared_ptr<AgentBase>> agents;
    std::unique_ptr<ThreadPool> thread_pool;

    htwk::ChannelSubscriber<LocPosition> localization_subscriber =
            loc_position_channel.create_subscriber();
    htwk::ChannelSubscriber<std::shared_ptr<Order>> team_strategy_order_subscriber =
            team_strategy_order_channel.create_subscriber();

    void log_motion_command(const MotionCommand& mc);
    void log_order(const Order& order, std::string agent_name);
};
