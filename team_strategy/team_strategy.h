#pragma once

#include <memory>
#include <vector>

#include "channel.h"
#include "gc_pub_sub.h"
#include "gc_state.h"
#include "noorder.h"
#include "order.h"
#include "robot_time.h"
#include "tc_data.h"
#include "tc_manager.h"
#include "team_strategy_pub_sub.h"

class TeamStrategy {
public:
    TeamStrategy(uint8_t team_nr, PlayerIdx player_idx)
        : team_nr(team_nr),
          player_idx(player_idx),
          gc_state_sub(gc_state.create_subscriber()),
          tc_manager(player_idx) {}
    virtual ~TeamStrategy() = default;

    TeamStrategy(TeamStrategy&) = delete;
    TeamStrategy(TeamStrategy&&) = delete;
    TeamStrategy& operator=(const TeamStrategy&) = delete;
    TeamStrategy& operator=(TeamStrategy&&) = delete;

    void proceed();

protected:
    virtual std::shared_ptr<Order> ready() = 0;
    virtual std::shared_ptr<Order> set() = 0;
    virtual std::shared_ptr<Order> play() = 0;
    virtual std::shared_ptr<Order> setPlay_initial() {
        return NoOrder::create();
    }
    virtual std::shared_ptr<Order> setPlay_ready() {
        return NoOrder::create();
    }
    virtual std::shared_ptr<Order> setPlay_set() {
        return NoOrder::create();
    }

    const uint8_t team_nr;
    const PlayerIdx player_idx;

    int64_t time_us_lastKickoff = time_us();
    int64_t time_us_inPlay = 0;
    int64_t time_us_lastSetPlayKickoff = time_us();
    int64_t time_us_inSetPlayPlay = 0;
    int64_t time_us_lastSetPlayPlacement = time_us();
    SecondaryState setPlay_type = SecondaryState::Normal;

    GCState gc_data;
    htwk::ChannelSubscriber<GCState> gc_state_sub;

    TeamComManager tc_manager;
    Myself myself;
    int64_t time;
    std::map<PlayerIdx, TeamComData> robots;
    bool penalized = false;
    bool prev_penalized = false;
    int64_t penalty_return_us = 0;
    bool returning_from_penalty = false;

    GameState prev_state = GameState::Initial;
    SecondaryState prev_secondary_state = SecondaryState::Normal;

    int64_t phase_switch = time_us();
};
