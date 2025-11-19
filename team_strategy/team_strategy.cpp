#include "team_strategy.h"

#include "noorder.h"

void TeamStrategy::proceed() {
    gc_data = gc_state_sub.latest();
    std::shared_ptr<Order> o = NoOrder::create();
    if (gc_data.state == GameState::Initial) {
        prev_state = gc_data.state;
        team_strategy_order_channel.publish(o);
        return;
    }
    if (gc_data.state != prev_state || (gc_data.secondary_state != prev_secondary_state &&
                                        prev_secondary_state == SecondaryState::Normal)) {
        phase_switch = time_us();
    }
    robots = tc_manager.getRobots();
    myself = tc_manager.getMyself(true);
    robots[myself.idx] = tc_manager.getTeamComMyself();
    std::erase_if(robots, [this](const auto& robot) {
        return gc_data.my_team.players[robot.first].is_penalized ||
               robot.second.sent_time_us < time - 5_s;
    });
    time = myself.last_update_us;
    penalized = gc_data.my_team.players[player_idx].is_penalized;
    if (!penalized && prev_penalized) {
        penalty_return_us = time;
        returning_from_penalty = true;
    }
    switch (gc_data.state) {
        case GameState::Ready:
            time_us_lastKickoff = time;
            setPlay_type = SecondaryState::Normal;
            o = ready();
            break;
        case GameState::Set:
            time_us_lastKickoff = time;
            setPlay_type = SecondaryState::Normal;
            o = set();
            break;
        case GameState::Playing:
            time_us_inPlay = time - time_us_lastKickoff;
            time_us_inSetPlayPlay = time - time_us_lastSetPlayKickoff;
            if (gc_data.secondary_state == SecondaryState::Normal ||
                gc_data.secondary_state == SecondaryState::Overtime) {
                o = play();
            } else if (gc_data.setPlay_state == GameState::Initial) {
                o = setPlay_initial();
                time_us_lastSetPlayPlacement = time_us();
            } else if (gc_data.setPlay_state == GameState::Ready) {
                o = setPlay_ready();
            } else {
                time_us_lastSetPlayKickoff = time;
                o = setPlay_set();
            }
            break;
        default:
            setPlay_type = SecondaryState::Normal;
            time_us_lastKickoff = time;
    }
    if (!penalized && !returning_from_penalty) {
        tc_manager.send();
    }

    prev_state = gc_data.state;
    prev_secondary_state = gc_data.secondary_state;
    prev_penalized = penalized;
    team_strategy_order_channel.publish(o);
}
