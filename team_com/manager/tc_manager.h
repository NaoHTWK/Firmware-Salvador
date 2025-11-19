#pragma once

#include <loguru.hpp>
#include <map>
#include <source_location>

#include "gc_pub_sub.h"
#include "rel_ball.h"
#include "tc_data.h"

// When starting the TeamComManager, until we've received the first game controller message, we
// assume that our message budget is 0.
class TeamComManager {
public:
    TeamComManager(PlayerIdx idx);
    void strikerRequest() {
        striker_request_time_us = getMyself().last_update_us;
    }
    void resetStrikerRequest() {
        striker_request_time_us = 0;
    }
    void send();

    std::map<PlayerIdx, TeamComData> getRobots() {
        std::lock_guard<std::mutex> lck(robot_mutex);
        return robots;
    }

    Myself getMyself(bool update = false) {
        std::lock_guard<std::mutex> lck(myself_mutex);
        return myself.getMyself(update);
    }

    TeamComData getTeamComMyself() {
        Myself myself = getMyself();
        TeamComData data;
        data.striker_request_time_us = striker_request_time_us;
        data.sent_time_us = myself.last_update_us;
        data.player_idx = myself.idx;
        data.pos = myself.loc.position;
        data.loc_quality = myself.loc.quality;
        if (myself.ball) {
            data.ball = TeamComBall{.pos_rel = myself.ball->pos_rel,
                                    .ball_age_us = myself.ball->ball_age_us};
        }
        data.is_fallen = myself.fallen.type != htwk::FallDownStateType::READY;
        data.robots = myself.robots;
        return data;
    }

private:
    void receive(const TeamComData& data);

    int64_t striker_request_time_us = 0;
    std::mutex robot_mutex;
    std::map<PlayerIdx, TeamComData> robots;
    std::mutex myself_mutex;
    MyselfProvider myself;
    PlayerIdx idx;
    htwk::ChannelSubscriber<GCState> gc_state_sub = gc_state.create_subscriber();
    int64_t last_team_com_send = 0;

    void log_team_com(const TeamComData& data);
};
