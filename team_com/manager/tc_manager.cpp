#include "tc_manager.h"

#include <unistd.h>

#include <source_location>

#include "algorithm_ext.h"
#include "gc_pub_sub.h"
#include "localization_utils.h"
#include "logging.h"
#include "named_thread.h"
#include "robot_time.h"
#include "tc_pub_sub.h"

TeamComManager::TeamComManager(PlayerIdx idx) : idx(idx), myself(idx) {
    tc_internal::receive.registerCallback([this](const TeamComData& data) { receive(data); });
}

void TeamComManager::send() {
    GCState gc_state = gc_state_sub.latest();
    if (gc_state.state == GameState::Initial || gc_state.state == GameState::Finished ||
        gc_state.state == GameState::Standby || gc_state.is_fake_gc ||
        gc_state.my_team.players[idx].is_penalized) {
        return;
    }
    TeamComData data = getTeamComMyself();
    int64_t time = getMyself().last_update_us;
    if (time > last_team_com_send + 200_ms) {
        last_team_com_send = time;
        tc_internal::broadcast.publish(data);
    }
    {
        std::lock_guard<std::mutex> lck(robot_mutex);
        robots[data.player_idx] = data;
    }
}

void TeamComManager::receive(const TeamComData& data) {
    if (gc_state_sub.latest().is_fake_gc) {
        return;
    }
    if (data.player_idx == myself.getMyself().idx) {
        return;
    }
    if (data.player_idx < 0 || data.player_idx >= kMaxNumPlayers) {
        LOG_S(ERROR) << "Received team com for player idx " << data.player_idx
                     << " -> something is wrong!";
        return;
    }
    {
        std::lock_guard<std::mutex> lck(robot_mutex);
        if (robots.find(data.player_idx) != robots.end() &&
            data.sent_time_us <= robots[data.player_idx].sent_time_us - 500_ms) {
            LOG_S(INFO) << "Received outdated team com from player idx " << data.player_idx << ": "
                        << data.sent_time_us << " <= " << robots[data.player_idx].sent_time_us;
            return;
        }
        LOG_S(INFO) << (time_us() - data.sent_time_us);
        robots[data.player_idx] = data;
        log_team_com(data);
    }
}

void TeamComManager::log_team_com(const TeamComData& data) {
    htwk::log_robot_position("soccerfield/team_com/robot_" + std::to_string(data.player_idx) + "/",
                             data.pos, 0.0f, rerun::Color(0, 0, 255, 255));

    if (data.ball) {
        point_2d ball_pos = LocalizationUtils::relToAbs(data.ball->pos_rel, data.pos);
        htwk::log_rerun("soccerfield/team_com/robot_" + std::to_string(data.player_idx) + "/ball",
                        rerun::Points2D({{ball_pos.x, ball_pos.y}})
                                .with_radii(rerun::Radius::ui_points(5.f))
                                .with_colors(rerun::Color(255, 255, 0, 255)));
    } else {
        htwk::log_rerun("soccerfield/team_com/robot_" + std::to_string(data.player_idx) + "/ball",
                        rerun::Clear());
    }

    htwk::log_rerun(
            "team_com/robot_" + std::to_string(data.player_idx),
            rerun::TextLog(rerun::Text(data.to_string())).with_level(rerun::TextLogLevel::Debug));
}
