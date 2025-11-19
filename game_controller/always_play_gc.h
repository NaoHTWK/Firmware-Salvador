#pragma once

#include "gc_pub_sub.h"
#include "gc_state.h"
#include "named_thread.h"

class AlwaysPlayGC : public GCInterface {
public:
    AlwaysPlayGC(TeamIdx team_idx, PlayerIdx player_idx)
        : player_idx(player_idx), team_idx(team_idx) {
        t = named_thread("AlwaysPlayGC", [this]() { run(); });
    }

    ~AlwaysPlayGC() override {
        shutdown = true;
        t.join();
    }

private:
    void run() {
        while (!shutdown) {
            GCState gc_state;
            gc_state.player_idx = player_idx;
            gc_state.my_team.team_nr = team_idx;
            gc_state.opp_team.team_nr = 99;
            gc_state.my_team.players.resize(5);
            gc_state.opp_team.players.resize(5);
            for (int i = 0; i < 5; i++) {
                gc_state.my_team.players[i].is_penalized = false;
                gc_state.opp_team.players[i].is_penalized = false;
            }
            gc_state.state = GameState::Playing;
            gc_state.secondary_state = SecondaryState::Normal;
            gc_state.kicking_team = KickingTeam::MyTeam;
            gc_state.secs_remaining = 500;
            gc_state.secondary_time = 0;
            gc_state.remaining_message_budget = 10000;
            gc_state.game_phase = GCState::GamePhase::FIRST_HALF;
            gc_state.is_fake_gc = true;
            ::gc_state.publish(gc_state);
            usleep(500'000);
        }
    }

    PlayerIdx player_idx;
    std::thread t;
    bool shutdown = false;
    TeamIdx team_idx;
};