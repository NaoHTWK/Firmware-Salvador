#pragma once

#include <map>
#include <vector>

#include "gc_state.h"
#include "order.h"
#include "point_2d.h"
#include "soccerfield.h"
#include "tc_data.h"

class PenaltyKick {
public:
    PenaltyKick(PlayerIdx player_idx) : player_idx(player_idx) {}
    std::shared_ptr<Order> proceedSetPiece(const point_2d& ball,
                                           const std::map<PlayerIdx, TeamComData>& alive_robots,
                                           const GCState& gc_data);

private:
    std::shared_ptr<Order> ready_positioning_def();
    std::shared_ptr<Order> ready_positioning_atk();

    static constexpr PlayerIdx goaly_idx = 0;
    const point_2d own_goal = point_2d(-SoccerField::length() / 2.f, 0);
    const std::vector<std::vector<htwk::Position>> pos_def = {
            {{-4.25f, 0.f, 0.f}, {-2.5f, 2.5f, M_PIf}},
            {{-4.25f, 0.f, 0.f}, {-2.5f, 2.5f, M_PIf}, {-2.25f, -1.f, M_PIf}},
            {{-4.25f, 0.f, 0.f}, {-2.5f, 2.5f, M_PIf}, {-2.25f, -1.f, M_PIf}, {-2.25f, 1.f, M_PIf}},
            {{-4.25f, 0.f, 0.f},
             {-2.5f, 2.5f, M_PIf},
             {-2.25f, -1.f, M_PIf},
             {-2.25f, 1.f, M_PIf},
             {-2.5f, -2.5f, M_PIf}},
            {{-4.25f, 0.f, 0.f},
             {-2.5f, 2.5f, M_PIf},
             {-2.25f, -1.f, M_PIf},
             {-2.25f, 1.f, M_PIf},
             {-2.5f, -2.5f, M_PIf},
             {-2.5f, 2.5f, M_PIf}},
            {{-4.25f, 0.f, 0.f},
             {-2.5f, 2.5f, M_PIf},
             {-2.25f, -1.f, M_PIf},
             {-2.25f, 1.f, M_PIf},
             {-2.5f, -2.5f, M_PIf},
             {-2.5f, 2.5f, M_PIf},
             {-1.f, 1.5f, M_PIf}},
            {{-4.25f, 0.f, 0.f},
             {-2.5f, 2.5f, M_PIf},
             {-2.25f, -1.f, M_PIf},
             {-2.25f, 1.f, M_PIf},
             {-2.5f, -2.5f, M_PIf},
             {-2.5f, 2.5f, M_PIf},
             {-1.f, -1.5f, M_PIf},
             {-1.f, 1.5f, M_PIf}}};
    const htwk::Position pos_atk_striker =
            htwk::Position{4.5f - SoccerField::penaltyAreaWidth(), 0.f, 0.f};
    const htwk::Position pos_atk_shadow =
            htwk::Position{4.5f - SoccerField::penaltyAreaWidth() - 0.75f, 0.5f, 0.f};

    const PlayerIdx player_idx;
    point_2d ball{0, 0};
    std::map<PlayerIdx, TeamComData> alive_robots;
    TeamComData myself;
};
