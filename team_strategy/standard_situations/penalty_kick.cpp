#include "penalty_kick.h"

#include <optional>

#include "keepgoalorder.h"
#include "noorder.h"
#include "ready_positioning.h"
#include "soccerfield.h"
#include "stl_ext.h"
#include "walktopositionorder.h"

using namespace std;

shared_ptr<Order> PenaltyKick::proceedSetPiece(const point_2d& ball,
                                               const std::map<PlayerIdx, TeamComData>& alive_robots,
                                               const GCState& gc_data) {
    this->ball = ball;
    this->alive_robots = alive_robots;
    if (auto me = find(alive_robots, player_idx)) {
        myself = *me;
    } else {
        return NoOrder::create();
    }

    if (player_idx == goaly_idx && gc_data.state == GameState::Playing)
        return KeepGoalOrder::create(true);

    if (gc_data.kicking_team == KickingTeam::Unknown) {
        // TODO: fall back to defensive behaviour until we implemented looking at the ref from the
        // 2025 rules.
        return ready_positioning_def();
    } else if (gc_data.kicking_team == KickingTeam::MyTeam) {
        return ready_positioning_atk();
    }

    return ready_positioning_def();
}

shared_ptr<Order> PenaltyKick::ready_positioning_def() {
    const int ready_index = (alive_robots.size() > 0) ? alive_robots.size() - 1 : 0;
    optional<htwk::Position> pos =
            robocupReadyPositioning(pos_def.at(ready_index), alive_robots, player_idx);
    if (!pos)
        return NoOrder::create();
    return WalkToPositionOrder::create(*pos, WalkToPositionOrder::Mode::USE_A, false,
                                       HeadFocus::LOC);
}

shared_ptr<Order> PenaltyKick::ready_positioning_atk() {
    multimap<float, PlayerIdx> robots_by_x;
    for (const auto& [idx, robot] : alive_robots) {
        robots_by_x.emplace(robot.pos.x, idx);
    }
    const point_2d stay_pos(clamp(myself.pos.point().x, -SoccerField::length() * 0.5f, 2.f),
                            myself.pos.point().y);

    int count = 0;
    int itsme = 0;
    for (auto it = robots_by_x.rbegin(); it != robots_by_x.rend(); ++it) {
        count++;
        if (it->second == player_idx) {
            itsme = count;
            break;
        }
        if (alive_robots.size() < 4)
            break;
    }

    switch (itsme) {
        case 1:
            return WalkToPositionOrder::create(pos_atk_striker, WalkToPositionOrder::Mode::STRIKER);
        case 2:
            return WalkToPositionOrder::create(pos_atk_shadow,
                                               WalkToPositionOrder::Mode::SUPPORTER);
        default:
            return WalkToPositionOrder::create(htwk::Position(stay_pos, 0.f),
                                               WalkToPositionOrder::Mode::SUPPORTER);
    }
}
