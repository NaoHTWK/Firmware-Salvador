#pragma once

#include <deque>
#include <map>
#include <optional>
#include <vector>

#include "order.h"
#include "point_2d.h"
#include "position_area.h"
#include "soccerfield.h"
#include "tc_data.h"

class FieldDefender {
public:
    FieldDefender() = default;
    std::shared_ptr<Order> proceedSetPiece(const point_2d& teamball,
                                           const std::map<PlayerIdx, TeamComData>& robots,
                                           const Myself& myself);
    void reset_fixedPosition() {
        pos = std::nullopt;
    }

private:
    const PositionArea posArea_critical =
            PositionArea({-SoccerField::length() * .5f, -SoccerField::width() * .5f},
                         {-SoccerField::length() * .5f + SoccerField::length() * .5f * .7f,
                          SoccerField::width() * .5f},
                         {0.2f, 0.2f});
    const PositionArea posArea_goal =
            PositionArea({-SoccerField::length() * .5f, -SoccerField::goalBoxHeight() * .5f - 0.2f},
                         {-SoccerField::length() * .5f + SoccerField::goalBoxWidth() + 0.2f,
                          SoccerField::goalBoxHeight() * .5f + 0.2f},
                         {0.25f, 0.25f});
    const PositionArea posArea_move =
            PositionArea({-SoccerField::length() * .5f, -SoccerField::width() * .5f + 0.25f},
                         {0.f, SoccerField::width() * .5f - 0.25f}, {0.2f, 0.2f});
    htwk::Line own_goal_line = htwk::Line({-SoccerField::length() * .5f, -SoccerField::width()},
                                          {-SoccerField::length() * .5f, SoccerField::width()});
    static constexpr PlayerIdx goaly_idx = 0;
    static constexpr float thres_corner = -4.f;
    static constexpr float thres_cover = -1.f;
    const float fieldwidth_offset = SoccerField::width() * .5f * .9f;
    const point_2d own_goal = point_2d(-SoccerField::length() / 2.f, 0);
    enum class Roles {
        GOALY,
        BACK_WALL,
        FRONT_WALL,
        SIDE_WALL,
        COVER,
        CRITICAL,
        FRONT_WALL_REL,
        BACK_WALL2,
        SIDE_WALL2
    };

    point_2d selectPosition(Roles role, const point_2d& teamball);
    std::vector<htwk::Position> selectRole(const point_2d& teamball,
                                           const std::map<PlayerIdx, TeamComData>& robots);
    Roles selectRole_criticalArea(const std::map<PlayerIdx, TeamComData>& robots,
                                  PlayerIdx player_idx, const point_2d& teamball);
    point_2d selectPosition_criticalArea(Roles role, const point_2d& teamball,
                                         const std::map<PlayerIdx, TeamComData>& robots,
                                         const Myself& myself);
    float getCriticalScore(const point_2d& teamball, const std::map<PlayerIdx, TeamComData>& robots,
                           const Myself& myself, const point_2d& sim_pos);
    static bool compareByX(const htwk::Position& p1, const htwk::Position& p2) {
        return p1.x < p2.x;
    }

    std::deque<point_2d> my_walkToPos_history;
    std::optional<htwk::Position> pos;
};
