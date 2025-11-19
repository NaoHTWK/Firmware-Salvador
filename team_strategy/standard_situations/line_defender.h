#pragma once

#include <map>
#include <optional>
#include <vector>

#include "order.h"
#include "point_2d.h"
#include "soccerfield.h"
#include "tc_data.h"

class LineDefender {
public:
    LineDefender() = default;
    std::shared_ptr<Order> proceedSetPiece(const point_2d& teamball,
                                           const std::map<PlayerIdx, TeamComData>& robots,
                                           const Myself& myself);
    void reset_fixedPosition() {
        pos = std::nullopt;
    }

private:
    static constexpr PlayerIdx goaly_idx = 0;

    static constexpr float thres_corner = -4.f;
    static constexpr float thres_cover = -1.f;
    const point_2d own_goal = point_2d(-SoccerField::length() / 2.f, 0);
    enum class Roles {
        GOALY,
        BACK_WALL,
        FRONT_WALL,
        SIDE_WALL,
        COVER,
        FRONT_WALL_REL,
        COUNTER,
        FIELD_COVER
    };

    point_2d selectPosition(Roles role, const point_2d& teamball, const Myself& myself);
    std::vector<htwk::Position> selectRole(const point_2d& teamball,
                                           const std::map<PlayerIdx, TeamComData>& robots,
                                           const Myself& myself);
    static bool compareByX(const htwk::Position& p1, const htwk::Position& p2) {
        return p1.x < p2.x;
    }
    std::optional<htwk::Position> pos;
};
