#pragma once

#include <source_location>

#include "logging.h"
#include "order.h"
#include "point_2d.h"
#include "position.h"
#include "soccerfield.h"

class KeepGoalOrder : public Order {
public:
    static std::shared_ptr<KeepGoalOrder> create(
            bool isPenaltyShot = false,
            bool allowedToMove = true) {
        const std::source_location& loc = std::source_location::current();
        const point_2d& own_goal = {-SoccerField::length() / 2.f, 0.f};
        point_2d offset = {};
        if(!isPenaltyShot) {
            offset = {1.f, 0.f};
        }
        auto order = std::shared_ptr<KeepGoalOrder>(new KeepGoalOrder(isPenaltyShot, allowedToMove, own_goal + offset));
        htwk::log_order(*order, "", loc.file_name(), loc.line());
        return order;
    }

    ~KeepGoalOrder() override = default;

    const bool allowedToMove;
    htwk::Position pos;
    const bool isPenaltyShot;

private:
    KeepGoalOrder(const KeepGoalOrder&) = delete;
    KeepGoalOrder(KeepGoalOrder&&) = delete;
    KeepGoalOrder& operator=(KeepGoalOrder&) = delete;
    KeepGoalOrder& operator=(KeepGoalOrder&&) = delete;

    KeepGoalOrder(bool isPenaltyShot, bool allowedToMove, const point_2d& walkToPos)
        : Order("KeepGoalOrder"), isPenaltyShot(isPenaltyShot), allowedToMove(allowedToMove), pos(walkToPos, 0.f) {}
};
