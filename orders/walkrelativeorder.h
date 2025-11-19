#pragma once

#include <source_location>

#include "logging.h"
#include "order.h"
#include "position.h"

class WalkRelativeOrder : public Order {
public:
    static std::shared_ptr<WalkRelativeOrder> create(
            const htwk::Position& pos,
            const std::source_location& loc = std::source_location::current()) {
        auto order = std::shared_ptr<WalkRelativeOrder>(new WalkRelativeOrder(pos));
        htwk::log_order(*order, "pos: " + std::to_string(pos.x) + ", " + std::to_string(pos.y),
                        loc.file_name(), loc.line());
        return order;
    }

    const htwk::Position pos;

private:
    WalkRelativeOrder(const htwk::Position& pos) : Order("WalkRelativeOrder"), pos(pos) {}
};
