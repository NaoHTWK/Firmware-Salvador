#pragma once

#include <head_focus.h>
#include <order.h>
#include <position.h>

#include <source_location>

#include "logging.h"

class WalkCustomOrder : public Order {
public:
    static std::shared_ptr<WalkCustomOrder> create(
            const htwk::Position& pos,
            const std::source_location& loc = std::source_location::current()) {
        auto order = std::shared_ptr<WalkCustomOrder>(new WalkCustomOrder(pos, loc));
        htwk::log_order(*order, "pos: " + std::to_string(pos.x) + ", " + std::to_string(pos.y),
                        loc.file_name(), loc.line());
        return order;
    }

    const htwk::Position pos;

private:
    WalkCustomOrder(const htwk::Position& pos,
                    const std::source_location& loc = std::source_location::current())
        : Order("WalkCustomOrder"), pos(pos) {}
};
