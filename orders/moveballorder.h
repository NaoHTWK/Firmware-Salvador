#pragma once

#include <source_location>

#include "logging.h"
#include "order.h"
#include "point_2d.h"

class MoveBallOrder : public Order {
public:
    static std::shared_ptr<MoveBallOrder> create(
            float x, float y, const std::source_location& loc = std::source_location::current()) {
        auto order = std::shared_ptr<MoveBallOrder>(new MoveBallOrder(x, y));
        htwk::log_order(*order, "ball position: " + std::to_string(x) + ", " + std::to_string(y),
                        loc.file_name(), loc.line());
        return order;
    }
    static std::shared_ptr<MoveBallOrder> create(
            point_2d pos, const std::source_location& loc = std::source_location::current()) {
        auto order = std::shared_ptr<MoveBallOrder>(new MoveBallOrder(pos));
        htwk::log_order(*order,
                        "ball position: " + std::to_string(pos.x) + ", " + std::to_string(pos.y),
                        loc.file_name(), loc.line());
        return order;
    }

    point_2d pos;

private:
    MoveBallOrder(float _x, float _y) : Order("MoveBallOrder"), pos(point_2d(_x, _y)) {}
    MoveBallOrder(point_2d pos) : Order("MoveBallOrder"), pos(pos) {}
};
