#pragma once

#include <source_location>

#include "logging.h"
#include "order.h"

class MoveBallGoalOrder : public Order {
public:
    static std::shared_ptr<MoveBallGoalOrder> create(
            const std::source_location& loc = std::source_location::current()) {
        auto order = std::shared_ptr<MoveBallGoalOrder>(new MoveBallGoalOrder());
        htwk::log_order(*order, "", loc.file_name(), loc.line());
        return order;
    }
    ~MoveBallGoalOrder() override = default;

private:
    MoveBallGoalOrder() : Order("MoveBallGoalOrder") {}
    MoveBallGoalOrder(const MoveBallGoalOrder&) = delete;
    MoveBallGoalOrder(MoveBallGoalOrder&&) = delete;
    MoveBallGoalOrder& operator=(MoveBallGoalOrder&) = delete;
    MoveBallGoalOrder& operator=(MoveBallGoalOrder&&) = delete;
};
