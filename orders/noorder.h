#pragma once

#include <source_location>

#include "logging.h"
#include "order.h"

class NoOrder : public Order {
public:
    static std::shared_ptr<NoOrder> create(
            const std::source_location& loc = std::source_location::current()) {
        auto order = std::shared_ptr<NoOrder>(new NoOrder());
        htwk::log_order(*order, "", loc.file_name(), loc.line());
        return order;
    }

private:
    NoOrder() : Order("NoOrder") {}
};
