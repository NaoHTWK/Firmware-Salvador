#pragma once

#include <source_location>

#include "order.h"
#include "point_2d.h"

class PenaltyDiveOrder : public Order {
public:
    static std::shared_ptr<PenaltyDiveOrder> create() {
        return std::shared_ptr<PenaltyDiveOrder>();
    }
};
