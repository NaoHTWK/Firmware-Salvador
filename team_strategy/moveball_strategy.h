#pragma once

#include "team_strategy.h"

class MoveBallStrategy : public TeamStrategy {
public:
    using TeamStrategy::TeamStrategy;

protected:
    std::shared_ptr<Order> ready() override;
    std::shared_ptr<Order> set() override;
    std::shared_ptr<Order> play() override;
};
