#pragma once

#include <noorder.h>
#include <team_strategy.h>

class NoOpStrategy : public TeamStrategy {
public:
    using TeamStrategy::TeamStrategy;

protected:
    std::shared_ptr<Order> ready() override {
        return NoOrder::create();
    }
    std::shared_ptr<Order> set() override {
        return NoOrder::create();
    }
    std::shared_ptr<Order> play() override {
        return NoOrder::create();
    }
};
