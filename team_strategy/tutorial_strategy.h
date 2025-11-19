#pragma once

#include "noorder.h"
#include "position.h"
#include "team_strategy.h"
#include "walktopositionorder.h"
#include "keepgoalorder.h"

class TutorialStrategy : public TeamStrategy {
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
