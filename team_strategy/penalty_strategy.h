#pragma once

#include "keepgoalorder.h"
#include "noorder.h"
#include "point_2d.h"
#include "shootorder.h"
#include "team_strategy.h"
#include "moveballgoalorder.h"
#include "walktopositionorder.h"
#include "soccerfield.h"

class PenaltyStrategy : public TeamStrategy {
public:
    using TeamStrategy::TeamStrategy;

protected:
    std::shared_ptr<Order> ready() override {
        if (player_idx == 0) {
            return WalkToPositionOrder::create({-SoccerField::length() / 2.f, 0.f});
        }
        return NoOrder::create(); 
    }
    std::shared_ptr<Order> set() override {
        return NoOrder::create();
    }
    std::shared_ptr<Order> play() override {
        if (player_idx == 0) {
            return KeepGoalOrder::create(true);
        }
        //return ShootOrder::create(point_2d{4.5f, -.3f});
        return MoveBallGoalOrder::create();
    }
};
