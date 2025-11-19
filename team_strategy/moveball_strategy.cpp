#include "moveball_strategy.h"

#include "moveballgoalorder.h"

std::shared_ptr<Order> MoveBallStrategy::ready() {
    return MoveBallGoalOrder::create();
}

std::shared_ptr<Order> MoveBallStrategy::set() {
    return MoveBallGoalOrder::create();
}
std::shared_ptr<Order> MoveBallStrategy::play() {
    return MoveBallGoalOrder::create();
}
