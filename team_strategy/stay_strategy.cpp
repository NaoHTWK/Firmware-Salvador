#include "head_focus.h"
#include "noorder.h"
#include "stay_strategy.h"
#include "walktopositionorder.h"

StayStrategy::StayStrategy(uint8_t team_nr, PlayerIdx player_idx)
    : TeamStrategy(team_nr, player_idx), position(loc_position_channel.create_subscriber()) {}

std::shared_ptr<Order> StayStrategy::ready() {
    return WalkToPositionOrder::create(position.latest().position,
                                       WalkToPositionOrder::Mode::SUPPORTER);
}
std::shared_ptr<Order> StayStrategy::set() {
    return WalkToPositionOrder::create(position.latest().position,
                                       WalkToPositionOrder::Mode::SUPPORTER);
}
std::shared_ptr<Order> StayStrategy::play() {
    // if (time_us() - time > 20_s) {
    //     pos = (pos + 1) % 2;
    //     time = time_us();
    // }
    // switch (pos) {
    //     case 0:
    //         return WalkToPositionOrder::create({4, 0, 0}, WalkToPositionOrder::Mode::USE_A,
    //         false,
    //                                            HeadFocus::LOC);
    //     default:
    //         return WalkToPositionOrder::create({-4, 0, 0}, WalkToPositionOrder::Mode::USE_A,
    //         false,
    //                                            HeadFocus::LOC);
    // }
    return WalkToPositionOrder::create(position.latest().position, WalkToPositionOrder::Mode::USE_A,
                                       false, HeadFocus::BALL);
}
