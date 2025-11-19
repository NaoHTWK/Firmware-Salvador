#pragma once

#include "channel.h"
#include "localization_pub_sub.h"
#include "position.h"
#include "team_strategy.h"

class StayStrategy : public TeamStrategy {
public:
    StayStrategy(uint8_t team_nr, PlayerIdx player_idx);

protected:
    std::shared_ptr<Order> ready() override;
    std::shared_ptr<Order> set() override;
    std::shared_ptr<Order> play() override;

private:
    htwk::ChannelSubscriber<LocPosition> position;
    int64_t time = time_us();
    int pos = 0;
};
