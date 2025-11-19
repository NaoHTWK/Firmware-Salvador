#include "team_strategy_pub_sub.h"

htwk::Channel<std::optional<TeamComData>> striker_channel;
htwk::Channel<std::shared_ptr<Order>> team_strategy_order_channel;
