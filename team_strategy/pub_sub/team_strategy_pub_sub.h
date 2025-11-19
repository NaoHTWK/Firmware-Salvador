#include "channel.h"
#include "order.h"
#include "tc_data.h"

extern htwk::Channel<std::optional<TeamComData>> striker_channel;
extern htwk::Channel<std::shared_ptr<Order>> team_strategy_order_channel;
