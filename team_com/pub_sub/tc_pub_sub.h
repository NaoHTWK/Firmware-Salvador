#pragma once

#include "callback.h"
#include "queue.h"
#include "tc_data.h"

namespace tc_internal {

// The channels/queues defined here are used between the TeamCom manager and its various network
// implementations. Don't directly depend on this, use TeamComManager instead which handles
// message budgets etc.

extern Queue<TeamComData> broadcast;
extern Callback<void(const TeamComData&)> receive;

}  // namespace tc_internal
