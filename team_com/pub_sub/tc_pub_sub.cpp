#include "tc_pub_sub.h"

namespace tc_internal {

Queue<TeamComData> broadcast;
Callback<void(const TeamComData&)> receive;

}  // namespace tc_internal
