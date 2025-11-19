#pragma once

#include <memory>

#include "channel.h"
#include "fall_down_state.h"
#include "imu_joint_state.h"
#include "map_store.h"
#include "odometer.h"
#include "position.h"

namespace htwk {

extern Channel<std::shared_ptr<IMUJointState>> imu_joint_states_channel;
extern Channel<FallDownState> fallen_channel;

extern Channel<std::shared_ptr<Odometer>> odometer_channel;

// TODO: Make sure this doesn't grow unbounded!
extern MapStore<int64_t /* time_us*/, Odometer> odometer_history;

Position deltaOdoRelativeWorld(int64_t from_time_us, int64_t to_time_us);
Position deltaOdoRelativeFrame(int64_t from_time_us, int64_t to_time_us);
Position deltaOdoRelative(int64_t from_time_us, int64_t to_time_us);
Position getRelativePosition(Odometer first, Odometer second);

}  // namespace htwk