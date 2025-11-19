#include "sensor_pub_sub.h"

#include <loguru.hpp>

namespace htwk {

Channel<std::shared_ptr<IMUJointState>> imu_joint_states_channel;
Channel<std::shared_ptr<Odometer>> odometer_channel;
MapStore<int64_t, Odometer> odometer_history;
Channel<FallDownState> fallen_channel;

Position getRelativePosition(Odometer first, Odometer second) {
    Position position;
    float x = second.x - first.x;
    float y = second.y - first.y;

    float theta = first.theta;
    float x_ = x * cos(theta) + y * sin(theta);
    float y_ = -x * sin(theta) + y * cos(theta);

    float theta_ = normalizeRotation(second.theta - first.theta);

    return {x_, y_, theta_};
}

Position deltaOdoRelative(int64_t from_time_us, int64_t to_time_us) {
    return deltaOdoRelativeFrame(from_time_us, to_time_us);
}

Position deltaOdoRelativeWorld(int64_t from_time_us, int64_t to_time_us) {
    std::map<int64_t, Odometer> all_odometer = odometer_history.get_all();
    auto first_it = all_odometer.lower_bound(from_time_us);
    auto second_it = all_odometer.lower_bound(to_time_us);

    if (first_it == all_odometer.end() || second_it == all_odometer.end()) {
        LOG_S(ERROR) << "Could not find odometer entries for timestamps: from " << from_time_us
                     << " to " << to_time_us << "size of all_odometer: " << all_odometer.size();

        if (!all_odometer.empty()) {
            LOG_S(ERROR) << "Lowest key in all_odometer: " << all_odometer.begin()->first
                         << ", highest key in all_odometer: " << all_odometer.rbegin()->first;
        }

        return Position{0, 0, 0};
    }

    Position rel_pos = getRelativePosition(first_it->second, second_it->second);
    Position odo;
    odo.add_relative(rel_pos);

    return odo;
}

Position deltaOdoRelativeFrame(int64_t from_time_us, int64_t to_time_us) {
    std::map<int64_t, Odometer> all_odometer = odometer_history.get_all();
    auto it_first = all_odometer.lower_bound(from_time_us);
    auto it_last = all_odometer.lower_bound(to_time_us);
    Position odo;
    if (all_odometer.empty()) {
        LOG_S(ERROR) << "Odometerstore is empty.";
        return Position{0, 0, 0};
    }
    if (it_first == all_odometer.end()) {
        LOG_S(ERROR) << "Odometerstore to young. Could not find odometer entries for timestamps: from " << from_time_us
                     << " to " << to_time_us << " size of all_odometer: " << all_odometer.size();

        LOG_S(ERROR) << "Lowest key in all_odometer: " << all_odometer.begin()->first
                         << ", highest key in all_odometer: " << all_odometer.rbegin()->first;

        return Position{0, 0, 0};
    }
    
    if (!all_odometer.empty() && from_time_us < all_odometer.begin()->first) {
        LOG_S(ERROR) << "Requested from_time_us " << from_time_us 
                     << " is older than oldest available odometer entry at " << all_odometer.begin()->first
                     << " (entries are too young)";

    }

    for (auto it = it_first; it != it_last; it++) {
        odo.add_relative(Position{it->second.x, it->second.y, it->second.theta});   
    }
    return odo;
}
}  // namespace htwk
