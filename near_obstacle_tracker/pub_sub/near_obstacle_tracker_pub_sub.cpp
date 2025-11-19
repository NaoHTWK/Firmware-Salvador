#include "near_obstacle_tracker_pub_sub.h"

#include <optional>

htwk::Channel<std::shared_ptr<htwk::NearObstacleTrackerResult>>
        near_obstacles_tracker_result_channel;