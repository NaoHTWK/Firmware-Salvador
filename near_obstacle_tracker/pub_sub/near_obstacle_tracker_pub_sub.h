#pragma once

#include "channel.h"
#include "near_obstacle_tracker_result.h"

extern htwk::Channel<std::shared_ptr<htwk::NearObstacleTrackerResult>>
        near_obstacles_tracker_result_channel;
