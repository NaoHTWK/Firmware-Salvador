#pragma once

#include <optional>

#include "channel.h"
#include "rel_ball.h"
#include "robot_detection.h"

extern htwk::Channel<std::optional<RelBall>> rel_ball_channel;
extern htwk::Channel<std::vector<RobotDetection>> rel_robots_channel;