#include "multi_target_tracker_pub_sub.h"

#include "robot_detection.h"

htwk::Channel<std::optional<RelBall>> rel_ball_channel;
htwk::Channel<std::vector<RobotDetection>> rel_robots_channel;