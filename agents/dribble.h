#pragma once

#include "agent_base.h"
#include "localization_pub_sub.h"
#include "multi_target_tracker_pub_sub.h"
#include "near_obstacle_tracker_pub_sub.h"
#include "near_obstacle_tracker_result.h"
#include "parameter_tuner/parameter_tuner.h"
#include "point_2d.h"
#include "walk_custom.h"

class DribbleAgent : public AgentBase {
public:
    DribbleAgent() : AgentBase("Dribble") {
        // near_obstacle_tracker_result_sub is already initialized during declaration
    }
    MotionCommand proceed(std::shared_ptr<Order> order) override;

private:
    point_2d getSpreadTargetVector(point_2d target, float maxAngle);
    float getSpreadFactor(point_2d robot_pos, float weight);

    htwk::ChannelSubscriber<std::optional<RelBall>> rel_ball_sub =
            rel_ball_channel.create_subscriber();

    htwk::ChannelSubscriber<LocPosition> loc_position_sub =
            loc_position_channel.create_subscriber();

    WalkCustomAgent walk_custom;

    htwk::ChannelSubscriber<std::shared_ptr<htwk::NearObstacleTrackerResult>>
            near_obstacle_tracker_result_sub =
                    near_obstacles_tracker_result_channel.create_subscriber();

    ParameterTuner param_tuner;
};