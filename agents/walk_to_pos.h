#pragma once

#include "agent_base.h"
#include "gc_pub_sub.h"
#include "localization_pub_sub.h"
#include "multi_target_tracker_pub_sub.h"
#include "near_obstacle_tracker_pub_sub.h"
#include "near_obstacle_tracker_result.h"
#include "rel_ball.h"
#include "robot_time.h"
#include "team_strategy_pub_sub.h"
#include "walktopositionorder.h"

class WalkToPositionAgent : public AgentBase {
public:
    WalkToPositionAgent() : AgentBase("WalkToPosition") {}
    MotionCommand proceed(std::shared_ptr<Order> order) override;

private:
    htwk::Position handelHeadAngle(htwk::Position order_pos, WalkToPositionOrder::Mode order_mode,
                                   htwk::Position current_pos);

    HeadFocus focus;

    htwk::ChannelSubscriber<LocPosition> pos_sub = loc_position_channel.create_subscriber();
    htwk::ChannelSubscriber<std::optional<RelBall>> rel_ball_sub =
            rel_ball_channel.create_subscriber();
    htwk::ChannelSubscriber<GCState> gc_state_sub = gc_state.create_subscriber();
    htwk::ChannelSubscriber<std::optional<TeamComData>> striker_sub =
            striker_channel.create_subscriber();
    htwk::ChannelSubscriber<std::vector<RobotDetection>> rel_robots_sub =
            rel_robots_channel.create_subscriber();
    htwk::ChannelSubscriber<std::shared_ptr<htwk::NearObstacleTrackerResult>>
            near_obstacle_tracker_result_sub =
                    near_obstacles_tracker_result_channel.create_subscriber();

    float avoidance_side_decision = 0;
    int64_t avoidance_side_decision_reset_time = time_us();
};
