#pragma once

#include <vector>

#include "T1_config.h"
#include "agent_base.h"
#include "multi_target_tracker_pub_sub.h"
#include "policy_executor.h"

class ShootAgent : public AgentBase {
public:
    ShootAgent() : AgentBase("Shoot"), last_actions(T1Config::Policy::Control::num_dofs) {}
    MotionCommand proceed(std::shared_ptr<Order> order) override;

private:
    htwk::ChannelSubscriber<std::optional<RelBall>> rel_ball_channel_sub =
            rel_ball_channel.create_subscriber();

    float gait_frequency = 0;

    PolicyExecutor policy_executor = PolicyExecutor("/policies/shoot_v1.tflite", {44});

    std::vector<float> last_actions;
    std::array<float, T1Config::Policy::Control::num_dofs> last_joint_control =
            T1Config::Common::default_qpos;
};
