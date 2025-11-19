#pragma once

#include <T1_config.h>
#include <agent_base.h>
#include <policy_executor.h>
#include <walkcustomorder.h>

#include <vector>

class WalkCustomAgent : public AgentBase {
public:
    WalkCustomAgent()
        : AgentBase("WalkCustom"), last_actions(T1Config::Policy::Control::num_dofs) {}
    MotionCommand proceed(std::shared_ptr<Order> order) override;

private:
    float avoidance_side_decision = 0;
    int64_t avoidance_side_decision_reset_time = time_us();

    float gait_frequency = 0;

    PolicyExecutor policy_executor = PolicyExecutor("/policies/thomas_walk_2.tflite", {54});

    std::vector<float> last_actions;
    std::array<float, T1Config::Policy::Control::num_dofs> last_joint_control =
            T1Config::Common::default_qpos;
};
