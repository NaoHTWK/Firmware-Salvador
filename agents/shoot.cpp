#include "shoot.h"

#include "shootorder.h"

using namespace htwk;
using namespace std;

MotionCommand ShootAgent::proceed(std::shared_ptr<Order> order) {
    if (!isOrder<ShootOrder>(order))
        return MotionCommand::Nothing;

    // TODO: Implement different shoot strengths

    auto* shoot_order = dynamic_cast<ShootOrder*>(order.get());

    std::optional<RelBall> ball = rel_ball_channel_sub.latest();

    if (!ball) {
        return MotionCommand::Nothing;
        // TODO: Why did we return something from a shoot agent if it doesn't see the ball???
        // policy_executor.get_dof_pos();

        // last_joint_control[1] = .88f;

        // focus = htwk::HeadFocusInternal{.type =
        // htwk::HeadFocusInternal::FocusType::BALL_SEARCH_LEFT}; return
        // MotionCommand::JointControl(last_joint_control, focus);
    }

    point_2d ball_pos = ball->pos_rel;

    std::vector<float> input;

    const auto gravity = policy_executor.get_projected_gravity();
    const auto ang_vel = policy_executor.get_base_ang_vel();

    input.insert(input.end(), gravity.begin(), gravity.end());
    input.insert(input.end(), ang_vel.begin(), ang_vel.end());

    input.push_back(ball_pos.x);
    input.push_back(ball_pos.y);

    const auto dof_pos = policy_executor.get_dof_pos();
    const auto dof_vel = policy_executor.get_dof_vel();
    input.insert(input.end(), dof_pos.begin() + 11, dof_pos.end());
    input.insert(input.end(), dof_vel.begin() + 11, dof_vel.end());

    input.insert(input.end(), last_actions.begin(), last_actions.end());

    std::optional<std::vector<float>> actions = policy_executor.execute(input);

    if (actions) {
        std::transform(actions->begin(), actions->end(), actions->begin(),
                       [](float a) { return std::clamp(a, -1.0f, 1.0f); });

        last_actions = *actions;

        std::array<float, 23> joint_control = T1Config::Common::default_qpos;
        for (size_t i = 11; i < joint_control.size(); i++) {
            joint_control[i] += (*actions)[i - 11];
        }

        joint_control[1] = 0.88f;
        last_joint_control = joint_control;

        return MotionCommand::JointControl(joint_control, HeadFocus::BALL);
    }

    return MotionCommand::JointControl(last_joint_control, HeadFocus::BALL);
}
