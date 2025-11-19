#include <logging.h>
#include <walk_custom.h>

using namespace htwk;
using namespace std;

MotionCommand WalkCustom::proceed(MotionCommand::WalkRequest walk_request) {
    float dx = walk_request.dx;
    float dy = walk_request.dy;
    float da = walk_request.da;

    point_3d direction = point_3d(dx, dy, da);

    if (direction.norm() < 1.0e-5) {
        gait_frequency = 0.f;
    } else {
        gait_frequency = 1.5f;
    }

    float gait_process = fmod(time_us() / 1e6 * gait_frequency, 1.0f);

    std::vector<float> input;

    const auto gravity = policy_executor.get_projected_gravity();
    const auto ang_vel = policy_executor.get_base_ang_vel();

    input.insert(input.end(), gravity.begin(), gravity.end());
    input.insert(input.end(), ang_vel.begin(), ang_vel.end());

    input.push_back(direction.x * (gait_frequency > 1.0e-8 ? 1.f : 0.f));
    input.push_back(direction.y * (gait_frequency > 1.0e-8 ? 1.f : 0.f));
    input.push_back(direction.z * (gait_frequency > 1.0e-8 ? 1.f : 0.f));

    input.push_back(gait_frequency);  // gait_frequency [1.2, 2.2]
    input.push_back(0.f);             // foot_yaw_L [-0.7,  0.7]
    input.push_back(0.f);             // foot_yaw_R [-0.1,  0.3]
    input.push_back(-0.15f);          // body_pitch_target [-0.1,  0.3]
    input.push_back(0.f);             // body_roll_target [-0.1,  0.1]
    input.push_back(0.f);             // feet_offset_x_target [-0.25, 0.25]
    input.push_back(0.08f);           // feet_offset_y_target [-0.2, 0.2]

    input.push_back(cos(gait_process * 2 * M_PI) * (gait_frequency > 1.0e-8 ? 1.f : 0.f));
    input.push_back(sin(gait_process * 2 * M_PI) * (gait_frequency > 1.0e-8 ? 1.f : 0.f));

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

        last_joint_control = joint_control;

        return MotionCommand::JointControl(joint_control, HeadFocus::LOC);
    }

    return MotionCommand::JointControl(last_joint_control, HeadFocus::LOC);
}
