#include "policy_executor.h"

#include <cstring>

PolicyExecutor::PolicyExecutor(const std::string& model_name, std::vector<int> input_dims) {
    tflite.loadModelFromFile(tflite.getTFliteModelPath() + model_name, input_dims);

    input_tensor = tflite.getInputTensor();
    int input_size = 1;
    for (int dim : input_dims) {
        input_size *= dim;
    }
    memset(input_tensor, 0, input_size * sizeof(float));
}

PolicyExecutor::~PolicyExecutor() {}

std::optional<std::vector<float>> PolicyExecutor::execute(const std::vector<float>& input) {
    if (time_us() - last_execution_time < plicy_interval * 1e6) {
        return std::nullopt;
    }

    last_execution_time = time_us();

    memcpy(input_tensor, input.data(), input.size() * sizeof(float));
    tflite.execute();

    const float* output_tensor = tflite.getOutputTensor();

    return std::make_optional(
            std::vector<float>(output_tensor, output_tensor + T1Config::Policy::num_actions));
}

std::array<float, 3> PolicyExecutor::get_projected_gravity() {
    std::shared_ptr<IMUJointState> imu_joint_state = imu_joint_states_subscriber.latest();
    if (imu_joint_state == nullptr) {
        return {0.0f, 0.0f, 0.0f};
    }

    point_3d gravity = {0.0f, 0.0f, -1.0f};
    gravity = gravity.rotated_inverse_rpy(imu_joint_state->imu.rpy[0], imu_joint_state->imu.rpy[1],
                                          imu_joint_state->imu.rpy[2]);

    return {gravity.x, gravity.y, gravity.z};
}

std::array<float, 3> PolicyExecutor::get_base_ang_vel() {
    std::shared_ptr<IMUJointState> imu_joint_state = imu_joint_states_subscriber.latest();
    if (imu_joint_state == nullptr) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {imu_joint_state->imu.gyro[0], imu_joint_state->imu.gyro[1],
            imu_joint_state->imu.gyro[2]};
}

std::array<float, T1Config::Policy::Control::num_dofs> PolicyExecutor::get_dof_pos() {
    std::shared_ptr<IMUJointState> imu_joint_state = imu_joint_states_subscriber.latest();
    if (imu_joint_state == nullptr) {
        return std::array<float, T1Config::Policy::Control::num_dofs>{};
    }

    std::array<float, T1Config::Policy::Control::num_dofs> dof_pos;
    for (int i = 0; i < T1Config::Policy::Control::num_dofs; i++) {
        dof_pos[i] = (imu_joint_state->serial[i].q - T1Config::Common::default_qpos[i]) *
                     T1Config::Policy::Normalization::dof_pos;
    }

    return dof_pos;
}

std::array<float, T1Config::Policy::Control::num_dofs> PolicyExecutor::get_dof_vel() {
    std::shared_ptr<IMUJointState> imu_joint_state = imu_joint_states_subscriber.latest();
    if (imu_joint_state == nullptr) {
        return std::array<float, T1Config::Policy::Control::num_dofs>{};
    }

    std::array<float, T1Config::Policy::Control::num_dofs> dof_vel;
    for (int i = 0; i < T1Config::Policy::Control::num_dofs; i++) {
        dof_vel[i] = imu_joint_state->serial[i].dq * T1Config::Policy::Normalization::dof_vel;
    }

    return dof_vel;
}