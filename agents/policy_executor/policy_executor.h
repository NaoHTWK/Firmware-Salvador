#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "T1_config.h"
#include "robot_time.h"
#include "sensor_pub_sub.h"
#include "tfliteexecuter.h"

class PolicyExecutor {
public:
    PolicyExecutor(const std::string& model_name, std::vector<int> input_dims);
    ~PolicyExecutor();

    std::optional<std::vector<float>> execute(const std::vector<float>& input);
    std::array<float, 3> get_projected_gravity();
    std::array<float, 3> get_base_ang_vel();
    std::array<float, T1Config::Policy::Control::num_dofs> get_dof_pos();
    std::array<float, T1Config::Policy::Control::num_dofs> get_dof_vel();

private:
    htwk::TFLiteExecuter tflite;
    float* input_tensor = nullptr;

    float plicy_interval = T1Config::Common::dt * T1Config::Policy::Control::decimation;
    uint64_t last_execution_time = time_us();

    htwk::ChannelSubscriber<std::shared_ptr<IMUJointState>> imu_joint_states_subscriber =
            htwk::imu_joint_states_channel.create_subscriber();
};