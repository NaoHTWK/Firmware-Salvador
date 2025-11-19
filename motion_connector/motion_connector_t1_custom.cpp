#include "motion_connector_t1_custom.h"

#include "gc_pub_sub.h"
#include "gc_state.h"
#include "logging.h"

MotionConnector::MotionConnector(PlayerIdx idx) : idx(idx) {
    client.Init();

    low_sdk_publisher.reset(
            new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(kTopicLowSDK));
    low_sdk_publisher->InitChannel();

    // init sdk cmd
    low_sdk_msg.cmd_type(booster_interface::msg::CmdType::SERIAL);

    for (size_t i = 0; i < booster::robot::b1::kJointCnt; i++) {
        booster_interface::msg::MotorCmd motor_cmd;
        low_sdk_msg.motor_cmd().push_back(motor_cmd);
    }
}

MotionConnector::~MotionConnector() {
    MotionCommand custom_command = custom_walk.proceed(MotionCommand::WalkRequest{0.f, 0.f, 0.f});
    send_low_level_motion(custom_command, YawPitch{0.f, 0.f});
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    LOG_F(INFO, "Changing mode to kPrepare (destructor)");
    client.ChangeMode(booster::robot::RobotMode::kPrepare);
}

void MotionConnector::send_low_level_motion(const MotionCommand& motion_command,
                                            const YawPitch& head_angles) {
    std::shared_ptr<IMUJointState> imu_joint_states = imu_joint_states_subscriber.latest();
    if (imu_joint_states == nullptr) {
        LOG_F(INFO, "No imu joint states");
        return;
    }

    if (time_us() - last_execution_time < plicy_interval * 1e6) {
        return;
    }
    last_execution_time = time_us();

    MotorState serial_motor_state[booster::robot::b1::kJointCnt];
    for (int i = 0; i < booster::robot::b1::kJointCnt; i++) {
        serial_motor_state[i] = imu_joint_states->serial[i];
    }

    std::array<float, 23> joint_control = motion_command.joint_control_request.joint_control;
    joint_control[0] = head_angles.yaw;
    joint_control[1] = head_angles.pitch;

    std::transform(filtered_dof_target.begin(), filtered_dof_target.end(), joint_control.begin(),
                   filtered_dof_target.begin(),
                   [](float filtered, float target) { return filtered * 0.8f + target * 0.2f; });

    for (int i = 0; i < booster::robot::b1::kJointCnt; i++) {
        low_sdk_msg.motor_cmd()[i].q(filtered_dof_target[i]);
        low_sdk_msg.motor_cmd()[i].dq(0.f);
        low_sdk_msg.motor_cmd()[i].kp(T1Config::Common::stiffness[i]);
        low_sdk_msg.motor_cmd()[i].kd(T1Config::Common::damping[i]);
        low_sdk_msg.motor_cmd()[i].tau(0.f);
    }

    for (int i : T1Config::Mech::parallel_mech_indexes) {
        float torque =
                (filtered_dof_target[i] - serial_motor_state[i].q) * T1Config::Common::stiffness[i];
        torque = std::clamp(torque, -T1Config::Common::torque_limit[i],
                            T1Config::Common::torque_limit[i]);

        low_sdk_msg.motor_cmd()[i].q(serial_motor_state[i].q);
        low_sdk_msg.motor_cmd()[i].tau(torque);
        low_sdk_msg.motor_cmd()[i].kp(0.f);
    }
    low_sdk_publisher->Write(&low_sdk_msg);
}

void MotionConnector::send_high_level_motion(const MotionCommand::WalkRequest& walk_request,
                                             const YawPitch& head_angles) {
    if (last_head_angles.pitch != head_angles.pitch || last_head_angles.yaw != head_angles.yaw) {
        client.RotateHead(head_angles.pitch, head_angles.yaw);
        last_head_angles = head_angles;
    }
    if (last_walk_request.dx == walk_request.dx && last_walk_request.dx == walk_request.dy &&
        last_walk_request.dx == walk_request.da) {
        same_motion_command_count += 1;
    } else {
        same_motion_command_count = 0;
    }

    if (same_motion_command_count < 10) {
        client.Move(walk_request.dx, walk_request.dy, walk_request.da);
    }
    last_walk_request = walk_request;
}

void MotionConnector::handle_fallen_robot() {
    std::shared_ptr<IMUJointState> imu_joint_state = imu_joint_states_subscriber.latest();
    if (abs(imu_joint_state->imu.gyro[0]) > falling_threshold_gyro_x ||
        abs(imu_joint_state->imu.gyro[1]) > falling_threshold_gyro_y) {
        if (current_mode != booster::robot::RobotMode::kDamping) {
            LOG_F(INFO, "Robot is still falling");
            LOG_F(INFO, "Changing mode to kDamping");
            client.ChangeMode(booster::robot::RobotMode::kDamping);
            current_mode = booster::robot::RobotMode::kDamping;
        }
        return;
    }
    LOG_F(INFO, "Robot is fallen");
    LOG_F(INFO, "Changing mode to kPrepare");
    client.ChangeMode(booster::robot::RobotMode::kPrepare);
    current_mode = booster::robot::RobotMode::kPrepare;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    client.GetUp();
    LOG_F(INFO, "Requested GetUp");
}

void MotionConnector::proceed() {
    htwk::FallDownState fall_down_state = fallen_subscriber.latest();
    if (fall_down_state.type == htwk::FallDownStateType::FALLEN) {
        handle_fallen_robot();
        last_fallen_state = fall_down_state.type;
        return;
    }
    if (fall_down_state.type != htwk::FallDownStateType::READY) {
        last_fallen_state = fall_down_state.type;
        return;
    }
    if (fall_down_state.type == htwk::FallDownStateType::READY &&
        last_fallen_state == htwk::FallDownStateType::GETTING_UP) {
        LOG_F(INFO, "change back to custom mode");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        LOG_F(INFO, "Changing mode to kPrepare");
        client.ChangeMode(booster::robot::RobotMode::kPrepare);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        LOG_F(INFO, "Changing mode to kCustom");
        client.ChangeMode(booster::robot::RobotMode::kCustom);
        current_mode = booster::robot::RobotMode::kCustom;
        last_fallen_state = fall_down_state.type;
        return;
    }

    MotionCommand motion_command = motion_command_subscriber.latest();
    YawPitch head_angles = head_control.proceed(motion_command);
    GCState gc_state = gc_state_sub.latest();
    if ((gc_state.state == GameState::Initial || gc_state.state == GameState::Finished ||
         gc_state.state == GameState::Standby || gc_state.state == GameState::Set ||
         gc_state.my_team.players[idx].is_penalized) &&
        !gc_state.is_fake_gc) {
        if (!standing) {
            standing = true;
            start_standing_time = time_us();
        }
        motion_command = MotionCommand::Stand();
        MotionCommand custom_command =
                custom_walk.proceed(MotionCommand::WalkRequest{0.f, 0.f, 0.f});
        send_low_level_motion(custom_command, head_angles);
        return;
    }
    standing = false;

    if (motion_command.type == MotionCommand::Type::JOINT_CONTROL) {
        if (current_mode != booster::robot::RobotMode::kCustom) {
            LOG_F(INFO, "Changing mode to kCustom (JOINT_CONTROL)");
            client.ChangeMode(booster::robot::RobotMode::kCustom);
            current_mode = booster::robot::RobotMode::kCustom;
        }
        send_low_level_motion(motion_command, head_angles);
    } else if (motion_command.type == MotionCommand::Type::WALK) {
        if (current_mode != booster::robot::RobotMode::kCustom) {
            LOG_F(INFO, "Changing mode to kCustom (WALK)");
            client.ChangeMode(booster::robot::RobotMode::kCustom);
            current_mode = booster::robot::RobotMode::kCustom;
        }
        MotionCommand custom_command = custom_walk.proceed(motion_command.walk_request);
        send_low_level_motion(custom_command, head_angles);
    } else if (motion_command.type == MotionCommand::Type::STAND ||
               motion_command.type == MotionCommand::Type::NOTHING) {
        if (current_mode != booster::robot::RobotMode::kCustom && !gc_state.is_fake_gc) {
            LOG_F(INFO, "Changing mode to kCustom (STAND/NOTHING)");
            client.ChangeMode(booster::robot::RobotMode::kCustom);
            current_mode = booster::robot::RobotMode::kCustom;
        }

        MotionCommand custom_command =
                custom_walk.proceed(MotionCommand::WalkRequest{0.f, 0.f, 0.f});
        send_low_level_motion(custom_command, head_angles);
    }
}
