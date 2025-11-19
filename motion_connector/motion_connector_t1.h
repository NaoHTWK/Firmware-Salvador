#pragma once

#include <T1_config.h>
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/MotorCmd.h>
#include <gc_pub_sub.h>
#include <head_control.h>
#include <motion_pub_sub.h>
#include <robot_time.h>
#include <sensor_pub_sub.h>

#include <array>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_publisher.hpp>

static const std::string kTopicLowSDK = booster::robot::b1::kTopicJointCtrl;

class MotionConnector {
public:
    MotionConnector(PlayerIdx idx);
    ~MotionConnector();

    void proceed();

private:
    htwk::ChannelSubscriber<MotionCommand> motion_command_subscriber =
            motion_command_channel.create_subscriber();
    htwk::ChannelSubscriber<std::shared_ptr<IMUJointState>> imu_joint_states_subscriber =
            htwk::imu_joint_states_channel.create_subscriber();
    htwk::ChannelSubscriber<GCState> gc_state_sub = gc_state.create_subscriber();

    htwk::ChannelSubscriber<htwk::FallDownState> fallen_subscriber =
            htwk::fallen_channel.create_subscriber();

    booster::robot::b1::B1LocoClient client;

    booster::robot::ChannelPublisherPtr<booster_interface::msg::LowCmd> low_sdk_publisher;
    booster_interface::msg::LowCmd low_sdk_msg;

    float falling_threshold_gyro_x = 0.1;
    float falling_threshold_gyro_y = 0.1;

    std::array<float, 23> filtered_dof_target = T1Config::Common::default_qpos;
    std::array<float, 23> dof_target = T1Config::Common::default_qpos;

    int same_motion_command_count = 0;
    int64_t start_standing_time = 0;
    bool standing = false;
    float plicy_interval = T1Config::Common::dt;
    int64_t last_execution_time = time_us();

    PlayerIdx idx;

    htwk::HeadControl head_control;
    booster::robot::RobotMode current_mode = booster::robot::RobotMode::kPrepare;
    MotionCommand::WalkRequest last_walk_request;
    YawPitch last_head_angles;

    void send_low_level_motion(const MotionCommand& motion_command, const YawPitch& head_angles);
    void send_high_level_motion(const MotionCommand::WalkRequest& walk_request,
                                const YawPitch& head_angles);
    void handle_fallen_robot();
};
