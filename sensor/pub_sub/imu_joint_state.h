#pragma once

#include "imu.h"
#include "point_3d.h"

struct IMU_new {
    float gyro[3];
    float rpy[3];
    float acc[3];
    // point_3d accel;

    IMU toOld() {
        IMU old;
        old.gyr = {rpy[2], rpy[1], rpy[0]};
        old.accel = {acc[0], acc[1], acc[2]};
        return old;
    }
};
struct MotorState {
    float q;        // Joint angle position, unit: rad.
    float dq;       // Joint angular velocity, unit: rad/s.
    float ddq;      // Joint angular acceleration, unit: rad/s².
    float tau_est;  // Joint torque, unit: nm
};

enum class JointIndex {
    // head
    kHeadYaw = 0,
    kHeadPitch = 1,

    // Left arm
    kLeftShoulderPitch = 2,
    kLeftShoulderRoll = 3,
    kLeftElbowPitch = 4,
    kLeftElbowYaw = 5,

    // Right arm
    kRightShoulderPitch = 6,
    kRightShoulderRoll = 7,
    kRightElbowPitch = 8,
    kRightElbowYaw = 9,

#ifdef ROBOT_MODEL_K1
    // left leg
    kLeftHipPitch = 10,
    kLeftHipRoll = 11,
    kLeftHipYaw = 12,
    kLeftKneePitch = 13,
    kCrankUpLeft = 14,
    kCrankDownLeft = 15,

    // right leg
    kRightHipPitch = 16,
    kRightHipRoll = 17,
    kRightHipYaw = 18,
    kRightKneePitch = 19,
    kCrankUpRight = 20,
    kCrankDownRight = 21,
#else
    // waist
    kWaist = 10,

    // left leg
    kLeftHipPitch = 11,
    kLeftHipRoll = 12,
    kLeftHipYaw = 13,
    kLeftKneePitch = 14,
    kCrankUpLeft = 15,
    kCrankDownLeft = 16,

    // right leg
    kRightHipPitch = 17,
    kRightHipRoll = 18,
    kRightHipYaw = 19,
    kRightKneePitch = 20,
    kCrankUpRight = 21,
    kCrankDownRight = 22,
#endif

};

struct IMUJointState {
    IMU_new imu;
#ifdef ROBOT_MODEL_K1
    MotorState parallel[22];
    MotorState serial[22];
#else
    MotorState parallel[23];
    MotorState serial[23];
#endif
};
