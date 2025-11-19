#pragma once

#include <array>

namespace T1Config {
// Common configuration
namespace Common {
constexpr float dt = 0.002;

constexpr std::array<float, 23> stiffness = {20, 20,  20,  20,  20,  20,  20,  20,
                                             20, 20,  200, 200, 200, 200, 200, 50,
                                             50, 200, 200, 200, 200, 50,  50};

constexpr std::array<float, 23> damping = {0.2, 0.2, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 5, 5,
                                           5,   5,   5,   3,   3,   5,   5,   5,   5,   3,   3};

constexpr std::array<float, 23> default_qpos = {0, 0,    0.2, -1.35, 0,   -0.5,  0.2, 1.35,
                                                0, 0.5,  0,   -0.2,  0,   0,     0.4, -0.25,
                                                0, -0.2, 0,   0,     0.4, -0.25, 0};

constexpr std::array<float, 23> torque_limit = {7,  7,  10, 10, 10, 10, 10, 10, 10, 10, 30, 60,
                                                25, 30, 60, 24, 15, 60, 25, 30, 60, 24, 15};
}  // namespace Common

// Policy configuration
namespace Policy {
constexpr int num_actions = 12;
constexpr float gait_frequency = 1.0;

namespace Normalization {
constexpr float gravity = 1.0;
constexpr float lin_vel = 1.0;
constexpr float ang_vel = 1.0;
constexpr float dof_pos = 1.0;
constexpr float dof_vel = 0.1;
constexpr float clip_actions = 1.0;
}  // namespace Normalization

namespace Control {
constexpr float action_scale = 1.0;
constexpr int decimation = 10;
constexpr int num_dofs = 23;
}  // namespace Control
}  // namespace Policy

// Mechanical configuration
namespace Mech {
constexpr std::array<int, 4> parallel_mech_indexes = {15, 16, 21, 22};
}

// Prepare configuration
namespace Prepare {
constexpr std::array<float, 23> stiffness = {5.0,   5.0,   40.0,  50.0,  20.0,  10.0,  40.0,  50.0,
                                             20.0,  10.0,  100.0, 350.0, 350.0, 180.0, 350.0, 450.0,
                                             450.0, 350.0, 350.0, 180.0, 350.0, 450.0, 450.0};

constexpr std::array<float, 23> damping = {0.1, 0.1, 0.5, 1.5, 0.2, 0.2, 0.5, 1.5,
                                           0.2, 0.2, 5.0, 7.5, 7.5, 3.0, 5.5, 0.5,
                                           0.5, 7.5, 7.5, 3.0, 5.5, 0.5, 0.5};

constexpr std::array<float, 23> default_qpos = {0.0, 0.0,  0.25, -1.4, 0.0, -0.5, 0.25, 1.4,
                                                0.0, 0.5,  0.0,  -0.1, 0.0, 0.0,  0.2,  -0.1,
                                                0.0, -0.1, 0.0,  0.0,  0.2, -0.1, 0.0};
}  // namespace Prepare
}  // namespace T1Config
