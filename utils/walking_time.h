#pragma once

#include "point_3d.h"
#include "position.h"
#include "stl_ext.h"

inline float walkingTimeInternal(float dx, float dy, float da) {
    // TODO: adjust these values!
#ifdef ROBOT_MODEL_K1
    static constexpr float kMaxForward = 0.75f;    // m/s
    static constexpr float kMaxBackward = -0.75f;  // m/s
    static constexpr float kMaxStrafe = 0.5f;      // m/s
    static constexpr float kMaxTurn = 1.5f;        // rad/s
#else
    static constexpr float kMaxForward = 1.f;    // m/s
    static constexpr float kMaxBackward = -1.f;  // m/s
    static constexpr float kMaxStrafe = 1.f;     // m/s
    static constexpr float kMaxTurn = 1.f;       // rad/s
#endif
    point_3d p{dx / (dx >= 0.f ? kMaxForward : kMaxBackward), dy / kMaxStrafe, da / kMaxTurn};
    if (p.x == 0 && p.y == 0 && p.z == 0)
        return 0;
    float norm_factor_ellipse = 1.f / p.norm();
    return 1.f / norm_factor_ellipse;
}

inline float walkingTimeStraight(float dx) {
    return walkingTimeInternal(dx, 0, 0);
}

// Estimates time (in seconds) to walk using omnidirectional walking for short distances and
// turn-walk-turn for long distances.
inline float walkingTime(const htwk::Position& start, const htwk::Position& end,
                         bool dribble = false) {
    htwk::Position d = (end - htwk::Position(start.point(), 0)).rotated(-start.a);
    float time_omni = walkingTimeInternal(d.x, d.y, d.a);
    float time_turn = walkingTimeInternal(0, 0, d.point().to_direction()) +
                      walkingTimeInternal(d.norm(), 0, 0) +
                      walkingTimeInternal(0, 0, d.rotated(-d.point().to_direction()).a) *
                              (dribble ? 1.5f : 1.f);
    // TODO: This isn't very accurate.
    return clamped_linear_interpolation(d.norm(), time_omni, time_turn, 0.4f, 0.8f);
}
