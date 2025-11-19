#pragma once

#include <array>

#include "head_focus.h"

class MotionCommand {
public:
    enum class Type { NOTHING, WALK, STAND, JOINT_CONTROL };

    struct WalkRequest {
        float dx = 0;  //!< Movement in [m/s]
        float dy = 0;  //!< Movement in [m/s]
        float da = 0;  //!< Rotation in [rad/s]
    };

    struct JointControlRequest {
        std::array<float, 23> joint_control;
    };

    static MotionCommand Nothing;
    static MotionCommand Walk(WalkRequest walk_request, HeadFocus focus = HeadFocus::NOTHING) {
        return {.type = Type::WALK, .walk_request = walk_request, .focus = focus};
    }

    static MotionCommand Stand(HeadFocus focus = HeadFocus::NOTHING) {
        return {.type = Type::STAND,
                .walk_request = {},
                .focus = focus,
                .joint_control_request = {}};
    }
    static MotionCommand JointControl(std::array<float, 23> joint_control,
                                      HeadFocus focus = HeadFocus::NOTHING) {
        return {.type = Type::JOINT_CONTROL,
                .walk_request = {},
                .focus = focus,
                .joint_control_request = {joint_control}};
    }

    Type type{Type::NOTHING};
    WalkRequest walk_request{};
    HeadFocus focus = HeadFocus::NOTHING;
    JointControlRequest joint_control_request{};

    friend bool operator==(const MotionCommand& lhs, const MotionCommand& rhs) {
        return lhs.type == rhs.type;
    }

    friend bool operator!=(const MotionCommand& lhs, const MotionCommand& rhs) {
        return !(lhs == rhs);
    }
};
