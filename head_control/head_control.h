#pragma once

// #include <head_focus.h>
#include <cam_constants.h>
#include <imu.h>
#include <imu_joint_state.h>
#include <joints.h>
#include <localization_utils.h>
#include <point_2d.h>
#include <sensor_pub_sub.h>
#include <stdint.h>

#include <atomic>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <mutex>
#include <variant>

#include "image.h"
#include "localization_pub_sub.h"
#include "multi_target_tracker_pub_sub.h"
#include "object_hypothesis.h"
#include "tracked_object.h"

class MotionCommand;

namespace htwk {

class HeadControl;

// TODO: We can't have 2 separate classes called HeadFocus!
// TODO: Agents shouldn't have to depend on all of the dependencies of this class, just to get the
// struct.

struct HeadFocusInternal {

    enum FocusType {

        NOTHING,            // No arguments
        BALL,               // No arguments
        BALL_GOALIE,        // No arguments
        BALL_SEARCH_LEFT,   // No arguments
        BALL_SEARCH_RIGHT,  // No arguments
        LOCALIZE,           // No arguments
        OBSTACLES,          // No arguments
        FIXED_REL,          // Fixed position relative coordinates, gets a point_2d as argument

    } type;

    /// This allows us to give some more information to the HeadControl when setting the focus.

    using argument_type_t = std::optional<std::variant<point_2d>>;

    argument_type_t argument;

    /*
    HeadFocusInternal();
    HeadFocusInternal(const FocusType & type);

    template <typename T> HeadFocusInternal(const FocusType & type, const T & v) : type(type),
    argument(v) {

    }
    */
    /// Look at nothing. Head moves to centered position
    static HeadFocusInternal nothing();

    /// Look at the ball.
    static HeadFocusInternal ball();

    /// Search for the ball starting towards the left of center.
    static HeadFocusInternal ball_search_left();

    /// Search for the ball starting towards the right of center.
    static HeadFocusInternal ball_search_right();

    /// Localize on field
    static HeadFocusInternal localize();

    /** Look at a fixed relative position

        @param relative_position The position to look at given in relative coordinates
     */
    static HeadFocusInternal fixed_relative(const point_2d& relative_postion);
};

#if defined(ROBOT_MODEL_T1)
/// Minimum head yaw according to booster converted to rad
constexpr const float YAW_MIN = (-58 * M_PI) / 180.0f;
/// Maximum head yaw according to booster converted to rad
constexpr const float YAW_MAX = (58 * M_PI) / 180.0f;

/// Minimum head pitch according to booster converted to rad
constexpr const float PITCH_MIN = (-18 * M_PI) / 180.0f;
/// Maximum head pitch according to booster converted to rad
constexpr const float PITCH_MAX = (47 * M_PI) / 180.0f;
#elif defined(ROBOT_MODEL_K1)
/// Minimum head yaw according to booster converted to rad
constexpr const float YAW_MIN = (-58 * M_PI) / 180.0f;
/// Maximum head yaw according to booster converted to rad
constexpr const float YAW_MAX = (58 * M_PI) / 180.0f;

/// Minimum head pitch according to booster converted to rad
constexpr const float PITCH_MIN = (-18 * M_PI) / 180.0f;
/// Maximum head pitch according to booster converted to rad
constexpr const float PITCH_MAX = (48 * M_PI) / 180.0f;
#endif

/// Frequency with which control signals are sent to the motors in Hz.
constexpr const float CONTROL_FREQUENCY = 500.0;
/// Time interval between to control signals in seconds
constexpr const float UPDATE_INTERVAL = 1 / CONTROL_FREQUENCY;

/// Maximum angular velocity allowed for the head in rad/s
constexpr const float max_ang_vel = 30.0 * 0.2;

/// Maximum angular acceleration allowed for the head in rad/s²
constexpr const float max_ang_accel = 1000.0 * 0.2;

class HeadControl {
public:
    struct YawPitch {
        /// Yaw angle in rad
        float yaw;
        /// Pitch angle in rad
        float pitch;
    };

    HeadControl();

    YawPitch proceed(const MotionCommand& command);

    void set_focus(const HeadFocusInternal& focus);

private:
    /// Proportional factor for PID controller
    static constexpr const float kp = 0.4 * 0.5;
    /// Integral factor for PID controller
    static constexpr const float ki = 0.035 * 0.5;
    /// Derivative factor for PID controller
    static constexpr const float kd = 0.05 * 0.5;

    struct sta_settings {
        float w_fac = 0;
        float t = 0;
        float gain = 0;
        sta_settings(){};
        constexpr sta_settings(float w_fac, float t, float gain) : w_fac(w_fac), t(t), gain(gain) {}
    };

    /// The current head focus mode
    HeadFocusInternal focus;

    /// Wether or not the robot may move
    bool may_move;

    /// The time at which the current search pattern started.
    std::chrono::high_resolution_clock::time_point pattern_start_time;

    /// Takes the role of the time_step variable from Nao version.
    /// Initial offset into pattern when started
    float pattern_time_offset = 0.0f;

    /// The time we have last seen a ball
    std::chrono::high_resolution_clock::time_point last_ball_percept;

    /// The target angles the head should aim for.
    YawPitch target{};

    /// The current angles of the head.
    YawPitch current{};

    /// The previous angles of the head (for velocity calculation)
    YawPitch previous{};

    /// The current angular velocities of the head in rad/s
    YawPitch current_velocity{};

    /// The previous angular velocities of the head in rad/s (for acceleration calculation)
    YawPitch previous_velocity{};

    /// Integral error for PID controller of the head angles
    YawPitch integral{};
    /// Last measured error for PID controller. Used when calculating differential part
    YawPitch last_error{};

    /// Contains information about the ball.
    htwk::ChannelSubscriber<std::optional<RelBall>> ball_subscriber;
    /// Contains information about the head pose. Namely the height above the ground.
    htwk::ChannelSubscriber<std::shared_ptr<Image>> image_subscriber;

    /// Implements functionality to look at the ball if a ball is present
    YawPitch look_at_ball();

    /// Implements functionality to look at the ball if a ball is present
    YawPitch look_at_ball_relative(const point_2d& relative_position);

    /// Implements the functionality for searching a ball.
    YawPitch look_search_for_ball_left();
    YawPitch look_search_for_ball_right();

    /// Implements a search pattern
    YawPitch look_search_pattern();

    /// Looks out for obstacles
    YawPitch look_obstacles();

    /** Computes the angles for looking at a relative position.

        This is used by other functions, but can also be used directly with the fixed_relative()
        head focus.

        @param relative_position The position to look at in relative coordinates.
     */
    YawPitch look_at_position_relative(const point_2d& relative_position);

    /// Search pattern for localization
    YawPitch look_search_localize();

    /// Smooth triangle function
    static float smoothTriAng(float w, sta_settings s);
    static float smoothTriAngStartTime(YawPitch head_pos, sta_settings sta_yaw,
                                       sta_settings sta_pitch = sta_settings());

    /// Apply acceleration limits to head movement
    YawPitch apply_acceleration_limits(const YawPitch& target_angles);

    const sta_settings BALL_SEARCH_LEFT_YAW_STA{4.0f, 0.1f, 0.25f};
    const sta_settings BALL_SEARCH_LEFT_PITCH_STA{4.0f, 0.1f, 0.25f};

    const sta_settings BALL_SEARCH_RIGHT_YAW_STA{-4.0f, 0.1f, 0.25f};
    const sta_settings BALL_SEARCH_RIGHT_PITCH_STA{4.0f, 0.1f, 0.25f};

    const sta_settings OBSTACLES_YAW_STA{2.0f, 0.1f, 0.5f};

    const sta_settings LOCALIZE_YAW_STA{1.0f, 0.1f, YAW_MAX};
};

extern htwk::Channel<std::shared_ptr<HeadControl::YawPitch>> head_angle_channel;

}  // namespace htwk
