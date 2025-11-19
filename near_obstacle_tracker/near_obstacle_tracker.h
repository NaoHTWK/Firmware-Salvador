#pragma once

#include <point_2d.h>
#include <raster.h>

#include <limits>
#include <optional>
#include <vector>

#include "channel.h"
#include "position.h"
#include "robot_time.h"
#include "stl_ext.h"


namespace htwk {
class NearObstacleTracker {
public:
    NearObstacleTracker();

    void proceed(int64_t img_time);
    void proceed_dummy(int64_t image_time);

    // Returns true if there are any obstacles in a trapezoid of the given depth and width in front
    // of the robot.
    bool corridorBlocked(float depth, float width) const;

    // Returns the average occupancy in a trapezoid of the given depth, width, and angle in front of
    // the robot.
    // float corridorOccupancy(float depth, float width, float angle) const;

private:
    static constexpr float reshape_dist = 0.1f;
#ifdef ROBOT_MODEL_K1
    static constexpr float resolution = 0.1f;
    static constexpr float radius = 2.f;
#elif ROBOT_MODEL_T1
    static constexpr float resolution = 0.15f;
    static constexpr float radius = 3.f;
#endif
    static constexpr float dec_per_usec = 1.f / 8'000'000.f;
    static constexpr int size = round_int(radius / resolution * 2.f);

    void prepareUpdate(int64_t image_time, bool ready_for_action);
    void reshape();
    void visualize();

    htwk::ChannelSubscriber<std::shared_ptr<std::vector<point_2d>>> near_obstacles_subscriber;
    htwk::ChannelSubscriber<std::shared_ptr<std::vector<point_2d>>> no_near_obstacles_subscriber;
    htwk::ChannelSubscriber<std::shared_ptr<std::vector<point_2d>>> ball_near_obstacles_subscriber;

    Raster<float> obstacle_probs{size, size, 0};
    void clear();
    Position pos{0, 0, 0};
    int64_t last_image_time = time_us();
};
}  // namespace htwk