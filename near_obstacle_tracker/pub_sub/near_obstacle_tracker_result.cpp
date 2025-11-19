#include "near_obstacle_tracker_result.h"
#include "logging.h"

namespace htwk {

NearObstacleTrackerResult::NearObstacleTrackerResult(Raster<float> obstacle_probs, Position pos,
                                                     float resolution) {
    this->obstacle_probs = std::move(obstacle_probs);
    this->pos = pos;
    this->resolution = resolution;
}

bool NearObstacleTrackerResult::corridorBlocked(float depth, float width) const {
    std::vector<point_2d> sides{point_2d(depth, width / 2.f), point_2d(depth, -width / 2.f),
                                point_2d(0, -width / 4.f), point_2d(0, width / 4.f)};
    for (auto& p : sides) {
        // The "absolute" position in this case is the robot's position inside the raster.
        p = LocalizationUtils::relToAbs(p, pos);
    }
    Line lines[] = {Line(sides[0], sides[1]), Line(sides[1], sides[2]), Line(sides[2], sides[3]),
                    Line(sides[3], sides[0])};
    float min_blocked_ratio=0.01f;
    float sum_blocked=0;
    float cnt_blocked=0;
    for (int oy = 0; oy < obstacle_probs.height; oy++) {
        for (int ox = 0; ox < obstacle_probs.width; ox++) {
            bool inside = true;
            for (int i = 0; i < 4; i++) {
                if (lines[i].side({(ox - (int)obstacle_probs.width / 2 + 0.5f) * resolution,
                                   (oy - (int)obstacle_probs.height / 2 + 0.5f) * resolution}) >
                    0) {
                    inside = false;
                    break;
                }
            }
            if (!inside)
                continue;
            cnt_blocked++;

            if (obstacle_probs(ox, oy) > 0.5f)
                sum_blocked++;
        }
    }
    //printf("%02f",sum_blocked/cnt_blocked);
    //LOG_F(INFO, "OBSTACLE: %02f",sum_blocked/cnt_blocked);
    return sum_blocked>cnt_blocked*min_blocked_ratio;
}

}  // namespace htwk
