
#pragma once

#include <memory>
#include <utility>

#include "line.h"
#include "localization_utils.h"
#include "point_2d.h"
#include "position.h"
#include "raster.h"

namespace htwk {

class NearObstacleTrackerResult {
public:
    NearObstacleTrackerResult(Raster<float> obstacle_probs, Position pos, float resolution);

    Raster<float> obstacle_probs = Raster<float>(0, 0);
    Position pos;
    float resolution;

    bool corridorBlocked(float depth, float width) const;
};

}  // namespace htwk
