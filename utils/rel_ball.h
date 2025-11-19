#pragma once

#include "point_2d.h"

struct RelBall {
    point_2d pos_rel;
    int64_t ball_age_us = 0;
    int64_t last_seen_time = 0;
    point_2d velocity{0.f, 0.f};
    point_2d high_risk_velocity{0.f, 0.f};
    point_2d medium_risk_velocity{0.f, 0.f};
    bool is_moving = false;
};