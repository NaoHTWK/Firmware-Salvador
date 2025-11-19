#pragma once

#include <unordered_map>

#include "point_2d.h"
#include "soccerfield.h"

inline point_2d dribblingDirection(const point_2d& pos) {
    static float h_length = SoccerField::length() / 2.f;
    static float h_width = SoccerField::width() / 2.f;
    static float q_length = SoccerField::length() / 5.f;
    static float q_width = SoccerField::width() / 5.f;
    static std::unordered_map<point_2d /*field_coord*/, point_2d /*weighted_dir*/> vertices{
            {{-h_length, -h_width}, {2.f, .5f}},  {{-h_length, -q_width}, {2.f, 0.f}},
            {{-h_length, 0.f}, {2.f, 0.f}},       {{-h_length, q_width}, {2.f, 0.f}},
            {{-h_length, h_width}, {2.f, -.5f}},

            {{0.f, -h_width}, {2.f, 0.5f}},       {{0.f, 0.f}, {1.f, 0.f}},
            {{0, h_width}, {2.f, -0.5f}},

            {{q_length, -h_width}, {1.f, 2.f}},   {{q_length, 0.f}, {5.f, 0.f}},
            {{q_length, h_width}, {1.f, -2.f}},

            {{h_length, -h_width}, {-2.5f, 1.f}}, {{h_length, -q_width}, {-1.f, 1.5f}},
            {{h_length, 0.f}, {3.f, 0.f}},        {{h_length, q_width}, {-1.f, -1.5f}},
            {{h_length, h_width}, {-2.5f, -1.f}}};
    point_2d sum(0, 0);
    for (const auto& [v_pos, dir] : vertices) {
        sum += dir / (.1f + (pos - v_pos).norm_sqr());
    }
    return sum.normalized();
}