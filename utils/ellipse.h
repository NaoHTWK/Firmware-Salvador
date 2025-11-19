#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "point_2d.h"

namespace htwk {

class Ellipse {
public:
    float a{}, b{}, c{}, d{}, e{}, f{};
    float a1{}, b1{}, c1{}, d1{}, e1{}, f1{};
    float ta{}, tb{};
    float brennpunkt{};
    float trans[2][2]{{}, {}};
    float translation[2]{};
    bool found{false};

    Ellipse() = default;

    Ellipse(float ellipse[6])
        : a(ellipse[0]),
          b(ellipse[1]),
          c(ellipse[2]),
          d(ellipse[3]),
          e(ellipse[4]),
          f(ellipse[5]) {}

    Ellipse(const std::vector<float> &p) : a(p[0]), b(p[1]), c(p[2]), d(p[3]), e(p[4]), f(p[5]) {}

    // Returns (x, y) on the ellipse for a given angle (in radians)
    point_2d getPoint(float angle) const {
        // Convert general form to center, axes, rotation
        // Matrix: | a  b/2 |
        //         | b/2 c  |
        float theta = 0.5f * std::atan2(b, a - c);  // rotation angle
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        // Calculate center
        float denom = b * b - 4 * a * c;
        float x0 = (2 * c * d - b * e) / denom;
        float y0 = (2 * a * e - b * d) / denom;

        // Calculate axes lengths
        float up = 2 * (a * e * e + c * d * d + f * b * b - 2 * b * d * e - a * c * f);
        float down1 = (b * b - 4 * a * c) * ((a + c) + std::sqrt((a - c) * (a - c) + b * b));
        float down2 = (b * b - 4 * a * c) * ((a + c) - std::sqrt((a - c) * (a - c) + b * b));
        float axis_a = std::sqrt(std::abs(up / down1));
        float axis_b = std::sqrt(std::abs(up / down2));

        // Parametric point on ellipse in canonical position
        float x = axis_a * std::cos(angle);
        float y = axis_b * std::sin(angle);

        // Rotate and translate back
        float xr = cos_t * x - sin_t * y + x0;
        float yr = sin_t * x + cos_t * y + y0;
        return point_2d(xr, yr);
    }
};

}  // namespace htwk
