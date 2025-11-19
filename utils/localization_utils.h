#pragma once

#include "point_2d.h"
#include "position.h"

class LocalizationUtils {
public:
    static point_2d relToAbs(const point_2d& p, const htwk::Position& pos);
    static point_2d absToRel(const point_2d& p, const htwk::Position& pos);
};
