#include "localization_utils.h"

point_2d LocalizationUtils::relToAbs(const point_2d& p, const htwk::Position& pos) {
    return pos.point() + p.rotated(pos.a);
}

point_2d LocalizationUtils::absToRel(const point_2d& p, const htwk::Position& pos) {
    return (p - pos.point()).rotated(-pos.a);
}
