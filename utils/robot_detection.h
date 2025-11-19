#pragma once

#include "point_2d.h"

struct RobotDetection {
    point_2d pos_rel;
    float ownTeamProb = 0.5f;  // 0 = opponent, 1 = own team, 0.5 = unsure

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& pos_rel;
        ar& ownTeamProb;
    }
};
