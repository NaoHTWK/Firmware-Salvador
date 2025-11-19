#include "tracked_object.h"

#include <algorithm_ext.h>

namespace htwk {

using namespace std;

void TrackedObject::addHistory(const point_2d &_posRel, const int64_t &_posTime) {
    double floatTime = _posTime / 1'000'000.;
    posHistory.push_front({_posRel, floatTime});
    erase_if(posHistory, [&](const PositionTime &elem) { return floatTime - elem.posTime > 2.; });
}

TrackedObject::TrackedObject(point_2d posRel, float pDetection, optional<float> ownTeamProb, int objImageRadius,
                             point_2d initialImageCoords)
    : posRel(posRel),
      pDetection(pDetection),
      ownTeamProb(ownTeamProb),
      objImageRadius(objImageRadius),
      imageCoords(initialImageCoords) {}

void TrackedObject::calcSpeed() {
    high_risk_velocity = calcHighRiskSpeedVector();
    if (posHistory.size() >= 4) {
        medium_risk_velocity = calcSpeedVector(4,5);
    }
    if (posHistory.size() >= 7) {
        point_2d speedVector = calcSpeedVector(7,5);
        if (speedVector.norm_sqr() > 0.06f * 0.06f) {
            velocity = speedVector;
            isMoving = true;
            return;
        }
    }
    velocity = {.0f, .0f};
    isMoving = false;
}

point_2d TrackedObject::calcSpeedVector(size_t min_num_consistent, size_t max_smoothing) {
    //size_t min_num_consistent = 4;
    //size_t max_smoothing = 5;
    for (size_t smoothing = 1; posHistory.size() >= (min_num_consistent + 1) * smoothing && smoothing <= max_smoothing;
         smoothing++) {
        vector<PositionTime> pos;
        auto it = posHistory.begin();
        while (pos.size() < min_num_consistent + 1) {
            point_2d p = it->posRel;
            double t = it->posTime;
            it++;
            for (size_t i = 1; i < smoothing; i++) {
                p += it->posRel;
                t += it->posTime;
                it++;
            }
            p /= smoothing;
            t /= smoothing;
            pos.push_back({.posRel = p, .posTime = t});
        }
        point_2d total_speed = (pos.back().posRel - pos.front().posRel) / (pos.back().posTime - pos.front().posTime);
        bool consistent = true;
        {
            auto it = pos.begin();
            auto it2 = pos.begin();
            it2++;
            while (it2 != pos.end()) {
                point_2d elem_speed = (it2->posRel - it->posRel) / (it2->posTime - it->posTime);
                if (!within(elem_speed.magnitude(), total_speed.magnitude() * .5f, total_speed.magnitude() * 2.f) ||
                    abs(angleDiff(elem_speed.to_direction(), total_speed.to_direction())) > 20_deg) {
                    consistent = false;
                    break;
                }
                it++;
                it2++;
            }
        }
        if (consistent) {
            return total_speed;
        }
    }
    return {0, 0};
}

point_2d TrackedObject::calcHighRiskSpeedVector() {
    if (posHistory.size() < 3)
        return {0, 0};
    vector<PositionTime> pos;
    for (auto it = posHistory.begin(); pos.size() < 3; it++)
        pos.push_back(*it);
    point_2d total_speed = (pos[2].posRel - pos[0].posRel) / (pos[2].posTime - pos[0].posTime);
    point_2d elem1_speed = (pos[1].posRel - pos[0].posRel) / (pos[1].posTime - pos[0].posTime);
    point_2d elem2_speed = (pos[2].posRel - pos[1].posRel) / (pos[2].posTime - pos[1].posTime);
    if (within(elem1_speed.magnitude(), total_speed.magnitude() * .6f, total_speed.magnitude() / .6f) &&
        abs(angleDiff(elem1_speed.to_direction(), total_speed.to_direction())) < 20_deg &&
        within(elem2_speed.magnitude(), total_speed.magnitude() * .6f, total_speed.magnitude() / .6f) &&
        abs(angleDiff(elem2_speed.to_direction(), total_speed.to_direction())) < 20_deg) {
        return total_speed;
    }
    return {0, 0};
}

}  // namespace htwk
