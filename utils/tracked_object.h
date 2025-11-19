#ifndef TRACKED_OBJECT_H
#define TRACKED_OBJECT_H

#include <algorithm_ext.h>
#include <point_2d.h>

#include <list>
#include <optional>

namespace htwk {
class TrackedObject {
public:
    struct PositionTime {
        point_2d posRel;
        double posTime;
    };

    point_2d posRel;
    float pDetection;                  //!< Probability of detection
    float confidence = 0.f;            //!< Confidence that the object is there
    std::optional<float> ownTeamProb;  //!< 0 = definitely opponent, 0.5 = unsure, 1 = definitely us

    int64_t age_us = 600_s;  //!< Time in us where 1'000'000ll is one second
    int64_t lastTimeSeen = 0;

    int objImageRadius;
    point_2d imageCoords;

    point_2d velocity{0.f, 0.f};
    bool isMoving = false;
    point_2d high_risk_velocity{0.f, 0.f};
    point_2d medium_risk_velocity{0.f, 0.f};
    std::list<PositionTime> posHistory;

    TrackedObject() {}
    TrackedObject(point_2d posRel, float pDetection, std::optional<float> ownTeamProb, int objImageRadius = 0.f,
                  point_2d initialImageCoords = {0.f, 0.f});
    void addHistory(const point_2d &_posRel, const int64_t &_posTime);

    void calcSpeed();
    point_2d calcSpeedVector(size_t min_num_consistent, size_t max_smoothing);
    point_2d calcHighRiskSpeedVector();
};
}  // namespace htwk
#endif  // TRACKED_OBJECT_H
