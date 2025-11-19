#include "multi_target_tracker.h"

#include <utility>

#include "algorithm_ext.h"
#include "cam_constants.h"
#include "localization_utils.h"
#include "sensor_pub_sub.h"
#include "soccerfield.h"

#include <iostream>

using namespace std;

namespace htwk {
namespace {

float dist(const TrackedObject& a, const TrackedObject& b, float cam_translation_z) {
    if (a.posRel.norm() < 1.f && b.posRel.norm() < 1.f)
        return a.posRel.dist(b.posRel) +
               (a.ownTeamProb ? 0.05f * abs(*a.ownTeamProb - *b.ownTeamProb) : 0);
    float angle_to_dist_correction = 2.5f;
    return a.posRel.angular_dist(b.posRel, cam_translation_z) * angle_to_dist_correction +
           (a.ownTeamProb ? 0.05f * abs(*a.ownTeamProb - *b.ownTeamProb) : 0);
}

}  // namespace

MTtracker::MTtracker(size_t maxObjects, float smoothing, float minInitialP,
                     std::function<void(const std::vector<TrackedObject>&)> publish_callback,
                     float incConf, float duration)
    : maxObjects(maxObjects),
      smoothing(smoothing),
      increaseConfidence(incConf),
      durationInSeconds(duration),
      decreaseConfidence((1 - checkConfidence) / (imagesPerSecond * durationInSeconds)),
      minInitialP(minInitialP),
      publish_callback(std::move(publish_callback)) {}

void MTtracker::proceed(std::vector<TrackedObject> percepts, int64_t camTime, const Position& p,
                        float cam_translation_z, bool ready_for_action) {
    erase_if(percepts, [&p, this](const TrackedObject& o) {
        point_2d abs_pos = LocalizationUtils::relToAbs(o.posRel, p);
        float outside_field_margin = 0.42f;
        return !within(abs_pos.x, -SoccerField::length() / 2.f - outside_field_margin,
                       SoccerField::length() / 2.f + outside_field_margin) ||
               !within(abs_pos.y, -SoccerField::width() / 2.f - outside_field_margin,
                       SoccerField::width() / 2.f + outside_field_margin);
    });

    std::lock_guard<std::mutex> lock(mutex);
    proceedFrame(camTime);
    addPercepts(percepts, camTime, cam_translation_z);
    updateReadyForAction(ready_for_action);
    updateResults();

    // Erase 0 conf objects
    erase_if(mtList, [](const pair<int, TrackedObject>& e) { return e.second.confidence <= 0; });
}

bool MTtracker::isValidInitialPercept(const point_2d& imageCoords, float pDetection, int radius) {
    // if (pDetection < minInitialP) {
    //     return false;
    // }
    // ignore objects at the border of the image
    /*return within(imageCoords.x, radius, cam_width - radius) &&
           within(imageCoords.y, radius, cam_height - radius);*/
    return true;
}

void MTtracker::addPercepts(const std::vector<TrackedObject>& curObjVec, int64_t camTime,
                            float cam_translation_z) {
    map<float, pair<int /*cur*/, int /*model*/>> assignments;

    for (int i = 0; i < curObjVec.size(); i++) {
        for (auto& [j, o] : mtList) {
            float d = dist(curObjVec[i], o, cam_translation_z);
            if (d < 0.5f)
                assignments[d] = {i, j};
        }
    }

    set<int> assigned_new;
    set<int> assigned_old;
    // Merge new percepts with the model.
    // TODO: This shouldn't be greedy but rather something like the Hungarian algorithm.
    for (const auto& [dist, ids] : assignments) {
        if (contains(assigned_new, ids.first) || contains(assigned_old, ids.second))
            continue;
        mergeNewPercept(curObjVec[ids.first], ids.second, camTime);
        assigned_new.insert(ids.first);
        assigned_old.insert(ids.second);
    }

    // Decrease confidencde of unmatched objects.
    for (auto& [id, o] : mtList) {
        if (contains(assigned_old, id))
            continue;
        o.confidence = limit01(o.confidence - decreaseConfidence);
        o.pDetection *= 0.95f;
    }
    // Add new percepts.
    for (size_t i = 0; i < curObjVec.size(); i++) {
        if (contains(assigned_new, i))
            continue;
        if (optional<int> new_id = addPercept(curObjVec[i], camTime)) {
            assigned_old.insert(*new_id);
            for (const auto& [id, o] : mtList) {
                if (id == *new_id)
                    continue;
                mergeable[{min(id, *new_id), max(id, *new_id)}] = 0;
            }
        }
    }
    // Update mergeability.
    erase_if(mergeable, [this](const std::pair<std::pair<int, int>, float>& kv) {
        return !contains(mtList, kv.first.first) || !contains(mtList, kv.first.second);
    });
    map<int, int> merged;
    for (auto& [ids, m] : mergeable) {
        if (!contains(mtList, ids.second))
            continue;
        if (contains(assigned_old, ids.first) && contains(assigned_old, ids.second)) {
            m = 0;
            continue;
        }
        if (dist(mtList[ids.first], mtList[ids.second], cam_translation_z) > 0.5f) {
            m -= 0.05f;
            continue;
        }
        m += 0.1f;
        if (m < 1)
            continue;
        int merge_to = ids.first;
        while (contains(merged, merge_to))
            merge_to = merged[merge_to];
        merged[ids.second] = merge_to;
        mergeObjects(ids.second, merge_to);
    }

    // Erase 0 conf objects
    erase_if(mtList,
             [](const std::pair<int, TrackedObject>& kv) { return kv.second.confidence <= 0; });
}

void MTtracker::mergeObjects(int merge_from, int merge_to) {
    mtList[merge_to].confidence = max(mtList[merge_to].confidence, mtList[merge_from].confidence);
    mtList[merge_to].pDetection = max(mtList[merge_to].pDetection, mtList[merge_from].pDetection);
    mtList[merge_to].ownTeamProb =
            (*mtList[merge_to].ownTeamProb + *mtList[merge_from].ownTeamProb) / 2.f;
    if (mtList[merge_from].posHistory.size() > mtList[merge_to].posHistory.size())
        mtList[merge_to].posHistory = mtList[merge_from].posHistory;
    mtList[merge_to].posRel = (mtList[merge_to].posRel + mtList[merge_from].posRel) / 2.f;
    mtList[merge_to].age_us = min(mtList[merge_to].age_us, mtList[merge_from].age_us);
    mtList[merge_to].lastTimeSeen =
            max(mtList[merge_to].lastTimeSeen, mtList[merge_from].lastTimeSeen);
    mtList.erase(merge_from);
}

void MTtracker::mergeNewPercept(const TrackedObject& curObj, int id, int64_t camTime) {
    // Update the hypothesis
    mtList[id].confidence = limit01(mtList[id].confidence + increaseConfidence);
    mtList[id].pDetection = mtList[id].pDetection * 0.7f + curObj.pDetection * 0.3f;
    if (curObj.ownTeamProb)
        mtList[id].ownTeamProb = *mtList[id].ownTeamProb * 0.9f + *curObj.ownTeamProb * 0.1f;
    mtList[id].addHistory(curObj.posRel, camTime - startMTT);
    mtList[id].posRel = curObj.posRel;
    mtList[id].age_us = 0;
    mtList[id].lastTimeSeen = camTime;
}

optional<int> MTtracker::addPercept(const TrackedObject& curObj, int64_t camTime) {
    if (!isValidInitialPercept(curObj.imageCoords, curObj.pDetection, curObj.objImageRadius)) {
        return nullopt;
    }
    TrackedObject percept = curObj;
    percept.age_us = 0;
    percept.lastTimeSeen = camTime;
    percept.confidence = increaseConfidence;
    percept.addHistory(percept.posRel, camTime - startMTT);
    if (!percept.ownTeamProb)
        percept.ownTeamProb = 0.5f;
    mtList[next_id] = percept;
    return next_id++;
}

point_2d MTtracker::addOdometry(const point_2d& posRel, const Position& odo) {
    return posRel.rotated(-odo.a) - odo.point();
}

// Replace
inline float dot(point_2d a, point_2d b) {
    return a.x * b.x + a.y * b.y;
}

// Replace
inline float norm(point_2d a) {
    return std::sqrt(a.x * a.x + a.y * a.y);
}

inline point_2d averageVelocity(std::list<htwk::TrackedObject::PositionTime> history, float minTime, float maxTime) {
    if(history.empty())
        return {0, 0};

    auto last = history.begin();
    auto start = last;
    int i = 0;

    for (auto it = history.begin(); it != history.end(); ++it) {
        if(last->posTime - it->posTime > maxTime)
            break;

        start = it;
        i++;
    }

    double time = last->posTime - start->posTime;
    if(time >= minTime)
        return (last->posRel - start->posRel) / time;
    else
        return {0.f, 0.f};
}

void MTtracker::proceedFrame(const int64_t& camTime) {
    Position odo;
    if (std::shared_ptr<Image> img = image_subscriber.latest()) {
        odo = htwk::deltaOdoRelative(img->last_image_time, img->timestamp_us);
    }
    std::vector<int> to_delete;
    int deleted_for_age = 0;
    for (auto& [id, o] : mtList) {
        o.age_us += camTime - lastCamTime;
        if (o.age_us > 4'000'000 && o.confidence < checkConfidence) {
            to_delete.push_back(id);
            deleted_for_age++;
            continue;
        }

        constexpr float minTime = 0.6f;
        constexpr float maxTime = 0.9f;

        float timeDelta = (camTime - lastCamTime) / 1000000.f;
        point_2d velocity = averageVelocity(o.posHistory, minTime, maxTime);

        // Reset the expected movement if it is > 20 m/s
        if (norm(velocity) > 20.f)
            velocity = point_2d{0.f, 0.f};
        else {
            o.posRel += velocity * timeDelta;
            o.velocity = velocity;
        }

        o.posRel = addOdometry(o.posRel, odo);
        for (auto& pos : o.posHistory) {
            pos.posRel = addOdometry(pos.posRel, odo);
        }
    }
    for (int id : to_delete)
        mtList.erase(id);

    lastCamTime = camTime;
}

void MTtracker::updateReadyForAction(bool ready_for_action) {
    if (!ready_for_action) {
        mtList.clear();
        fallenTime = lastCamTime;
        last_ready_for_action = false;
    } else {
        last_ready_for_action = true;
    }
}

void MTtracker::updateResults() {
    std::multimap<float /*pDetection*/, int /*id*/> sorted;
    for (const auto& [id, o] : mtList)
        sorted.emplace(o.pDetection, id);

    std::vector<TrackedObject> resultVec;
    for (auto it = sorted.rbegin(); it != sorted.rend(); it++) {
        TrackedObject object = mtList[it->second];
        if (object.confidence > checkConfidence) {
            if (resultVec.size() < maxObjects) {
                if ((lastCamTime - fallenTime) > 1_s) {
                    object.calcSpeed();
                } else {
                    object.velocity = {.0f, .0f};
                    object.isMoving = false;
                }
                if (smoothing > 0 && !object.posHistory.empty()) {
                    object.posRel = {0, 0};
                    size_t cnt = 0;
                    for (const TrackedObject::PositionTime& pt : object.posHistory) {
                        if (pt.posTime < (lastCamTime - startMTT) / (double)1._s - smoothing)
                            break;
                        object.posRel += pt.posRel;
                    }
                    if (cnt == 0)
                        object.posRel = mtList[it->second].posRel;
                    else
                        object.posRel /= cnt;
                }
                resultVec.push_back(object);
            } else {
                publish_callback(resultVec);
                return;
            }
        }
    }
    publish_callback(resultVec);
}
}  // namespace htwk
