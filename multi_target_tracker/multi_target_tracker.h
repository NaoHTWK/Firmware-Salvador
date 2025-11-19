#pragma once

#include <camera_pub_sub.h>

#include <cstdint>
#include <loguru.hpp>
#include <optional>
#include <vector>

#include "algorithm_ext.h"
#include "cam_pose.h"
#include "localization_pub_sub.h"
#include "localization_utils.h"
#include "logging.h"
#include "multi_target_tracker_pub_sub.h"
#include "point_2d.h"
#include "position.h"
#include "rel_ball.h"
#include "robot_time.h"
#include "tracked_object.h"

namespace htwk {

class MTtracker {
public:
    MTtracker(size_t maxObjects, float smoothing, float minInitialP,
              std::function<void(const std::vector<TrackedObject>&)> publish_callback,
              float incConf = 0.2f, float duration = 2.f);

    void proceed(std::vector<TrackedObject> percepts, int64_t camTime, const Position& p,
                 float cam_translation_z, bool ready_for_action);

private:
    bool isValidInitialPercept(const point_2d& imageCoords, float pDetection, int radius);
    point_2d addOdometry(const point_2d& posRel, const Position& odo);
    void updateReadyForAction(bool ready_for_action);
    void updateResults();

    std::function<void(const std::vector<TrackedObject>&)> publish_callback;
    void addPercepts(const std::vector<TrackedObject>& curObjVec, int64_t camTime,
                     float cam_translation_z);
    std::optional<int> addPercept(const TrackedObject& curObj, int64_t camTime);
    void mergeNewPercept(const TrackedObject& curObj, int id, int64_t camTime);
    void mergeObjects(int merge_from, int merge_to);
    void proceedFrame(const int64_t& camTime);

    static constexpr float checkConfidence = .4f;
    static constexpr float imagesPerSecond = 30;

    const size_t maxObjects;
    const float smoothing;           // seconds
    const float increaseConfidence;  // <- this should be adjusted with experience
    const float durationInSeconds;
    const float decreaseConfidence;
    const float minInitialP;

    std::map<int /*id*/, TrackedObject> mtList;
    int next_id = 0;
    std::map<std::pair<int, int>, float /*mergeability*/> mergeable;
    bool last_ready_for_action = false;
    int64_t lastCamTime = time_us();
    int64_t fallenTime = time_us();
    const int64_t startMTT = time_us();

    htwk::ChannelSubscriber<std::shared_ptr<Image>> image_subscriber = images.create_subscriber();
};

class RelRobotsTracker : public MTtracker {
public:
    RelRobotsTracker()
        : MTtracker(10, 2.f, 0.93f, std::bind_front(&RelRobotsTracker::publishRelRobots, this)) {}

private:
    htwk::ChannelSubscriber<LocPosition> loc_position_sub =
            loc_position_channel.create_subscriber();

    void publishRelRobots(const std::vector<TrackedObject>& tracked_objects) {
        std::vector<RobotDetection> robots;
        std::vector<rerun::Position2D> positions;
        std::vector<rerun::Position2D> positions_abs;
        for (const TrackedObject& obj : tracked_objects) {
            robots.push_back(
                    RobotDetection{.pos_rel = obj.posRel,
                                   .ownTeamProb = obj.ownTeamProb ? *obj.ownTeamProb : 0.5f});
            positions.push_back(rerun::Position2D{obj.posRel.x, obj.posRel.y});
            auto loc_position = loc_position_sub.latest();
            auto abs_pos = LocalizationUtils::relToAbs(obj.posRel, loc_position.position);
            positions_abs.push_back(rerun::Position2D{abs_pos.x, abs_pos.y});
        }
        rel_robots_channel.publish(robots);
        htwk::log_rerun(
                "relative/far_obstacles",
                rerun::Points2D(positions).with_radii(rerun::Collection<rerun::components::Radius>(
                        rerun::components::Radius{0.1f})));
        htwk::log_rerun("soccerfield/far_obstacles",
                        rerun::Points2D(positions_abs)
                                .with_radii(rerun::Collection<rerun::components::Radius>(
                                        rerun::components::Radius{0.2f})));
    }
};

}  // namespace htwk
