#pragma once

#include <cam_constants.h>
#include <cam_pose.h>
#include <channel.h>
#include <ellipse.h>
#include <imu.h>
#include <imu_joint_state.h>
#include <line.h>
#include <linecross.h>
#include <linegroup.h>
#include <object_hypothesis.h>
#include <odometer.h>
#include <parameter_tuner.h>
#include <point_2d.h>
#include <position.h>
#include <robot_time.h>
#include <sensor_pub_sub.h>
#include <soccerfield.h>
#include <vision_pub_sub.h>

#include <atomic>
#include <mutex>
#include <optional>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "cam_pose.h"
#include "gc_pub_sub.h"

class Localization {
public:
    // Describes the how a point-like feature (penalty spot, goal post, etc.) and a line interact.
    struct FeatureLine {
        float dist;  //!< (Extended) orthogonal distance to the feature
        float angle; /**< Angle of the line wrt the direction of the robot. Lines along the length
                        of the field are 0, lines across the field (e.g. center line) are M_PI/2.
                        (Or M_PI and -M_PI/2) */
        size_t id;   //!< Id of the line in field_lines
    };

    struct PointFeature {
        point_2d pos;
        size_t feature_id;
        std::vector<FeatureLine> lines;
    };

    struct ProjectedPointFeature {
        point_2d pos;
        htwk::ObjectType type;
    };

    Localization(PlayerIdx idx) : idx(idx) {

        // Initialize last_odo in constructor
        last_odo = std::make_shared<Odometer>();
        last_odo->x = 0;
        last_odo->y = 0;
        last_odo->theta = 0;

        // Precompute hessian for model lines to avoid multi-threading issues on concurrent access.
        for (auto& l : field_lines)
            l.convertToHessian();

        for (size_t i = 0; i < field_lines.size(); i++) {
            for (size_t j = i + 1; j < field_lines.size(); j++) {
                if (auto intersection = field_lines[i].intersect(field_lines[j])) {
                    if (std::any_of(line_intersections.begin(), line_intersections.end(),
                                    [&intersection](const auto& i) {
                                        return intersection->dist(-i) == 0;
                                    }))
                        continue;
                    line_pairs.emplace_back(i, j);
                    line_intersections.push_back(*intersection);
                }
            }
        }
        std::unordered_multimap<htwk::ObjectType, point_2d> point_objects{
                {htwk::ObjectType::PENALTY_SPOT, SoccerField::penaltySpots()[0]},
        };
        for (const auto& p : SoccerField::get_L()) {
            point_objects.emplace(htwk::ObjectType::L_SPOT, p);
        }
        for (const auto& p : SoccerField::get_T()) {
            point_objects.emplace(htwk::ObjectType::T_SPOT, p);
        }
        for (const auto& p : SoccerField::get_X()) {
            point_objects.emplace(htwk::ObjectType::X_SPOT, p);
        }

        // Center circle is feature 0
        size_t feature_id = 1;
        for (const auto& [type, pos] : point_objects) {
            PointFeature feature{.pos = pos, .feature_id = feature_id++};
            for (size_t i = 0; i < field_lines.size(); ++i) {
                if (field_lines[i].distance(pos) > 10.f)
                    continue;
                feature.lines.push_back({.dist = field_lines[i].extendedDistance(pos),
                                         .angle = field_lines[i].u().to_direction(),
                                         .id = i});
            }
            point_feature_model[type].emplace_back(feature);
        }

        for (const auto& [type, feature] : point_feature_model) {
            std::string name;

            if (type == htwk::ObjectType::CENTER_SPOT)
                name = "CENTER_SPOT";
            else if (type == htwk::ObjectType::PENALTY_SPOT)
                name = "PENALTY_SPOT";
            else if (type == htwk::ObjectType::GOAL_POST)
                name = "GOAL_POST";
            else if (type == htwk::ObjectType::L_SPOT)
                name = "L_SPOT";
            else if (type == htwk::ObjectType::T_SPOT)
                name = "T_SPOT";
            else if (type == htwk::ObjectType::X_SPOT)
                name = "X_SPOT";

            printf("%s: %zu features\n", name.c_str(), feature[0].lines.size());
        }
    }

    void proceed(CamPose& cam_pose, int64_t timestamp_us);
    htwk::Position getPosition() {
        std::lock_guard<std::mutex> lock(mutex);
        return position;
    }
    float getQuality() {
        std::lock_guard<std::mutex> lock(mutex);
        return loc_quality;
    }
    std::optional<PitchRoll> angleFromEllipse(const htwk::Ellipse& e);
    bool isGoodProjection() {
        return good_projection;
    }

    std::vector<Hypothesis> hypothesesPointFeatures(
            const std::vector<ProjectedPointFeature>& projected_points,
            std::vector<htwk::Line>& projected);
    // Matching the longest line gives a 0.25 bonus in matched_lines.
    std::pair<std::vector<Hypothesis>, float /*matched_lines*/> hypothesesLines(
            const std::vector<htwk::Line>& projected);
    std::vector<Hypothesis> merge(const std::vector<Hypothesis>& hyps);

private:
    htwk::ChannelSubscriber<std::shared_ptr<Odometer>> odometer_subscriber =
            htwk::odometer_channel.create_subscriber();
    std::shared_ptr<Odometer> last_odo;  // Member declaration only
    htwk::ChannelSubscriber<GCState> gc_state_sub = gc_state.create_subscriber();

    htwk::ChannelSubscriber<std::vector<htwk::LineGroup>> lines_subscriber =
            lines_channel.create_subscriber();

    htwk::ChannelSubscriber<std::vector<htwk::ObjectHypothesis>> point_features_subscriber =
            point_features_channel.create_subscriber();

    std::vector<htwk::Line> projectLines(const std::vector<htwk::LineGroup>& line_groups,
                                         CamPose& cam_pose);
    void visualizeRelative(const std::vector<htwk::Line>& lines,
                           const std::vector<ProjectedPointFeature>& projected_points,
                           CamPose& cam_pose);
    std::vector<Hypothesis> hypothesesCircle(const htwk::Ellipse& ellipse,
                                             const std::vector<htwk::LineGroup>& line_groups,
                                             CamPose& cam_pose);
    std::optional<Hypothesis> pointFeatureTriangulation(const point_2d& p1, const point_2d& p2,
                                                        float d1, float d2);
    void evaluateSideSwitching(int64_t image_time);

    std::vector<htwk::Line> field_lines = SoccerField::getFieldLines();
    std::vector<std::pair<size_t, size_t>> line_pairs;
    std::unordered_map<htwk::ObjectType, std::vector<PointFeature>> point_feature_model;
    std::vector<point_2d> line_intersections;
    std::atomic_bool initialized{false};  // guarded by mutex
    htwk::Position position;              // guarded by mutex
    float loc_quality = 0;                // guarded by mutex
    std::mutex mutex;
    // max correction
    static constexpr float f = 0.5f;
    static constexpr float f_yaw = 0.5f;
    // correction factors
    static constexpr float fp = 0.1f;
    static constexpr float fp_yaw = 0.15f;
    int64_t last_image_time = time_us();
    int64_t last_side_switch = 0;
    bool too_many_lines = false;
    bool good_projection = false;
    PlayerIdx idx;
};
