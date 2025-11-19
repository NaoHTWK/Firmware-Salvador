#include "ball_tracker.h"

#include <cmath>
#include <opencv2/core/mat.hpp>

#include "cam_pose.h"
#include "image.h"
#include "localization_pub_sub.h"
#include "localization_utils.h"
#include "logging.h"
#include "point_2d.h"
#include "position.h"
#include "soccerfield.h"
#include "tensorflow/lite/kernels/internal/compatibility.h"
#include "vision_pub_sub.h"

BallTracker::BallTracker()
    : ball_sub(ball_hypothesis_channel.create_subscriber()),
      loc_position_sub(loc_position_channel.create_subscriber()) {}

BallTracker::~BallTracker() = default;

void BallTracker::publishBallCallback(const std::vector<htwk::TrackedObject>& tracked_objects) {
    if (tracked_objects.empty()) {
        rel_ball_channel.publish(std::nullopt);
        htwk::log_rerun("relative/ball", rerun::Clear());
        htwk::log_rerun("soccerfield/ball", rerun::Clear());
        return;
    }

    // get the tracked object with the highest confidence
    auto tracked_object =
            std::max_element(tracked_objects.begin(), tracked_objects.end(),
                             [](const htwk::TrackedObject& a, const htwk::TrackedObject& b) {
                                 return a.confidence < b.confidence;
                             });

    std::vector<rerun::Position2D> positions;
    positions.push_back(rerun::Position2D(tracked_object->posRel.x, tracked_object->posRel.y));
    htwk::log_rerun("relative/ball", rerun::Points2D(positions).with_radii({0.15f}));

    htwk::ChannelSubscriber<LocPosition> loc_position_sub =
            loc_position_channel.create_subscriber();
    auto loc_position = loc_position_sub.latestIfExists();
    if (loc_position) {
        point_2d ball_position =
                LocalizationUtils::relToAbs(tracked_object->posRel, loc_position->position);
        htwk::log_rerun("soccerfield/ball",
                        rerun::Points2D({{ball_position.x, ball_position.y}}).with_radii({0.1}));
    }

    rel_ball_channel.publish(RelBall{.pos_rel = tracked_object->posRel,
                                     .ball_age_us = tracked_object->age_us,
                                     .last_seen_time = tracked_object->lastTimeSeen,
                                     .velocity = tracked_object->velocity,
                                     .high_risk_velocity = tracked_object->high_risk_velocity,
                                     .medium_risk_velocity = tracked_object->medium_risk_velocity,
                                     .is_moving = tracked_object->isMoving});
}

void BallTracker::proceed(const Image& img) {
    std::optional<htwk::ObjectHypothesis> ball = ball_sub.latest();
    std::optional<LocPosition> loc_position = loc_position_sub.latestIfExists();

    htwk::Position robot_pos = htwk::Position(0.0, 0.0, 0.0);

    if (loc_position) {
        robot_pos = htwk::Position(loc_position->position.x, loc_position->position.y,
                                   loc_position->position.a);
    }

    std::vector<htwk::TrackedObject> percepts;

    if (ball) {
        if (auto pBall = CamUtils::project(ball->point(), ball_radius, *img.cam_pose_ptr)) {
            if (true /*isValidBall(img, ball->point(), pBall.value())*/) {
                percepts.emplace_back(*pBall, ball->prob, std::nullopt, ball->r, ball->point());
            } else {
                LOG_F(INFO, "Ball is invalid");
            }
        }
    }
    ball_tracker.proceed(percepts, img.timestamp_us, robot_pos,
                         img.cam_pose_ptr->get_translation().z, true);
}

bool BallTracker::isValidBall(const Image& img, point_2d ball, point_2d ball_pos) {
    int ball_x = int(ball.x);
    int ball_y = int(ball.y);
    if (ball_x < 0 || ball_x >= img.WIDTH || ball_y < 0 || ball_y >= img.HEIGHT) {
        LOG_F(INFO, "Ball is out of image");
        return true;
    }
    float depth = img.data_depth_m[ball_y * img.WIDTH + ball_x];

    if (std::isnan(depth)) {
        return true;
    }

    auto loc_position_opt = loc_position_sub.latestIfExists();
    if (!loc_position_opt) {
        return true;
    }

    point_2d abs_ball = LocalizationUtils::relToAbs(ball_pos, loc_position_opt->position);
    std::vector<point_2d> penalty_spots = SoccerField::penaltySpots();

    // only use to filter out penalty spots
    bool near_penalty_spot = false;
    for (const auto& penalty_spot : penalty_spots) {
        float dist = penalty_spot.dist(abs_ball);
        if (dist < 1.5f) {
            near_penalty_spot = true;
            break;
        }
    }

    if (!near_penalty_spot) {
        return true;
    }

    // Check if depth is valid (not NaN and within reasonable range)
    if (ball_pos.norm() > 0.1f && ball_pos.norm() < 5.0f) {
        const CamPose& cam_pose = *img.cam_pose_ptr;
        auto pBallGround = CamUtils::pixelToPlane(ball.x, ball.y, cam_pose.plane, cam_pose.get_fx(),
                                                  cam_pose.get_fy(), cam_pose.get_cx(),
                                                  cam_pose.get_cy(), 0.f);
        if (!pBallGround) {
            return true;
        }
        float dist = sqrt(pBallGround->x * pBallGround->x + pBallGround->y * pBallGround->y);
        float cam_height = pBallGround->z;

        float calc_depth = sqrt(dist * dist + cam_height * cam_height);
        if (calc_depth - depth > 0.22) {
            return true;
        }
        return false;
    }
    return true;
}
