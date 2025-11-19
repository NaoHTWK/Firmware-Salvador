#include "htwk_vision.h"

#include <cam_pose.h>
#include <fluxlog_vision.h>
#include <logging.h>
#include <pub_sub/vision_pub_sub.h>

#include <array>
#include <cstdint>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "bounding_box.h"
#include "ellipse.h"
#include "image.h"
#include "plane_detection.h"
#include "point_2d.h"
#include "robot_time.h"
namespace htwk {

// Add profiler variables
static int vision_loop_count = 0;
static int64_t last_loop_log_time = time_us();

// Add timing accumulators for different proceed operations
static int64_t total_proceed_time_acc = 0;
static int64_t plane_time_acc = 0;
static int64_t field_border_time_acc = 0;
static int64_t field_color_time_acc = 0;
static int64_t booster_yolo_time_acc = 0;
static int64_t line_time_acc = 0;
static int64_t near_obstacle_time_acc = 0;
static int64_t log_image_time_acc = 0;

HTWKVision::HTWKVision() {
    thread_pool = new ::ThreadPool("Vision");
    fieldBorderDetector = std::make_shared<FieldBorderDetector>();
    regionClassifier = new RegionClassifier();
    lineDetector = new LineDetector();
    obstacle_detector_legacy = new ObstacleDetectorLegacy();
    fieldColorDetector = new FieldColorDetector();

    this->booster_yoloV8_detector = booster_vision::YoloV8Detector::CreateYoloV8Detector();
    if (!this->booster_yoloV8_detector) {
        LOG_F(ERROR, "Failed to create YoloV8Detector");
        throw std::runtime_error("Failed to create YoloV8Detector");
    }
}

HTWKVision::~HTWKVision() {
    delete fieldColorDetector;
    delete regionClassifier;
    delete lineDetector;
    delete obstacle_detector_legacy;
}

void HTWKVision::publish_booster_vision_to_pubsub(const Image& img) {

    // ball
    std::vector<htwk::ObjectHypothesis> ball_hypotheses;
    for (booster_vision::DetectionRes& ball : booster_yolo_results.ball) {
        float size =
                std::max(static_cast<float>(ball.bbox.width), static_cast<float>(ball.bbox.height));
        htwk::ObjectHypothesis ball_hypothesis =
                htwk::ObjectHypothesis(point_2d(static_cast<float>(ball.bbox.x) +
                                                        (static_cast<float>(ball.bbox.width) / 2),
                                                static_cast<float>(ball.bbox.y) +
                                                        (static_cast<float>(ball.bbox.height) / 2)),
                                       size / 4, ball.confidence);
        ball_hypothesis.prob = ball.confidence;
        ball_hypothesis.type = htwk::ObjectType::BALL;
        ball_hypotheses.push_back(ball_hypothesis);
    }
    if (booster_yolo_results.ball.empty()) {
        ball_hypothesis_channel.publish(std::nullopt);
    } else {
        auto best_ball =
                *std::max_element(ball_hypotheses.begin(), ball_hypotheses.end(),
                                  [](const htwk::ObjectHypothesis& a,
                                     const htwk::ObjectHypothesis& b) { return a.prob < b.prob; });
        ball_hypothesis_channel.publish(best_ball);
        cam_ball_callback(best_ball.point(), img.cam_pose_ptr->head_angles, img.timestamp_us,
                          img.cx, img.cy, img.fx, img.fy);
    }

    // penalty spot
    std::vector<htwk::ObjectHypothesis> penalty_spot_hypotheses;
    for (booster_vision::DetectionRes& penalty_spot : booster_yolo_results.penaltypoint) {
        htwk::ObjectHypothesis penalty_spot_hypothesis = htwk::ObjectHypothesis(
                point_2d(static_cast<float>(penalty_spot.bbox.x) +
                                 (static_cast<float>(penalty_spot.bbox.width) / 2),
                         static_cast<float>(penalty_spot.bbox.y) +
                                 (static_cast<float>(penalty_spot.bbox.height) / 2)),
                static_cast<float>(penalty_spot.bbox.width) / 4 +
                        static_cast<float>(penalty_spot.bbox.height) / 4,
                penalty_spot.confidence);
        penalty_spot_hypothesis.prob = penalty_spot.confidence;
        penalty_spot_hypothesis.type = htwk::ObjectType::PENALTY_SPOT;
        penalty_spot_hypotheses.push_back(penalty_spot_hypothesis);
    }

    // goal post lower bounding box center
    // disable goal posts since they don't fit well with the way we do localization.
    // std::vector<htwk::ObjectHypothesis> goal_post_hypotheses;
    // for (booster_vision::DetectionRes& goal_post : booster_yolo_results.goalpost) {
    //     htwk::ObjectHypothesis goal_post_hypothesis = htwk::ObjectHypothesis(
    //             point_2d(static_cast<float>(goal_post.bbox.x) +
    //                              (static_cast<float>(goal_post.bbox.width) / 2),
    //                      static_cast<float>(goal_post.bbox.y) +
    //                              (static_cast<float>(goal_post.bbox.height))),
    //             static_cast<float>(goal_post.bbox.width) / 2, goal_post.confidence);
    //     goal_post_hypothesis.prob = goal_post.confidence;
    //     goal_post_hypothesis.type = htwk::ObjectType::GOAL_POST;
    //     goal_post_hypotheses.push_back(goal_post_hypothesis);
    // }

    // L spot
    std::vector<htwk::ObjectHypothesis> L_spot_hypotheses;
    for (booster_vision::DetectionRes& l_spot : booster_yolo_results.lcross) {
        htwk::ObjectHypothesis l_spot_hypothesis = htwk::ObjectHypothesis(
                point_2d(static_cast<float>(l_spot.bbox.x) +
                                 (static_cast<float>(l_spot.bbox.width) / 2),
                         static_cast<float>(l_spot.bbox.y) +
                                 (static_cast<float>(l_spot.bbox.height) / 2)),
                static_cast<float>(l_spot.bbox.width) / 4 +
                        static_cast<float>(l_spot.bbox.height) / 4,
                l_spot.confidence);
        l_spot_hypothesis.prob = l_spot.confidence;
        l_spot_hypothesis.type = htwk::ObjectType::L_SPOT;
        L_spot_hypotheses.push_back(l_spot_hypothesis);
    }

    // T spot
    std::vector<htwk::ObjectHypothesis> T_spot_hypotheses;
    for (booster_vision::DetectionRes& t_spot : booster_yolo_results.tcross) {
        htwk::ObjectHypothesis t_spot_hypothesis = htwk::ObjectHypothesis(
                point_2d(static_cast<float>(t_spot.bbox.x) +
                                 (static_cast<float>(t_spot.bbox.width) / 2),
                         static_cast<float>(t_spot.bbox.y) +
                                 (static_cast<float>(t_spot.bbox.height) / 2)),
                static_cast<float>(t_spot.bbox.width) / 4 +
                        static_cast<float>(t_spot.bbox.height) / 4,
                t_spot.confidence);
        t_spot_hypothesis.prob = t_spot.confidence;
        t_spot_hypothesis.type = htwk::ObjectType::T_SPOT;
        T_spot_hypotheses.push_back(t_spot_hypothesis);
    }

    // X spot
    std::vector<htwk::ObjectHypothesis> X_spot_hypotheses;
    for (booster_vision::DetectionRes& x_spot : booster_yolo_results.xcross) {
        htwk::ObjectHypothesis x_spot_hypothesis = htwk::ObjectHypothesis(
                point_2d(static_cast<float>(x_spot.bbox.x) +
                                 (static_cast<float>(x_spot.bbox.width) / 2),
                         static_cast<float>(x_spot.bbox.y) +
                                 (static_cast<float>(x_spot.bbox.height) / 2)),
                static_cast<float>(x_spot.bbox.width) / 4 +
                        static_cast<float>(x_spot.bbox.height) / 4,
                x_spot.confidence);
        x_spot_hypothesis.prob = x_spot.confidence;
        x_spot_hypothesis.type = htwk::ObjectType::X_SPOT;
        X_spot_hypotheses.push_back(x_spot_hypothesis);
    }

    std::vector<htwk::ObjectHypothesis> point_features;
    point_features.insert(point_features.end(), penalty_spot_hypotheses.begin(),
                          penalty_spot_hypotheses.end());
    // point_features.insert(point_features.end(), goal_post_hypotheses.begin(),
    //                       goal_post_hypotheses.end());
    point_features.insert(point_features.end(), L_spot_hypotheses.begin(), L_spot_hypotheses.end());
    point_features.insert(point_features.end(), T_spot_hypotheses.begin(), T_spot_hypotheses.end());
    point_features.insert(point_features.end(), X_spot_hypotheses.begin(), X_spot_hypotheses.end());
    point_features_channel.publish(point_features);

    // Robots
    std::vector<htwk::ObjectHypothesis> robot_hypotheses;
    for (booster_vision::DetectionRes& robot : booster_yolo_results.opponent) {
        htwk::ObjectHypothesis robot_hypothesis =
                htwk::ObjectHypothesis(point_2d(static_cast<float>(robot.bbox.x) +
                                                        (static_cast<float>(robot.bbox.width) / 2),
                                                static_cast<float>(robot.bbox.y) +
                                                        (static_cast<float>(robot.bbox.height))),
                                       static_cast<float>(robot.bbox.width) / 2, robot.confidence);
        robot_hypothesis.prob = robot.confidence;
        robot_hypothesis.type = htwk::ObjectType::ROBOT;
        robot_hypotheses.push_back(robot_hypothesis);
    }
    opponent_hypotheses_channel.publish(robot_hypotheses);
}

void HTWKVision::proceed(const std::shared_ptr<Image> img) {

    int64_t proceed_start_time = time_us();

    TaskScheduler scheduler(thread_pool);

    auto plane = scheduler.addTask(
            [&]() {
                int64_t plane_start_time = time_us();
                img->cam_pose_ptr->plane =
                        estimate_plane_ransac(img->data_depth_m.data(), img->WIDTH, img->HEIGHT,
                                              img->fx, img->fy, img->cx, img->cy);
                float a = img->cam_pose_ptr->plane[0];
                float b = img->cam_pose_ptr->plane[1];
                float c = img->cam_pose_ptr->plane[2];
                float d = img->cam_pose_ptr->plane[3];

                // Distance from origin (0,0,0) to plane ax+by+cz+d=0
                float distance = std::abs(d) / std::sqrt(a * a + b * b + c * c);
                // TODO: get expected height from somewhere
                float cam_height = img->cam_pose_ptr->get_translation().z;
                if (distance < cam_height - 0.10 || distance > cam_height + 0.10) {
                    img->cam_pose_ptr->plane = {};
                } else {
                    img->cam_pose_ptr->rotation =
                            CamUtils::planeToHorizontalRotation(img->cam_pose_ptr->plane);
                }

                plane_time_acc += time_us() - plane_start_time;
            },
            {});

    auto fieldBorder = scheduler.addTask(
            [&]() {
                int64_t start_time = time_us();
                fieldBorderDetector->proceed(img.get());
                field_border_time_acc += time_us() - start_time;
            },
            {});

    auto regions = scheduler.addTask(
            [&]() {
                int64_t start_time = time_us();
                fieldColorDetector->proceed(img.get());
                regionClassifier->proceed(img.get(), fieldColorDetector);
                field_color_time_acc += time_us() - start_time;
            },
            {});
    auto lines = scheduler.addTask(
            [&]() {
                int64_t start_time = time_us();
                // LineDetector modifies the LineSegments from RegionClassifier.
                lineDetector->proceed(img.get(),
                                      regionClassifier->getLineSegments(
                                              fieldBorderDetector->getConvexFieldBorder()),
                                      regionClassifier->lineSpacing);
                line_time_acc += time_us() - start_time;
            },
            {regions, fieldBorder});

    auto booster_vision = scheduler.addTask(
            [&]() {
                int64_t start_time = time_us();
                booster_yolo_results = booster_yoloV8_detector->proceed(img->data_bgra.data());
                publish_booster_vision_to_pubsub(*img);
                booster_yolo_time_acc += time_us() - start_time;
            },
            {});
    auto obstacle_detection_legacy_task = scheduler.addTask(
            [&]() {
                int64_t start_time = time_us();
                // we need the ball to filter it out
                std::vector<BoundingBox> ball_boxes;

                for (booster_vision::DetectionRes& booster_ball : booster_yolo_results.ball) {
                    point_2d p1 = {static_cast<float>(booster_ball.bbox.x), static_cast<float>(booster_ball.bbox.y)};
                    point_2d p2 = {static_cast<float>(booster_ball.bbox.x + booster_ball.bbox.width),
                                   static_cast<float>(booster_ball.bbox.y + booster_ball.bbox.height)};
                    float prob = 1.f;
                    ball_boxes.push_back(BoundingBox(p1, p2, prob));
                }

                obstacle_detector_legacy->proceed(img, img->cam_pose_ptr->plane, ball_boxes);
                near_obstacle_time_acc += time_us() - start_time;
            },
            {plane, booster_vision});
    auto log_image = scheduler.addTask(
            [&]() {
                int64_t start_time = time_us();
                img->log();
                log_image_time_acc += time_us() - start_time;
            },
            {});
    scheduler.run();

    total_proceed_time_acc += time_us() - proceed_start_time;

    // Profiler logging (similar to main.cpp)
    vision_loop_count++;
    int64_t now = time_us();
    if (now - last_loop_log_time > 1'000'000) {
        if (vision_loop_count > 0) {
            LOG_F(DEBUG,
                  "avg times (ms): total: %.4f, plane: %.4f, booster_yolo: %.4f, line_detection: "
                  "%.4f, field_border: %.4f, field_color: %.4f, near_obstacle: %.4f, log_image: "
                  "%.4f, fps: %i",
                  static_cast<double>(total_proceed_time_acc) / vision_loop_count / 1000.0,
                  static_cast<double>(plane_time_acc) / vision_loop_count / 1000.0,
                  static_cast<double>(booster_yolo_time_acc) / vision_loop_count / 1000.0,
                  static_cast<double>(line_time_acc) / vision_loop_count / 1000.0,
                  static_cast<double>(field_border_time_acc) / vision_loop_count / 1000.0,
                  static_cast<double>(field_color_time_acc) / vision_loop_count / 1000.0,
                  static_cast<double>(near_obstacle_time_acc) / vision_loop_count / 1000.0,
                  static_cast<double>(log_image_time_acc) / vision_loop_count / 1000.0,
                  vision_loop_count);
        }
        last_loop_log_time += 1'000'000;
        vision_loop_count = 0;
        total_proceed_time_acc = 0;
        plane_time_acc = 0;
        booster_yolo_time_acc = 0;
        line_time_acc = 0;
        near_obstacle_time_acc = 0;
        field_border_time_acc = 0;
        field_color_time_acc = 0;
        log_image_time_acc = 0;
    }

    log();
};

void HTWKVision::log() {
    // fw sydney stuff
    //  log lines
    std::vector<rerun::LineStrip2D> linesList;
    std::vector<LineGroup> lineGroups = lineDetector->getLineGroups();
    if (lineGroups.size() < 0) {
        htwk::log_rerun("image/percepts/lines", rerun::Clear());
    } else {
        for (int i = 0; i < lineGroups.size(); i++) {
            float line_px1_1 = lineGroups[i].lines[0].px1;
            float line_py1_1 = lineGroups[i].lines[0].py1;
            float line_px2_1 = lineGroups[i].lines[0].px2;
            float line_py2_1 = lineGroups[i].lines[0].py2;
            float line_px1_2 = lineGroups[i].lines[1].px1;
            float line_py1_2 = lineGroups[i].lines[1].py1;
            float line_px2_2 = lineGroups[i].lines[1].px2;
            float line_py2_2 = lineGroups[i].lines[1].py2;

            rerun::LineStrip2D line1({{line_px1_1, line_py1_1}, {line_px2_1, line_py2_1}});
            linesList.push_back(line1);
            rerun::LineStrip2D line2({{line_px1_2, line_py1_2}, {line_px2_2, line_py2_2}});
            linesList.push_back(line2);
        }
        htwk::log_vision_lines("image/percepts/lines", linesList, 2.0f,
                               rerun::Color(255, 0, 255, 255));
    }

    // fw salvador stuff
    // log ball
    std::vector<cv::Rect> ball_rects;
    std::vector<std::string> ball_confidence;
    ball_rects.reserve(booster_yolo_results.ball.size());
    for (auto ball : booster_yolo_results.ball) {
        ball_rects.emplace_back(ball.bbox);
        ball_confidence.emplace_back(std::to_string(ball.confidence));
    };

    htwk::log_vision_OpencvRect("image/percepts/ball", ball_rects, 4.0f,
                                rerun::Color(255, 80, 80, 255), ball_confidence, false);
    // log goalpost
    std::vector<cv::Rect> goalpost_rects;
    std::vector<std::string> goalpost_confidence;
    goalpost_rects.reserve(booster_yolo_results.goalpost.size());
    for (auto goalpost : booster_yolo_results.goalpost) {
        goalpost_rects.emplace_back(goalpost.bbox);
        goalpost_confidence.emplace_back(std::to_string(goalpost.confidence));
    };
    htwk::log_vision_OpencvRect("image/percepts/goalpost", goalpost_rects, 2.0f,
                                rerun::Color(128, 128, 128, 255), goalpost_confidence, false);
    // log person
    std::vector<cv::Rect> person_rects;
    std::vector<std::string> person_confidence;
    person_rects.reserve(booster_yolo_results.person.size());
    for (auto pers : booster_yolo_results.person) {
        person_rects.emplace_back(pers.bbox);
        person_confidence.emplace_back(std::to_string(pers.confidence));
    };
    htwk::log_vision_OpencvRect("image/percepts/person", person_rects, 2.0f,
                                rerun::Color(255, 80, 255, 255), person_confidence, false);
    // log lcross
    std::vector<cv::Rect> lcross_rects;
    std::vector<std::string> lcross_confidence;
    lcross_rects.reserve(booster_yolo_results.lcross.size());
    for (auto lcros : booster_yolo_results.lcross) {
        lcross_rects.emplace_back(lcros.bbox);
        lcross_confidence.emplace_back(std::to_string(lcros.confidence));
    };
    htwk::log_vision_OpencvRect("image/percepts/lcross", lcross_rects, 2.0f,
                                rerun::Color(0, 255, 0, 255), lcross_confidence, false);
    // log tcross
    std::vector<cv::Rect> tcross_rects;
    std::vector<std::string> tcross_confidence;
    tcross_rects.reserve(booster_yolo_results.tcross.size());
    for (auto tcros : booster_yolo_results.tcross) {
        tcross_rects.emplace_back(tcros.bbox);
        tcross_confidence.emplace_back(std::to_string(tcros.confidence));
    };
    htwk::log_vision_OpencvRect("image/percepts/tcross", tcross_rects, 2.0f,
                                rerun::Color(0, 0, 255, 255), tcross_confidence, false);
    // log xcross
    std::vector<cv::Rect> xcross_rects;
    std::vector<std::string> xcross_confidence;
    xcross_rects.reserve(booster_yolo_results.xcross.size());
    for (auto xcros : booster_yolo_results.xcross) {
        xcross_rects.emplace_back(xcros.bbox);
        xcross_confidence.emplace_back(std::to_string(xcros.confidence));
    };
    htwk::log_vision_OpencvRect("image/percepts/xcross", xcross_rects, 2.0f,
                                rerun::Color(255, 0, 0, 255), xcross_confidence, false);
    // log penaltypoint
    std::vector<cv::Rect> penaltypoint_rect;
    std::vector<std::string> penaltypoint_confidence;
    penaltypoint_rect.reserve(booster_yolo_results.penaltypoint.size());
    for (auto penaltypoint : booster_yolo_results.penaltypoint) {
        penaltypoint_rect.emplace_back(penaltypoint.bbox);
        penaltypoint_confidence.emplace_back(std::to_string(penaltypoint.confidence));
    };
    htwk::log_vision_OpencvRect("image/percepts/penaltypoint", penaltypoint_rect, 2.0f,
                                rerun::Color(255, 165, 0, 255), penaltypoint_confidence, false);
    // log opponent
    std::vector<cv::Rect> opponent_rect;
    std::vector<std::string> opponent_confidence;
    opponent_rect.reserve(booster_yolo_results.opponent.size());
    for (auto opponent : booster_yolo_results.opponent) {
        opponent_rect.emplace_back(opponent.bbox);
        opponent_confidence.emplace_back(std::to_string(opponent.confidence));
    };
    htwk::log_vision_OpencvRect("image/percepts/opponent", opponent_rect, 2.0f,
                                rerun::Color(255, 200, 255, 255), opponent_confidence, false);
    // log brmarker
    std::vector<cv::Rect> brmarker_rect;
    std::vector<std::string> brmarker_confidence;
    brmarker_rect.reserve(booster_yolo_results.brmarker.size());
    for (auto brmarker : booster_yolo_results.brmarker) {
        brmarker_rect.emplace_back(brmarker.bbox);
        brmarker_confidence.emplace_back(std::to_string(brmarker.confidence));
    };
    htwk::log_vision_OpencvRect("image/percepts/brmarker", brmarker_rect, 2.0f,
                                rerun::Color(255, 200, 255, 255), brmarker_confidence, false);
};

}  // namespace htwk
