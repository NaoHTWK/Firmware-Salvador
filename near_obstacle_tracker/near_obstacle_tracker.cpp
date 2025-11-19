#include "near_obstacle_tracker.h"

#include <asm-generic/ioctl.h>
#include <point_2d.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "fluxlog_vision.h"
#include "localization_utils.h"
#include "logging.h"
#include "near_obstacle_tracker_pub_sub.h"
#include "near_obstacle_tracker_result.h"
#include "raster.h"
#include "rerun.hpp"
#include "rerun/archetypes/boxes2d.hpp"
#include "rerun/archetypes/points2d.hpp"
#include "sensor_pub_sub.h"
#include "vision_pub_sub.h"


#include "team_strategy_pub_sub.h"
#include "gc_state.h"


namespace htwk {

// Add profiler variables
static int near_obstacle_loop_count = 0;
static int64_t last_near_obstacle_log_time = time_us();

// Add timing accumulators for different operations
static int64_t total_proceed_time_acc = 0;
static int64_t prepare_update_time_acc = 0;
static int64_t total_data_processing_time_acc = 0;
static int64_t near_obstacles_processing_time_acc = 0;
static int64_t no_obstacles_processing_time_acc = 0;
static int64_t ball_obstacles_processing_time_acc = 0;
static int64_t probability_update_time_acc = 0;
static int64_t visualization_time_acc = 0;
static int64_t publishing_time_acc = 0;

void NearObstacleTracker::reshape() {
    int dx = round_int(pos.x / resolution);
    int dy = round_int(pos.y / resolution);
    Raster<float> new_probs(size, size, 0);
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            if (obstacle_probs.inside(x + dx, y + dy)) {
                new_probs(x, y) = obstacle_probs(x + dx, y + dy);
            }
        }
    }
    pos.x -= dx * resolution;
    pos.y -= dy * resolution;
    obstacle_probs = new_probs;
}

void NearObstacleTracker::prepareUpdate(int64_t img_time, bool ready_for_action) {
    if (!ready_for_action) {
        pos = Position{0, 0, 0};
        obstacle_probs = Raster<float>(size, size, 0);
        last_image_time = img_time;
        return;
    }
    Position odo = htwk::deltaOdoRelative(last_image_time, img_time);
    pos.add_relative(odo);
    if (abs(pos.x) > reshape_dist || abs(pos.y) > reshape_dist)
        reshape();
    // fade out obstacles
    for (float& val : obstacle_probs) {
        val = std::max(0.f, val - dec_per_usec * (img_time - last_image_time));
    }
}

NearObstacleTracker::NearObstacleTracker()
    : near_obstacles_subscriber(near_obstacles_channel.create_subscriber()),
      no_near_obstacles_subscriber(no_near_obstacles_channel.create_subscriber()),
      ball_near_obstacles_subscriber(ball_obstacles_channel.create_subscriber()) {

        on_gc_state_change.registerCallback(
            [this](const GCState& old_state, const GCState& new_state) {
                if (old_state.state == GameState::Set &&
                    new_state.state != GameState::Set ) {
                    clear();
                }
                if (old_state.state == GameState::Initial &&
                    new_state.state != GameState::Initial ) {
                    clear();
                }
                auto player_idx = old_state.player_idx;
                auto old_player = old_state.my_team.players.at(player_idx);
                auto new_player = new_state.my_team.players.at(player_idx);

                if (old_player.is_penalized &&
                    !new_player.is_penalized) {
                    clear();
                }
            });
      }

void NearObstacleTracker::proceed(int64_t img_time) {
    int64_t proceed_start_time = time_us(); 

    // TODO ready_for_action
    bool ready_for_action = true;

    auto near_obstacles_ptr = near_obstacles_subscriber.latest();
    auto no_near_obstacles_ptr = no_near_obstacles_subscriber.latest();
    auto ball_obstacles_ptr = ball_near_obstacles_subscriber.latest();

    // Handle null pointers and dereference
    std::vector<point_2d> near_obstacles = near_obstacles_ptr ? *near_obstacles_ptr : std::vector<point_2d>();
    std::vector<point_2d> no_near_obstacles = no_near_obstacles_ptr ? *no_near_obstacles_ptr : std::vector<point_2d>();
    std::vector<point_2d> ball_obstacles = ball_obstacles_ptr ? *ball_obstacles_ptr : std::vector<point_2d>();

    int64_t prepare_start_time = time_us();
    prepareUpdate(img_time, ready_for_action);
    prepare_update_time_acc += time_us() - prepare_start_time;

    int64_t total_data_start_time = time_us();
    Raster<uint8_t> obstacle_candidates(size, size, 0);
    Raster<uint8_t> no_obstacle_candidates(size, size, 0);
    Raster<uint8_t> ball_candidates(size, size, 0);
    
    int64_t near_obstacles_start_time = time_us();
    for (const point_2d& p : near_obstacles) {
        point_2d point_on_raster = (LocalizationUtils::relToAbs(p, pos) / resolution) +
                                   point_2d(size / 2.f, size / 2.f);
        if (obstacle_candidates.inside(point_on_raster.x, point_on_raster.y)) {
            uint8_t& val = obstacle_candidates(point_on_raster.x, point_on_raster.y);
            val = 1;
        }
    }
    near_obstacles_processing_time_acc += time_us() - near_obstacles_start_time;
    
    int64_t no_obstacles_start_time = time_us();
    for (const point_2d& p : no_near_obstacles) {
        point_2d point_on_raster = (LocalizationUtils::relToAbs(p, pos) / resolution) +
                                   point_2d(size / 2.f, size / 2.f);
        if (no_obstacle_candidates.inside(point_on_raster.x, point_on_raster.y)) {
            uint8_t& val = no_obstacle_candidates(point_on_raster.x, point_on_raster.y);
            val = 1;
        }
    }
    no_obstacles_processing_time_acc += time_us() - no_obstacles_start_time;
    
    int64_t ball_obstacles_start_time = time_us();
    for (const point_2d& p : ball_obstacles) {
        point_2d point_on_raster = (LocalizationUtils::relToAbs(p, pos) / resolution) +
                                   point_2d(size / 2.f, size / 2.f);
        if (ball_candidates.inside(point_on_raster.x, point_on_raster.y)) {
            uint8_t& val = ball_candidates(point_on_raster.x, point_on_raster.y);
            val = 1;
        }
    }
    ball_obstacles_processing_time_acc += time_us() - ball_obstacles_start_time;
    
    total_data_processing_time_acc += time_us() - total_data_start_time;

    int64_t prob_start_time = time_us();
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            if (ball_candidates(x, y)) {
                float new_prob =
                        obstacle_probs(x, y) - 60 * dec_per_usec * (img_time - last_image_time);
                obstacle_probs(x, y) = clamp(new_prob, 0.f, 1.f);
            } else if (obstacle_candidates(x, y)) {
                float new_prob =
                        obstacle_probs(x, y) + 20 * dec_per_usec * (img_time - last_image_time);
                obstacle_probs(x, y) = clamp(new_prob, 0.f, 1.f);
            } else if (no_obstacle_candidates(x, y)) {
                float new_prob =
                        obstacle_probs(x, y) - 20 * dec_per_usec * (img_time - last_image_time);
                obstacle_probs(x, y) = clamp(new_prob, 0.f, 1.f);
            }
        }
    }
    probability_update_time_acc += time_us() - prob_start_time;

    int64_t publish_start_time = time_us();
    std::shared_ptr<NearObstacleTrackerResult> result =
            std::make_shared<NearObstacleTrackerResult>(obstacle_probs, pos, resolution);
    near_obstacles_tracker_result_channel.publish(result);
    publishing_time_acc += time_us() - publish_start_time;

    int64_t viz_start_time = time_us();
    visualize();
    visualization_time_acc += time_us() - viz_start_time;

    last_image_time = img_time;

    total_proceed_time_acc += time_us() - proceed_start_time;

    // Profiler logging
    near_obstacle_loop_count++;
    int64_t now = time_us();
    if (now - last_near_obstacle_log_time > 1'000'000) {
        if (near_obstacle_loop_count > 0) {
            LOG_F(DEBUG,
                  "NearObstacleTracker avg times (ms): total: %.4f, prepare: %.4f, total_data: %.4f, "
                  "near_obs: %.4f, no_obs: %.4f, ball_obs: %.4f, prob_update: %.4f, publish: %.4f, visualize: %.4f, fps: %i",
                  static_cast<double>(total_proceed_time_acc) / near_obstacle_loop_count / 1000.0,
                  static_cast<double>(prepare_update_time_acc) / near_obstacle_loop_count / 1000.0,
                  static_cast<double>(total_data_processing_time_acc) / near_obstacle_loop_count / 1000.0,
                  static_cast<double>(near_obstacles_processing_time_acc) / near_obstacle_loop_count / 1000.0,
                  static_cast<double>(no_obstacles_processing_time_acc) / near_obstacle_loop_count / 1000.0,
                  static_cast<double>(ball_obstacles_processing_time_acc) / near_obstacle_loop_count / 1000.0,
                  static_cast<double>(probability_update_time_acc) / near_obstacle_loop_count / 1000.0,
                  static_cast<double>(publishing_time_acc) / near_obstacle_loop_count / 1000.0,
                  static_cast<double>(visualization_time_acc) / near_obstacle_loop_count / 1000.0,
                  near_obstacle_loop_count);
        }
        last_near_obstacle_log_time += 1'000'000;
        near_obstacle_loop_count = 0;
        total_proceed_time_acc = 0;
        prepare_update_time_acc = 0;
        total_data_processing_time_acc = 0;
        near_obstacles_processing_time_acc = 0;
        no_obstacles_processing_time_acc = 0;
        ball_obstacles_processing_time_acc = 0;
        probability_update_time_acc = 0;
        publishing_time_acc = 0;
        visualization_time_acc = 0;
    }
}

void NearObstacleTracker::proceed_dummy(int64_t image_time) {
    bool ready_for_action = true;
    prepareUpdate(image_time, ready_for_action);
    visualize();
    last_image_time = image_time;
}

void NearObstacleTracker::visualize() {
    std::vector<rerun::Position2D> centers;
    std::vector<rerun::Vec2D> sizes;
    std::vector<rerun::Color> colors;
    std::vector<rerun::Text> labels;
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            point_2d center = point_2d(x - size / 2.f + 0.5f, y - size / 2.f + 0.5f) * resolution;
            center = center.rotated(-pos.a);
            center = center - pos.point();
            centers.push_back(rerun::Position2D(center.x, center.y));
            sizes.push_back(rerun::Vec2D(resolution / 2, resolution / 2));
            float prob = obstacle_probs(x, y);
            colors.emplace_back(255, prob > 0.5f ? 0 : 255, prob > 0.5f ? 0 : 255,
                                static_cast<uint8_t>(255 * prob));
            labels.emplace_back(rerun::Text(std::to_string(prob)));
        }
    }
    rerun::Boxes2D boxes = rerun::Boxes2D::from_centers_and_sizes(centers, sizes)
                                   .with_colors(colors)
                                   .with_radii(rerun::Radius::scene_units(resolution / 4.f))
                                   .with_labels(labels)
                                   .with_show_labels(false);
    htwk::log_rerun("relative/near_obstacle_tracker", boxes);
}

void NearObstacleTracker::clear() {
    obstacle_probs = Raster<float>(size, size, 0);
    pos = Position{0, 0, 0};
}

}  // namespace htwk