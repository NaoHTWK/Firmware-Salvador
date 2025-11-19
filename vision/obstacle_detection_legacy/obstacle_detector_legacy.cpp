#include "obstacle_detector_legacy.h"

#include <cmath>
#include <cstdlib>
#include <iterator>
#include <vector>

#include "bounding_box.h"
#include "cam_pose.h"  // Ensure CamPose is defined
#include "fluxlog_vision.h"
#include "image.h"
#include "logging.h"
#include "point_2d.h"
#include "point_3d.h"
#include "rerun.hpp"
#include "rerun/archetypes/points2d.hpp"
#include "vision_pub_sub.h"

#ifdef ROBOT_MODEL_K1
#define MAX_DISTANCE 4.f
#elif ROBOT_MODEL_T1
#define MAX_DISTANCE 8.f
#endif
#define MIN_DISTANCE 0.05f
#define MIN_HEIGHT 0.20f
#define MAX_HEIGHT 1.0f
#ifdef ROBOT_MODEL_K1
#define MAX_PLANE_DISTANCE 3.f
#define MIN_PLANE_DISTANCE 0.35f
#elif ROBOT_MODEL_T1
#define MAX_PLANE_DISTANCE 4.5f
#define MIN_PLANE_DISTANCE 0.05f
#endif
constexpr int BALL_TOLERANCE = 30;
constexpr int PIXEL_STEP_SIZE = 10;

namespace htwk {

ObstacleDetectorLegacy::ObstacleDetectorLegacy() {
    // Constructor implementation
}

ObstacleDetectorLegacy::~ObstacleDetectorLegacy() {
    // Destructor implementation
}

point_3d pixel_to_world_with_depth(float px, float py, float depth, float fx, float fy, float cx,
                                   float cy) {
    // Create point in camera coordinates
    float cam_x = (px - cx) / fx * depth;
    float cam_y = (py - cy) / fy * depth;
    float cam_z = depth;

    // Transform to world coordinates
    // Camera coordinates: x right, y down, z forward
    // World coordinates: x forward, y left, z up
    // Transformation: world_x = cam_z, world_y = -cam_x, world_z = -cam_y
    point_3d world_point;
    world_point.x = cam_z;
    world_point.y = -cam_x;
    world_point.z = -cam_y;
    return world_point;
}

void ObstacleDetectorLegacy::proceed(const std::shared_ptr<Image>& img, std::vector<float>& plane,
                                     std::vector<BoundingBox> ball_boxes) {
    if (plane.size() != 4) {
        // if no plane is detected dont change
        htwk::log_rerun("image/obstacles");
        near_obstacles_channel.publish(std::make_shared<std::vector<point_2d>>());
        no_near_obstacles_channel.publish(std::make_shared<std::vector<point_2d>>());
        ball_obstacles_channel.publish(std::make_shared<std::vector<point_2d>>());
        return;
    }
    float* depth_image = img->data_depth_m.data();
    bool is_obstacle = false;

    float plane_sqrt = std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
    float camera_height = std::abs(plane[3]) / plane_sqrt;
    std::vector<point_2d> no_obstacles;
    std::vector<rerun::components::Position2D> no_obstacles_rerun;
    std::vector<point_2d> obstacles;
    std::vector<rerun::components::Position2D> obstacles_rerun;
    std::vector<point_2d> ball_obstacles;
    std::vector<rerun::components::Position2D> ball_obstacles_rerun;
    std::vector<rerun::components::Position2D> image_obstacles_rerun;
    // get ball detection

#ifdef ROBOT_MODEL_T1
    // Realsense has lots of noise at the bottom of the image, so we skip the bottom rows
    for (int y = 0; y < 710; y += PIXEL_STEP_SIZE) {
#else
    for (int y = 0; y < img->HEIGHT; y += PIXEL_STEP_SIZE) {
#endif
        for (int x = 0; x < img->WIDTH; x += PIXEL_STEP_SIZE) {

            is_obstacle = false;
            float local_depth = depth_image[y * img->WIDTH + x];
            if (std::isnan(local_depth)) {
                is_obstacle = false;
                continue;
            }

            point_3d pixel_from_depth = pixel_to_world_with_depth(x, y, local_depth, img->fx,
                                                                  img->fy, img->cx, img->cy);
            pixel_from_depth = img->cam_pose_ptr->rotation.multiply(pixel_from_depth);

            // get distance from plane to point
            if (local_depth > MAX_DISTANCE || local_depth < MIN_DISTANCE) {
                is_obstacle = false;
                continue;
            }
            float distance_above_ground = camera_height + pixel_from_depth.z;
            point_2d projection_on_plane =
                    pixel_from_depth.xy().rotated(img->cam_pose_ptr->head_angles.yaw);

            /*if (distance_above_ground > 0.3f) {
                if (local_depth < 0.05f) {
                    is_obstacle = false;
                    continue;
                }
            }*/

            if (distance_above_ground > MAX_HEIGHT) {
                is_obstacle = false;
                continue;
            }

            // here ball logic
            for (BoundingBox ball_box : ball_boxes) {
                if (ball_box.is_inside(x, y, BALL_TOLERANCE)) {
                    ball_obstacles.emplace_back(projection_on_plane.x, projection_on_plane.y);
                    ball_obstacles_rerun.emplace_back(projection_on_plane.x, projection_on_plane.y);
                    continue;
                }
            }

            float plane_distance = std::sqrt(projection_on_plane.x * projection_on_plane.x +
                                             projection_on_plane.y * projection_on_plane.y);

            if (plane_distance < MIN_PLANE_DISTANCE) {
                no_obstacles.emplace_back(projection_on_plane.x, projection_on_plane.y);
                no_obstacles_rerun.emplace_back(projection_on_plane.x, projection_on_plane.y);
                is_obstacle = false;
                continue;
            }

            if (plane_distance > MAX_PLANE_DISTANCE) {
                is_obstacle = false;
                continue;
            }

            if (distance_above_ground < MIN_HEIGHT) {
                no_obstacles.emplace_back(projection_on_plane.x, projection_on_plane.y);
                no_obstacles_rerun.emplace_back(projection_on_plane.x, projection_on_plane.y);

                is_obstacle = false;
                continue;
            }
            obstacles.emplace_back(projection_on_plane.x, projection_on_plane.y);
            obstacles_rerun.emplace_back(projection_on_plane.x, projection_on_plane.y);
            image_obstacles_rerun.emplace_back(x, y);
            is_obstacle = true;
        }
    }

    near_obstacles_channel.publish(std::make_shared<std::vector<point_2d>>(obstacles));
    no_near_obstacles_channel.publish(std::make_shared<std::vector<point_2d>>(no_obstacles));
    ball_obstacles_channel.publish(std::make_shared<std::vector<point_2d>>(ball_obstacles));
    // log obstacles red
    // htwk::log_rerun("relative/no_obstacles",
    //                 rerun::Points2D(no_obstacles_rerun).with_colors(rerun::Color(255, 0,
    //                 0)));
    // htwk::log_rerun("relative/obstacles", rerun::Points2D(obstacles_rerun)
    //                                               .with_colors(rerun::Color(255, 0, 0, 50))
    //                                               .with_radii(0.05f));  // Adjust radius as
    //                                               needed

    // htwk::log_rerun("relative/obstacles", rerun::Points2D(ball_obstacles_rerun)
    //                                               .with_colors(rerun::Color(255, 255, 0, 50))
    //                                               .with_radii(0.05f));  // Adjust radius as
    //                                               needed
    htwk::log_vision_points("image/obstacles", image_obstacles_rerun, 8.f,
                            rerun::Color(255, 0, 0, 110));
}
}  // namespace htwk