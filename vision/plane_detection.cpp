#include "plane_detection.h"

#include <random>

// Convert pixel coordinates to world coordinates given a depth value
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

// Estimate ground plane from depth image using RANSAC
std::vector<float> estimate_plane_ransac(const float* depth_image, int width, int height, float fx,
                                         float fy, float cx, float cy, int num_iterations,
                                         float inlier_threshold) {
    std::vector<float> best_plane(4, 0.0f);
    int max_inliers = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, width * height - 1);
    std::vector<point_3d> test_depths;
    test_depths.reserve((width / 5) * (height / 5));
    for (int y = 0; y < height; y += 5) {
        for (int x = 0; x < width; x += 5) {
            float depth = depth_image[y * width + x];
            if (!(depth > 0.1f && depth < 6.0f))
                continue;
            test_depths.push_back(pixel_to_world_with_depth(x, y, depth, fx, fy, cx, cy));
        }
    }

    for (int i = 0; i < num_iterations; ++i) {
        // 1. Sample 3 random points
        point_3d points[3];
        bool found_valid_points = true;
        for (int j = 0; j < 3; ++j) {
            int px, py;
            float depth;
            int attempts = 0;
            int rand_idx;
            do {
                rand_idx = distrib(gen);
                depth = depth_image[rand_idx];
                attempts++;
            } while (!(depth > 0.1f && depth < 6.0f) && attempts < 100);

            if (attempts >= 100) {
                found_valid_points = false;
                break;
            }
            px = rand_idx % width;
            py = rand_idx / width;
            points[j] = pixel_to_world_with_depth(px, py, depth, fx, fy, cx, cy);
        }
        if (!found_valid_points)
            continue;

        // 2. Calculate plane equation from the 3 points
        point_3d p1 = points[0];
        point_3d p2 = points[1];
        point_3d p3 = points[2];

        point_3d v1 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
        point_3d v2 = {p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

        float a = v1.y * v2.z - v1.z * v2.y;
        float b = v1.z * v2.x - v1.x * v2.z;
        float c = v1.x * v2.y - v1.y * v2.x;

        float norm = std::sqrt(a * a + b * b + c * c);
        if (norm < 1e-6)
            continue;
        norm = 1.0f / norm;

        a *= norm;
        b *= norm;
        c *= norm;
        float d = -(a * p1.x + b * p1.y + c * p1.z);

        int current_inliers = 0;
        for (const auto& p : test_depths) {
            float distance = std::abs(a * p.x + b * p.y + c * p.z + d);
            if (distance < inlier_threshold) {
                current_inliers++;
            }
        }

        if (current_inliers > max_inliers) {
            max_inliers = current_inliers;
            best_plane[0] = a;
            best_plane[1] = b;
            best_plane[2] = c;
            best_plane[3] = d;
        }
    }

    // To ensure the plane normal is pointing "up" in world coordinates (positive
    // Z), we check the z component of the normal vector in world space. The plane
    // normal in the equation ax+by+cz+d=0 is (a,b,c).
    if (best_plane[2] < 0) {
        best_plane[0] *= -1;
        best_plane[1] *= -1;
        best_plane[2] *= -1;
        best_plane[3] *= -1;
    }

    return best_plane;
}
