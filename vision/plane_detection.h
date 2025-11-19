#pragma once

#include <vector>

#include "point_3d.h"

// Convert pixel coordinates to world coordinates given a depth value
point_3d pixel_to_world_with_depth(float px, float py, float depth, float fx, float fy, float cx,
                                   float cy);

// Estimate ground plane from depth image using RANSAC
std::vector<float> estimate_plane_ransac(const float* depth_image, int width, int height, float fx,
                                         float fy, float cx, float cy, int num_iterations = 100,
                                         float inlier_threshold = 0.05);