#pragma once

#include <cmath>
#include <point_2d.h>
#include <point_3d.h>
#include <array>

constexpr int cam_width  = CAMERA_WIDTH;
constexpr int cam_height = CAMERA_HEIGHT;
//Camera intrinsics: width=1280, height=720, fx=645.555481, fy=644.090881, ppx=647.596313, ppy=368.293396
// constexpr float fx = 645.555481f;
// constexpr float fy = 644.090881f;  
// constexpr float ppx = 647.596313f; 
// constexpr float ppy = 368.293396f; 

//constexpr float cam_fov = CAMERA_FOV_WIDTH;
constexpr float cam_fov_width  = CAMERA_FOV_WIDTH;
constexpr float cam_fov_height = CAMERA_FOV_HEIGHT;

inline float cam_tan_fov_2_width = std::tan(cam_fov_width/ 2.f); //1.000003673;  // 0.516129032f;  // tan(fov/2)
inline float cam_tan_fov_2_height = std::tan(cam_fov_height / 2.f);
inline float cam_depth_width = cam_width / 2.f / cam_tan_fov_2_width;
inline float cam_depth_height = cam_height / 2.f / cam_tan_fov_2_height;

#ifdef ROBOT_MODEL_T1

constexpr float torso_length  = 0.41f;
constexpr float foot_height   = 0.045f;

constexpr float upper_cam_pitch = 0.0f;

// Relative position of upper cam to the neck joint
constexpr point_3d cam_neck{.07f, 0, .115f};

#elif ROBOT_MODEL_K1

constexpr float torso_length  = 0.31f;
constexpr float foot_height   = 0.02f;

constexpr float upper_cam_pitch = 0.244f;

// Relative position of upper cam to the neck joint
constexpr point_3d cam_neck{.05f, -0.035f, .09f};

#endif
