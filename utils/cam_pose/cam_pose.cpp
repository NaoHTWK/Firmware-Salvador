#include "cam_pose.h"

#include "cam_constants.h"
#include "logging.h"
#include "point_3d.h"
#include "quaternion.h"
#include "rerun.hpp"

namespace {

rerun::Quaternion toRerun(const Quaternion& q) {
    return rerun::Quaternion::from_xyzw(q.w, q.x, q.y, q.z);
}

}  // namespace

CamPose::CamPose(float leg_height, YPR body_angles, YawPitch head_angles, float height, float fx,
                 float fy, float cx, float cy)
    : head_angles(head_angles),
      leg_height(leg_height),
      body_angles(body_angles),
      height(height),
      fx(fx),
      fy(fy),
      cx(cx),
      cy(cy) {
    calcCamTranslation();
}

void CamPose::calcCamTranslation() {
    // Go straight up from the ground to the hip.
    translation = point_3d(0, 0, leg_height + foot_height);

    // Add the torso which is rotated by body_angles.
    translation +=
            point_3d(0, 0, torso_length).rotated_y(body_angles.pitch).rotated_x(body_angles.roll);

    // Add the vector to the cam, rotated by body_angles and head_Cangles.
    translation += (cam_neck)
                           .rotated_y(head_angles.pitch)
                           .rotated_z(head_angles.yaw)
                           .rotated_y(body_angles.pitch)
                           .rotated_x(body_angles.roll);
}

void CamPose::visualize() {
    std::vector<rerun::Position3D> translations;
    // Go straight up from the ground to the hip.
    translations.push_back(rerun::Position3D(0, 0, 0));
    translation = point_3d(0, 0, leg_height + foot_height);
    translations.push_back(rerun::Position3D(translation.x, translation.y, translation.z));

    // Add the torso which is rotated by body_angles.
    translation +=
            point_3d(0, 0, torso_length).rotated_y(body_angles.pitch).rotated_x(body_angles.roll);
    translations.push_back(rerun::Position3D(translation.x, translation.y, translation.z));

    // Add the vector to the cam, rotated by body_angles and head_Cangles.
    translation += (cam_neck)
                           .rotated_y(head_angles.pitch)
                           .rotated_z(head_angles.yaw)
                           .rotated_y(body_angles.pitch)
                           .rotated_x(body_angles.roll);

    //     htwk::log_rerun("localization/cam_pose",
    //   rerun::Points3D(translations).with_radii({0.025f}));

    // Add rotation using quaternion
    if (ellipse_angles) {
        // Create quaternion for ellipse angles and head yaw
        Quaternion ellipse_quat =
                Quaternion::fromEuler(ellipse_angles->roll, ellipse_angles->pitch, head_angles.yaw);

        htwk::log_rerun(
                "relative/cam_pose/cam",
                rerun::Transform3D::from_translation({translation.x, translation.y, translation.z})
                        .with_quaternion(toRerun(ellipse_quat)));
    } else {
        // Use body angles for rotation
        Quaternion head_rot =
                Quaternion::fromEuler(head_angles.yaw, -(head_angles.pitch + upper_cam_pitch), 0.f);
        Quaternion body_rot = Quaternion::fromEuler(body_angles.roll, -body_angles.pitch, 0.f);

        Quaternion combined = body_rot * head_rot;

        htwk::log_rerun(
                "relative/cam_pose/cam",
                rerun::Transform3D::from_translation({translation.x, translation.y, translation.z})
                        .with_quaternion(toRerun(combined)));
    }

    // todo: use cam_fov_width and cam_fov_height from utils/cam_constants.h
    htwk::log_rerun("relative/cam_pose/cam/image",
                    rerun::Pinhole::from_fov_and_aspect_ratio(cam_fov_height,
                                                              (float)cam_width / (float)cam_height)
                            .with_camera_xyz(rerun::components::ViewCoordinates::FRD));
}

std::optional<point_2d> CamUtils::project(const point_2d& p_, const CamPose& cam_pose) {
    return project(p_, 0, cam_pose);
}

std::optional<point_2d> CamUtils::project(const point_2d& p_, float height,
                                          const CamPose& cam_pose) {
    // TODO: add height stuff
    if (cam_pose.plane.empty())
        return std::nullopt;
    if (auto p = CamUtils::pixelToPlane(p_.x, p_.y, cam_pose.plane, cam_pose.fx, cam_pose.fy,
                                        cam_pose.cx, cam_pose.cy, height)) {
        // TODO: Properly account for pitch-related changes to the position.
        return (cam_pose.rotation.multiply(*p).xy() - cam_neck.xy())
                .rotated(cam_pose.head_angles.yaw);
    }
    return std::nullopt;
}

std::optional<htwk::Line> CamUtils::project(const htwk::Line& l, const CamPose& cam_pose) {
    if (auto p1 = project(l.p1(), cam_pose)) {
        if (auto p2 = project(l.p2(), cam_pose)) {
            return htwk::Line(*p1, *p2);
        }
    }
    return {};
}

// Convert pixel coordinates to world coordinates by intersecting camera ray
// with ground plane
std::optional<point_3d> CamUtils::pixelToPlane(float px, float py, const std::vector<float>& plane,
                                               float fx, float fy, float cx, float cy,
                                               float height) {
    if (plane.size() < 4) {
        return std::nullopt;  // Invalid plane
    }

    // Create ray direction in camera coordinates (normalized)
    point_3d cam_dir((px - cx) / fx, (py - cy) / fy, 1.0f);

    // Transform ray direction to world coordinates
    // Camera coordinates: x right, y down, z forward
    // World coordinates: x forward, y left, z up
    // Transformation: world_x = cam_z, world_y = -cam_x, world_z = -cam_y
    point_3d world_dir(cam_dir.z, -cam_dir.x, -cam_dir.y);

    // Plane equation: ax + by + cz + d = 0
    float a = plane[0];
    float b = plane[1];
    float c = plane[2];
    float d = plane[3] - c * height;

    // Ray-plane intersection: P = t * direction
    // Substituting into plane equation: a*(t*dir_x) + b*(t*dir_y) + c*(t*dir_z) +
    // d = 0
    float denominator = a * world_dir.x + b * world_dir.y + c * world_dir.z;

    if (std::abs(denominator) < 1e-6) {
        // Ray is parallel to plane
        return std::nullopt;
    }

    float t = -d / denominator;

    if (t <= 0) {
        // Intersection is behind the camera
        return std::nullopt;
    }

    // Calculate intersection point
    return world_dir * t;
}

// Convert pixel coordinates to world coordinates by known depth
std::optional<point_2d> CamUtils::projectWithDepth(float px, float py, float depth,
                                                   const CamPose& cam_pose) {
    if (depth <= 0) {
        return std::nullopt;  // Invalid depth
    }
    point_3d cam_dir((px - cam_pose.cx) / cam_pose.fx, (py - cam_pose.cy) / cam_pose.fy, 1.0f);
    point_3d world_dir(cam_dir.z, -cam_dir.x, -cam_dir.y);
    point_3d world_point = world_dir * depth;
    return (cam_pose.rotation.multiply(world_point).xy() - cam_neck.xy())
            .rotated(cam_pose.head_angles.yaw);
}

Matrix3x3 CamUtils::planeToHorizontalRotation(const std::vector<float>& plane) {
    Matrix3x3 identity = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

    if (plane.size() < 4)
        return identity;

    // Get plane normal and normalize it
    point_3d normal(plane[0], plane[1], plane[2]);
    if (normal.norm() < 1e-6)
        return identity;
    normal.normalize();

    point_3d target_normal(0, 0, 1);

    // If already aligned, return identity
    if (std::abs(normal.x - target_normal.x) < 1e-6 &&
        std::abs(normal.y - target_normal.y) < 1e-6 &&
        std::abs(normal.z - target_normal.z) < 1e-6) {
        return identity;
    }

    // Calculate rotation axis (cross product of current normal and target normal)
    point_3d axis = normal.cross(target_normal);

    float axis_length = axis.norm();

    if (axis_length < 1e-6) {
        // Vectors are parallel or anti-parallel
        if (axis.z < 0) {
            // Flip the normal if it's pointing downward
            return {{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};
        }
        return identity;
    }

    axis /= axis_length;

    // Calculate rotation angle
    float cos_angle = normal.dot(target_normal);
    cos_angle = std::clamp(cos_angle, -1.0f, 1.0f);
    float angle = std::acos(cos_angle);

    // Rodrigues' rotation formula
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    float one_minus_cos = 1.0f - cos_a;

    Matrix3x3 rotation;
    rotation.m[0][0] = cos_a + axis.x * axis.x * one_minus_cos;
    rotation.m[0][1] = axis.x * axis.y * one_minus_cos - axis.z * sin_a;
    rotation.m[0][2] = axis.x * axis.z * one_minus_cos + axis.y * sin_a;

    rotation.m[1][0] = axis.y * axis.x * one_minus_cos + axis.z * sin_a;
    rotation.m[1][1] = cos_a + axis.y * axis.y * one_minus_cos;
    rotation.m[1][2] = axis.y * axis.z * one_minus_cos - axis.x * sin_a;

    rotation.m[2][0] = axis.z * axis.x * one_minus_cos - axis.y * sin_a;
    rotation.m[2][1] = axis.z * axis.y * one_minus_cos + axis.x * sin_a;
    rotation.m[2][2] = cos_a + axis.z * axis.z * one_minus_cos;

    return rotation;
}

point_3d CamUtils::relToCam(point_3d p, const CamPose& cam_pose) {
    p -= cam_pose.translation;
    if (cam_pose.ellipse_angles)
        p = p.rotated_z(-cam_pose.head_angles.yaw)
                    .rotated_x(-cam_pose.ellipse_angles->roll)
                    .rotated_y(-cam_pose.ellipse_angles->pitch);
    else
        p = p.rotated_x(-cam_pose.body_angles.roll)
                    .rotated_y(-cam_pose.body_angles.pitch)
                    .rotated_z(-cam_pose.head_angles.yaw)
                    .rotated_y(-cam_pose.head_angles.pitch - upper_cam_pitch)
                    .rotated_x(0);
    return p;
}

point_3d CamUtils::relToCam(const point_2d& p, const CamPose& cam_pose) {
    return relToCam({p.x, p.y, 0}, cam_pose);
}

std::optional<point_2d> CamUtils::camToImage(point_3d p) {
    if (p.x <= 0)
        return {};
    p *= cam_depth_width / p.x;
    return point_2d{-p.y + cam_width / 2, -p.z + cam_height / 2};
}

std::optional<point_2d> CamUtils::neckToCam(point_3d p, const YawPitch& head_pos) {
    point_3d cam_offset_to_neck = cam_neck.rotated_y(head_pos.pitch).rotated_z(head_pos.yaw);
    p = p - cam_offset_to_neck;
    p = p.rotated_x(0).rotated_z(-head_pos.yaw).rotated_y(-(upper_cam_pitch)-head_pos.pitch);
    // p is behind/inside the camera.
    if (p.x <= 0)
        return {};
    return point_2d(-p.y, -p.z) * (cam_depth_width / p.x) + point_2d(cam_width / 2, cam_height / 2);
}
