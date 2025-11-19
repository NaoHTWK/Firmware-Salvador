#pragma once

#include "imu.h"
#include "line.h"
#include "point_2d.h"
#include "point_3d.h"

struct Matrix3x3 {
    float m[3][3];

    point_3d multiply(const point_3d& p) const {
        return {m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z,
                m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z,
                m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z};
    }
};

class CamPose {
public:
    CamPose(float leg_height, YPR body_angles, YawPitch head_angles, float height, float fx,
            float fy, float cx, float cy);

    CamPose() = default;
    CamPose(const CamPose& other) = default;
    CamPose(CamPose&& other) = default;
    CamPose& operator=(const CamPose&) = default;
    CamPose& operator=(CamPose&&) = default;

    void setEllipseAngles(PitchRoll angles) {
        ellipse_angles = angles;
    }
    void removeEllipseAngles() {
        ellipse_angles = std::nullopt;
    }
    const point_3d& get_translation() const {
        return translation;
    }

    float get_fx() const {
        return fx;
    }
    float get_fy() const {
        return fy;
    }
    float get_cx() const {
        return cx;
    }
    float get_cy() const {
        return cy;
    }

    void visualize();

    YawPitch head_angles;
    std::vector<float> plane;
    Matrix3x3 rotation;

private:
    void calcCamTranslation();

    float leg_height = 0;
    YPR body_angles;
    std::optional<PitchRoll> ellipse_angles;
    point_3d translation;
    float height;
    float fx;
    float fy;
    float cx;
    float cy;

    friend class CamUtils;
    friend class VisionLogger;
};

class CamUtils {
public:
    static std::optional<point_2d> project(const point_2d& p_, float height,
                                           const CamPose& cam_pose);
    static std::optional<point_2d> project(const point_2d& p_, const CamPose& cam_pose);
    static std::optional<htwk::Line> project(const htwk::Line& l, const CamPose& cam_pose);
    // Transforms a relative point into a cam-centered coordinate system with x towards the depth
    // and z up (i.e. -y in image coordinates).
    static point_3d relToCam(point_3d p, const CamPose& cam_pose);
    static point_3d relToCam(const point_2d& p, const CamPose& cam_pose);
    static std::optional<point_2d> camToImage(point_3d p);
    static std::optional<point_3d> pixelToPlane(float px, float py, const std::vector<float>& plane,
                                                float fx, float fy, float cx, float cy,
                                                float height);
    static std::optional<point_2d> projectWithDepth(float px, float py, float depth,
                                                    const CamPose& cam_pose);
    static Matrix3x3 planeToHorizontalRotation(const std::vector<float>& plane);
    // Project points in a coordinate system with (0,0) in the center of the neck joint into the
    // image.
    static std::optional<point_2d> neckToCam(point_3d p, const YawPitch& head_pos);
};
