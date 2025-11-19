#include "point_2d.h"

#include <cmath>

#include "imu.h"

using namespace std;

YawPitch getYawPitch(const point_2d& rel, float cam_translation_z) {
    return {atan2(rel.y, rel.x), atan(rel.norm() / cam_translation_z)};
}

float point_2d::angular_dist(const point_2d& other, float cam_translation_z) const {
    YawPitch p0 = getYawPitch(*this, cam_translation_z);
    YawPitch p1 = getYawPitch(other, cam_translation_z);
    return sqrt(pow(angleDiff(p0.pitch, p1.pitch), 2.f) + pow(angleDiff(p0.yaw, p1.yaw), 2.f));
}
