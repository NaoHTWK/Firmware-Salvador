#include "quaternion.h"

// Example of how to combine two rotations using quaternions
point_3d rotateWithQuaternions(const point_3d& point, float roll1, float pitch1, float yaw1,
                               float roll2, float pitch2, float yaw2) {
    // Create quaternions from Euler angles
    Quaternion q1 = Quaternion::fromEuler(roll1, pitch1, yaw1);
    Quaternion q2 = Quaternion::fromEuler(roll2, pitch2, yaw2);

    // Combine rotations (q2 is applied first, then q1)
    Quaternion combined = q1 * q2;

    // Apply the combined rotation to the point
    return combined.rotate(point);
}
