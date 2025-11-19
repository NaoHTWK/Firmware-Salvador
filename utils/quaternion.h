#pragma once

#include <cmath>
#include <functional>
#include <tuple>

#include "point_3d.h"

class Quaternion {
public:
    float w{1.0f};  // Real part (scalar)
    float x{0.0f};  // Imaginary part (i)
    float y{0.0f};  // Imaginary part (j)
    float z{0.0f};  // Imaginary part (k)

    Quaternion() = default;
    ~Quaternion() = default;
    Quaternion(const Quaternion&) = default;
    Quaternion(Quaternion&&) = default;
    Quaternion& operator=(const Quaternion&) = default;
    Quaternion& operator=(Quaternion&&) = default;

    constexpr Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    // Create quaternion from axis-angle representation
    static Quaternion fromAxisAngle(const point_3d& axis, float angle) {
        float half_angle = angle * 0.5f;
        float s = std::sin(half_angle);

        point_3d normalized_axis = axis;
        float norm = normalized_axis.norm();
        if (norm > 0.0f) {
            normalized_axis /= norm;
        }

        return Quaternion(std::cos(half_angle), normalized_axis.x * s, normalized_axis.y * s,
                          normalized_axis.z * s);
    }

    // Create quaternion from Euler angles (in radians, ZYX convention)
    static Quaternion fromEuler(float roll, float pitch, float yaw) {
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);

        return Quaternion(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                          cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy);
    }

    // Convert to Euler angles (in radians, ZYX convention)
    std::tuple<float, float, float> toEuler() const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        float roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        float pitch;
        if (std::abs(sinp) >= 1.0f) {
            // Use 90 degrees if out of range
            pitch = std::copysign(M_PI / 2.0f, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        float yaw = std::atan2(siny_cosp, cosy_cosp);

        return std::make_tuple(roll, pitch, yaw);
    }

    // Calculate the norm of the quaternion
    float norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // Calculate the squared norm
    float normSquared() const {
        return w * w + x * x + y * y + z * z;
    }

    // Normalize the quaternion
    void normalize() {
        float n = norm();
        if (n > 0.0f) {
            float invN = 1.0f / n;
            w *= invN;
            x *= invN;
            y *= invN;
            z *= invN;
        }
    }

    // Return a normalized copy
    Quaternion normalized() const {
        Quaternion result = *this;
        result.normalize();
        return result;
    }

    // Calculate the conjugate (w, -x, -y, -z)
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Calculate the inverse
    Quaternion inverse() const {
        float n_sq = normSquared();
        if (n_sq < 1e-6f) {
            return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Default to identity if too small
        }

        float inv_n_sq = 1.0f / n_sq;
        return Quaternion(w * inv_n_sq, -x * inv_n_sq, -y * inv_n_sq, -z * inv_n_sq);
    }

    // Rotate a point using the quaternion
    point_3d rotate(const point_3d& p) const {
        // Convert point to quaternion
        Quaternion p_quat(0.0f, p.x, p.y, p.z);

        // q * p * q^-1
        Quaternion rotated = (*this) * p_quat * this->inverse();

        return point_3d(rotated.x, rotated.y, rotated.z);
    }

    // Quaternion multiplication
    Quaternion operator*(const Quaternion& rhs) const {
        return Quaternion(w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
                          w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
                          w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
                          w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
    }

    // Addition
    Quaternion& operator+=(const Quaternion& rhs) {
        w += rhs.w;
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    friend Quaternion operator+(Quaternion lhs, const Quaternion& rhs) {
        lhs += rhs;
        return lhs;
    }

    // Subtraction
    Quaternion& operator-=(const Quaternion& rhs) {
        w -= rhs.w;
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    friend Quaternion operator-(Quaternion lhs, const Quaternion& rhs) {
        lhs -= rhs;
        return lhs;
    }

    // Scalar multiplication
    Quaternion& operator*=(float scalar) {
        w *= scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    friend Quaternion operator*(Quaternion lhs, float rhs) {
        lhs *= rhs;
        return lhs;
    }

    friend Quaternion operator*(float lhs, Quaternion rhs) {
        rhs *= lhs;
        return rhs;
    }

    // Equality comparison
    friend bool operator==(const Quaternion& lhs, const Quaternion& rhs) {
        return std::tie(lhs.w, lhs.x, lhs.y, lhs.z) == std::tie(rhs.w, rhs.x, rhs.y, rhs.z);
    }
};

namespace std {
template <>
struct hash<Quaternion> {
    size_t operator()(const Quaternion& q) const {
        size_t h1 = std::hash<float>{}(q.w);
        size_t h2 = std::hash<float>{}(q.x);
        size_t h3 = std::hash<float>{}(q.y);
        size_t h4 = std::hash<float>{}(q.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
    }
};
}  // namespace std