#include "sim/core/Transform.hpp"

namespace sim {

Mat4 Transform::matrix() const {
    const Quat q = rotation.normalized();
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;

    Mat4 m = Mat4::identity();
    m(0, 0) = (1.0 - 2.0 * (yy + zz)) * scale.x;
    m(0, 1) = (2.0 * (xy + wz)) * scale.x;
    m(0, 2) = (2.0 * (xz - wy)) * scale.x;
    m(1, 0) = (2.0 * (xy - wz)) * scale.y;
    m(1, 1) = (1.0 - 2.0 * (xx + zz)) * scale.y;
    m(1, 2) = (2.0 * (yz + wx)) * scale.y;
    m(2, 0) = (2.0 * (xz + wy)) * scale.z;
    m(2, 1) = (2.0 * (yz - wx)) * scale.z;
    m(2, 2) = (1.0 - 2.0 * (xx + yy)) * scale.z;
    m(3, 0) = translation.x;
    m(3, 1) = translation.y;
    m(3, 2) = translation.z;
    return m;
}

Vec3 Transform::transformPoint(Vec3 p) const { return sim::transformPoint(matrix(), p); }

}  // namespace sim
