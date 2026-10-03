#include "sim/core/Math.hpp"

#include "sim/core/Transform.hpp"

namespace sim {

Quat Quat::fromAxisAngle(Vec3 axis, double radians) {
    const Vec3 a = normalize(axis);
    const double s = std::sin(radians * 0.5);
    return {std::cos(radians * 0.5), a.x * s, a.y * s, a.z * s};
}

Quat Quat::fromEulerXYZ(double rx, double ry, double rz) {
    const double c1 = std::cos(rx * 0.5);
    const double c2 = std::cos(ry * 0.5);
    const double c3 = std::cos(rz * 0.5);
    const double s1 = std::sin(rx * 0.5);
    const double s2 = std::sin(ry * 0.5);
    const double s3 = std::sin(rz * 0.5);
    Quat q;
    q.x = s1 * c2 * c3 + c1 * s2 * s3;
    q.y = c1 * s2 * c3 - s1 * c2 * s3;
    q.z = c1 * c2 * s3 + s1 * s2 * c3;
    q.w = c1 * c2 * c3 - s1 * s2 * s3;
    return q.normalized();
}

void Quat::toEulerXYZ(double& rx, double& ry, double& rz) const {
    const Transform probe{{0, 0, 0}, normalized(), {1, 1, 1}};
    const Mat4 m = probe.matrix();
    const double m11 = m(0, 0);
    const double m12 = m(1, 0);
    const double m13 = m(2, 0);
    const double m22 = m(1, 1);
    const double m23 = m(2, 1);
    const double m32 = m(1, 2);
    const double m33 = m(2, 2);
    ry = std::asin(std::clamp(m13, -1.0, 1.0));
    if (std::abs(m13) < 0.9999999) {
        rx = std::atan2(-m23, m33);
        rz = std::atan2(-m12, m11);
    } else {
        rx = std::atan2(m32, m22);
        rz = 0;
    }
}

Quat Quat::normalized() const {
    const double len = std::sqrt(w * w + x * x + y * y + z * z);
    if (len < 1e-15) {
        return identity();
    }
    return {w / len, x / len, y / len, z / len};
}

Vec3 Quat::rotate(Vec3 v) const {
    const Quat r = normalized();
    const Quat p{0, v.x, v.y, v.z};
    const Quat out = r * p * r.conjugate();
    return {out.x, out.y, out.z};
}

Quat operator*(Quat a, Quat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            r(column, row) = a(0, row) * b(column, 0) + a(1, row) * b(column, 1) +
                             a(2, row) * b(column, 2) + a(3, row) * b(column, 3);
        }
    }
    return r;
}

Vec3 transformPoint(const Mat4& m, Vec3 p) {
    const double x = m(0, 0) * p.x + m(1, 0) * p.y + m(2, 0) * p.z + m(3, 0);
    const double y = m(0, 1) * p.x + m(1, 1) * p.y + m(2, 1) * p.z + m(3, 1);
    const double z = m(0, 2) * p.x + m(1, 2) * p.y + m(2, 2) * p.z + m(3, 2);
    const double w = m(0, 3) * p.x + m(1, 3) * p.y + m(2, 3) * p.z + m(3, 3);
    if (std::abs(w) > 1e-12 && std::abs(w - 1.0) > 1e-8) {
        return {x / w, y / w, z / w};
    }
    return {x, y, z};
}

Vec3 transformVector(const Mat4& m, Vec3 v) {
    return {m(0, 0) * v.x + m(1, 0) * v.y + m(2, 0) * v.z,
            m(0, 1) * v.x + m(1, 1) * v.y + m(2, 1) * v.z,
            m(0, 2) * v.x + m(1, 2) * v.y + m(2, 2) * v.z};
}

Vec4 transformPoint4(const Mat4& m, Vec4 p) {
    return {m(0, 0) * p.x + m(1, 0) * p.y + m(2, 0) * p.z + m(3, 0) * p.w,
            m(0, 1) * p.x + m(1, 1) * p.y + m(2, 1) * p.z + m(3, 1) * p.w,
            m(0, 2) * p.x + m(1, 2) * p.y + m(2, 2) * p.z + m(3, 2) * p.w,
            m(0, 3) * p.x + m(1, 3) * p.y + m(2, 3) * p.z + m(3, 3) * p.w};
}

Mat4 inverseAffine(const Mat4& m) {
    const double a00 = m(0, 0);
    const double a01 = m(1, 0);
    const double a02 = m(2, 0);
    const double a10 = m(0, 1);
    const double a11 = m(1, 1);
    const double a12 = m(2, 1);
    const double a20 = m(0, 2);
    const double a21 = m(1, 2);
    const double a22 = m(2, 2);
    const double det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) +
                       a02 * (a10 * a21 - a11 * a20);
    Mat4 r = Mat4::identity();
    if (std::abs(det) < 1e-18) {
        return r;
    }
    const double inv = 1.0 / det;
    r(0, 0) = (a11 * a22 - a12 * a21) * inv;
    r(1, 0) = (a02 * a21 - a01 * a22) * inv;
    r(2, 0) = (a01 * a12 - a02 * a11) * inv;
    r(0, 1) = (a12 * a20 - a10 * a22) * inv;
    r(1, 1) = (a00 * a22 - a02 * a20) * inv;
    r(2, 1) = (a02 * a10 - a00 * a12) * inv;
    r(0, 2) = (a10 * a21 - a11 * a20) * inv;
    r(1, 2) = (a01 * a20 - a00 * a21) * inv;
    r(2, 2) = (a00 * a11 - a01 * a10) * inv;
    const Vec3 t{m(3, 0), m(3, 1), m(3, 2)};
    const Vec3 nt = transformVector(r, -t);
    r(3, 0) = nt.x;
    r(3, 1) = nt.y;
    r(3, 2) = nt.z;
    return r;
}

void Aabb::expand(Vec3 p) {
    if (!valid) {
        min = max = p;
        valid = true;
        return;
    }
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    max.z = std::max(max.z, p.z);
}

void Aabb::expand(const Aabb& other) {
    if (!other.valid) {
        return;
    }
    expand(other.min);
    expand(other.max);
}

Vec3 Aabb::center() const {
    if (!valid) {
        return {};
    }
    return (min + max) * 0.5;
}

Vec3 Aabb::size() const {
    if (!valid) {
        return {};
    }
    return max - min;
}

double Aabb::diagonal() const { return length(size()); }

double distancePointToSegment2D(double px, double py, double ax, double ay, double bx, double by) {
    const double vx = bx - ax;
    const double vy = by - ay;
    const double c2 = vx * vx + vy * vy;
    const double t = c2 > 1e-12 ? std::clamp(((px - ax) * vx + (py - ay) * vy) / c2, 0.0, 1.0) : 0.0;
    const double dx = ax + t * vx - px;
    const double dy = ay + t * vy - py;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace sim
