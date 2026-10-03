#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace sim {

struct Vec3 {
    double x = 0;
    double y = 0;
    double z = 0;

    Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
};

inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }

inline Vec3 normalize(Vec3 a) {
    const double len = length(a);
    if (len < 1e-15) {
        return {};
    }
    return a / len;
}

struct Vec4 {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 1;
};

struct Quat {
    double w = 1;
    double x = 0;
    double y = 0;
    double z = 0;

    static Quat identity() { return {}; }
    static Quat fromAxisAngle(Vec3 axis, double radians);
    // Intrinsic Tait-Bryan XYZ, radians: X, then Y, then Z.
    static Quat fromEulerXYZ(double rx, double ry, double rz);
    void toEulerXYZ(double& rx, double& ry, double& rz) const;
    Quat normalized() const;
    Quat conjugate() const { return {w, -x, -y, -z}; }
    Vec3 rotate(Vec3 v) const;
};

Quat operator*(Quat a, Quat b);

// Column-major 4x4. Index (column, row).
struct Mat4 {
    double m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static Mat4 identity() { return {}; }
    double& operator()(int column, int row) { return m[column * 4 + row]; }
    double operator()(int column, int row) const { return m[column * 4 + row]; }
};

Mat4 mul(const Mat4& a, const Mat4& b);
Vec3 transformPoint(const Mat4& m, Vec3 p);
Vec3 transformVector(const Mat4& m, Vec3 v);
Vec4 transformPoint4(const Mat4& m, Vec4 p);
Mat4 inverseAffine(const Mat4& m);

struct Aabb {
    Vec3 min{0, 0, 0};
    Vec3 max{0, 0, 0};
    bool valid = false;

    void expand(Vec3 p);
    void expand(const Aabb& other);
    Vec3 center() const;
    Vec3 size() const;
    double diagonal() const;
};

double distancePointToSegment2D(double px, double py, double ax, double ay, double bx, double by);

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

}  // namespace sim
