#include "sim/render/Camera.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 forward = normalize(target - eye);
    Vec3 right = cross(forward, up);
    if (length(right) < 1e-8) {
        const Vec3 fallback = std::abs(forward.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{0, 1, 0};
        right = cross(forward, fallback);
    }
    right = normalize(right);
    const Vec3 trueUp = cross(right, forward);
    Mat4 m = Mat4::identity();
    m(0, 0) = right.x;
    m(1, 0) = right.y;
    m(2, 0) = right.z;
    m(0, 1) = trueUp.x;
    m(1, 1) = trueUp.y;
    m(2, 1) = trueUp.z;
    m(0, 2) = -forward.x;
    m(1, 2) = -forward.y;
    m(2, 2) = -forward.z;
    m(3, 0) = -dot(right, eye);
    m(3, 1) = -dot(trueUp, eye);
    m(3, 2) = dot(forward, eye);
    return m;
}

Mat4 perspective(double fovYDegrees, double aspect, double nearPlane, double farPlane) {
    const double fov = deg2rad(fovYDegrees);
    const double f = 1.0 / std::tan(fov * 0.5);
    Mat4 m{};
    m.m[0] = m.m[5] = m.m[10] = m.m[15] = 0;
    m(0, 0) = f / std::max(aspect, 1e-6);
    m(1, 1) = f;
    m(2, 2) = (farPlane + nearPlane) / (nearPlane - farPlane);
    m(2, 3) = -1.0;
    m(3, 2) = (2.0 * farPlane * nearPlane) / (nearPlane - farPlane);
    m(3, 3) = 0;
    return m;
}

}  // namespace

Vec3 OrbitCamera::eye() const {
    Vec3 direction = eyeDirection;
    if (length(direction) < 1e-8) {
        direction = {0.72, -0.62, 0.55};
    }
    return target + normalize(direction) * distance;
}

void OrbitCamera::basis(Vec3& right, Vec3& up, Vec3& forward) const {
    forward = normalize(target - eye());
    Vec3 hint = upHint;
    if (std::abs(dot(forward, hint)) > 0.98) {
        hint = std::abs(hint.z) > 0.5 ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
    }
    right = normalize(cross(forward, hint));
    up = cross(right, forward);
}

Mat4 OrbitCamera::viewMatrix() const {
    Vec3 right, up, forward;
    basis(right, up, forward);
    (void)right;
    (void)up;
    (void)forward;
    Vec3 hint = upHint;
    const Vec3 toTarget = normalize(target - eye());
    if (std::abs(dot(toTarget, hint)) > 0.98) {
        hint = std::abs(hint.z) > 0.5 ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
    }
    return lookAt(eye(), target, hint);
}

Mat4 OrbitCamera::projectionMatrix(double aspect) const {
    const double nearPlane = std::clamp(distance * 0.002, 0.05, 50.0);
    const double farPlane = std::max(distance * 40.0, nearPlane + 100.0);
    return perspective(fovYDegrees, aspect, nearPlane, farPlane);
}

Ray OrbitCamera::rayFromPixel(double x, double y, int width, int height) const {
    Vec3 right, up, forward;
    basis(right, up, forward);
    const double aspect = height > 0 ? static_cast<double>(width) / static_cast<double>(height) : 1.0;
    const double ndcX = width > 0 ? (2.0 * x / static_cast<double>(width) - 1.0) : 0;
    const double ndcY = height > 0 ? (1.0 - 2.0 * y / static_cast<double>(height)) : 0;
    const double tanHalf = std::tan(deg2rad(fovYDegrees) * 0.5);
    const Vec3 direction = normalize(right * (ndcX * aspect * tanHalf) + up * (ndcY * tanHalf) + forward);
    return Ray{eye(), direction};
}

Vec3 OrbitCamera::project(Vec3 world, double aspect, int width, int height) const {
    const Vec4 clip = transformPoint4(mul(projectionMatrix(aspect), viewMatrix()), Vec4{world.x, world.y, world.z, 1});
    if (std::abs(clip.w) < 1e-8) {
        return {};
    }
    const double ndcX = clip.x / clip.w;
    const double ndcY = clip.y / clip.w;
    return {(ndcX * 0.5 + 0.5) * width, (1.0 - (ndcY * 0.5 + 0.5)) * height, clip.z / clip.w};
}

void OrbitCamera::orbit(double dxPixels, double dyPixels) {
    Vec3 direction = normalize(eyeDirection);
    double pitch = std::asin(std::clamp(direction.z, -1.0, 1.0));
    double yaw = std::atan2(direction.x, -direction.y);
    yaw -= dxPixels * 0.008;
    pitch += dyPixels * 0.008;
    pitch = std::clamp(pitch, -1.45, 1.45);
    eyeDirection = {std::cos(pitch) * std::sin(yaw), -std::cos(pitch) * std::cos(yaw), std::sin(pitch)};
    upHint = {0, 0, 1};
}

void OrbitCamera::pan(double dxPixels, double dyPixels) {
    Vec3 right, up, forward;
    basis(right, up, forward);
    (void)forward;
    const double scale = distance * 0.0016;
    target = target - right * (dxPixels * scale) - up * (dyPixels * scale);
}

void OrbitCamera::zoomWheel(double notches) {
    distance *= std::pow(0.88, notches);
    distance = std::clamp(distance, 1.0, 1.0e7);
}

void OrbitCamera::setStandardView(StandardView view) {
    upHint = {0, 0, 1};
    switch (view) {
        case StandardView::Front:
            eyeDirection = {0, -1, 0};
            break;
        case StandardView::Back:
            eyeDirection = {0, 1, 0};
            break;
        case StandardView::Left:
            eyeDirection = {-1, 0, 0};
            break;
        case StandardView::Right:
            eyeDirection = {1, 0, 0};
            break;
        case StandardView::Top:
            eyeDirection = {0, 0, 1};
            upHint = {0, 1, 0};
            break;
        case StandardView::Bottom:
            eyeDirection = {0, 0, -1};
            upHint = {0, -1, 0};
            break;
        case StandardView::Iso:
            eyeDirection = {0.72, -0.62, 0.55};
            break;
    }
}

void OrbitCamera::fit(const Aabb& bounds, double aspect, double margin) {
    if (!bounds.valid) {
        target = {};
        distance = 800;
        setStandardView(StandardView::Iso);
        return;
    }
    target = bounds.center();
    const double radius = std::max(bounds.diagonal() * 0.5, 1.0);
    const double fov = deg2rad(fovYDegrees);
    const double vertical = radius / std::max(std::sin(fov * 0.5), 1e-3);
    const double horizontal = radius / std::max(std::sin(fov * 0.5) * std::max(aspect, 0.2), 1e-3);
    distance = std::max(vertical, horizontal) * std::max(margin, 1.0);
    distance = std::clamp(distance, 1.0, 1.0e7);
}

}  // namespace sim
