#pragma once

#include "sim/core/Math.hpp"
#include "sim/core/Pick.hpp"

namespace sim {

enum class StandardView { Front, Back, Left, Right, Top, Bottom, Iso };

// Z-up orbit camera. Middle-drag orbits, Shift+middle pans, wheel zooms.
class OrbitCamera {
public:
    Vec3 target{0, 0, 0};
    double distance = 800;
    double fovYDegrees = 35;
    Vec3 eyeDirection{0.72, -0.62, 0.55};
    Vec3 upHint{0, 0, 1};

    Vec3 eye() const;
    void basis(Vec3& right, Vec3& up, Vec3& forward) const;
    Mat4 viewMatrix() const;
    Mat4 projectionMatrix(double aspect) const;
    Ray rayFromPixel(double x, double y, int width, int height) const;
    Vec3 project(Vec3 world, double aspect, int width, int height) const;

    void orbit(double dxPixels, double dyPixels);
    void pan(double dxPixels, double dyPixels);
    void zoomWheel(double notches);
    void setStandardView(StandardView view);
    // margin scales the bounding-sphere distance. 1.25 is the demo framing.
    void fit(const Aabb& bounds, double aspect, double margin = 1.25);
};

}  // namespace sim
