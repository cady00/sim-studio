#pragma once

#include "sim/core/Math.hpp"

namespace sim {

// Rigid-plus-scale placement. Internal scene unit is millimeters, Z-up.
struct Transform {
    Vec3 translation{0, 0, 0};
    Quat rotation = Quat::identity();
    Vec3 scale{1, 1, 1};

    Mat4 matrix() const;
    Vec3 transformPoint(Vec3 p) const;
};

}  // namespace sim
