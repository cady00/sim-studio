#pragma once

#include "sim/core/Scene.hpp"

#include <optional>
#include <string>

namespace sim {

struct Ray {
    Vec3 origin;
    Vec3 direction;  // unit length
};

struct PickHit {
    std::string nodeId;
    double distance = 0;
    Vec3 point;
    Vec3 normal{0, 0, 1};
};

// Closest visible triangle along the ray. Empty if nothing is hit.
std::optional<PickHit> pickScene(const Scene& scene, const Ray& ray);

}  // namespace sim
