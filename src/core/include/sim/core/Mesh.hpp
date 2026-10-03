#pragma once

#include "sim/core/Math.hpp"

#include <cstdint>
#include <vector>

namespace sim {

struct MeshData {
    std::vector<float> positions;  // xyz
    std::vector<float> normals;    // xyz, one per vertex
    std::vector<std::uint32_t> indices;
    Aabb bounds;

    bool empty() const { return indices.size() < 3 || positions.size() < 9; }
    std::size_t vertexCount() const { return positions.size() / 3; }
    std::size_t triangleCount() const { return indices.size() / 3; }
    void recomputeBounds();
};

}  // namespace sim
