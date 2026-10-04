#pragma once

#include "sim/core/Math.hpp"

#include <cstdint>
#include <memory>
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

std::shared_ptr<MeshData> makeBoxMesh(Vec3 halfExtents);
// A grid of `triangles` (rounded up to an even count) inside a square of `size` millimeters.
std::shared_ptr<MeshData> makeTriangleGrid(int triangles, double size);
// Vertex clustering. Larger cells drop more triangles. The result is a coarser LOD.
std::shared_ptr<MeshData> simplifyMesh(const MeshData& source, double cell);

}  // namespace sim
