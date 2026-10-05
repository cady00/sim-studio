#pragma once

#include "sim/core/Math.hpp"
#include "sim/core/Mesh.hpp"

#include <vector>

namespace sim {

// One point of a contact manifold. `normal` pushes the query box out of the triangle.
struct NarrowContact {
    Vec3 normal{0, 0, 1};
    Vec3 point{0, 0, 0};
    double depth = 0;
};

// Binary tree over mesh triangles in the mesh's local frame.
struct TriangleBvh {
    struct Node {
        Vec3 bmin{0, 0, 0};
        Vec3 bmax{0, 0, 0};
        int left = -1;
        int right = -1;
        int triangle = -1;
    };
    std::vector<Vec3> triangles;
    std::vector<Node> nodes;
    int root = -1;
};

TriangleBvh buildTriangleBvh(const MeshData& mesh);

// World-space axis-aligned box against a mesh. `world` maps mesh-local points into the scene.
// At most `limit` contacts are returned, clustered so nearly parallel triangles share one point.
std::vector<NarrowContact> queryBoxMesh(const TriangleBvh& bvh, const Mat4& world, Vec3 boxCenter, Vec3 boxHalf, int limit);

}  // namespace sim
