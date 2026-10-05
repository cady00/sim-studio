#pragma once

#include "sim/collision/Likelihood.hpp"
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
// Up to `limit` contacts. Parallel triangles are kept when their points are spatially apart.
// The walk order is part of the greedy merge, so this function stays exhaustive.
// `satTests`, when set, counts separating-axis tests.
std::vector<NarrowContact> queryBoxMesh(const TriangleBvh& bvh, const Mat4& world, Vec3 boxCenter, Vec3 boxHalf, int limit,
                                       int* satTests = nullptr);

// Last confirmed triangle. -1 means the previous query missed.
struct NarrowMemory {
    int triangle = -1;
};

struct NarrowDetectStats {
    int candidates = 0;
    int tested = 0;
    bool hit = false;
    int triangle = -1;
};

// Same candidate set and the same separating-axis test as `queryBoxMesh`.
// Leaves are ordered by `model` (distance, then the previous hit) and the walk
// returns at the first confirmed overlap. A null model uses distance-and-previous.
// The boolean matches `queryBoxMesh` being non-empty for any positive limit.
bool detectBoxMesh(const TriangleBvh& bvh, const Mat4& world, Vec3 boxCenter, Vec3 boxHalf, const ICollisionLikelihood* model,
                   NarrowMemory& memory, NarrowDetectStats* stats = nullptr);

// Both meshes carry a BVH. Even entries push the first mesh out of the second.
// The following entry is the same point with the opposite normal.
std::vector<NarrowContact> queryMeshMesh(const TriangleBvh& first, const Mat4& firstWorld, const TriangleBvh& second,
                                         const Mat4& secondWorld, int limit);

// Moving AABB against a static AABB. `toi` is seconds in [0, dt]. `normal` pushes the moving box out.
bool sweepAabbToi(Vec3 center, Vec3 half, Vec3 velocity, double dt, Vec3 obstacleCenter, Vec3 obstacleHalf, double& toi,
                  Vec3& normal);

// Moving AABB against a triangle mesh. A thin plate is caught when the discrete step would jump past it.
bool sweepAabbMesh(Vec3 center, Vec3 half, Vec3 velocity, double dt, const TriangleBvh& bvh, const Mat4& world, double& toi,
                   Vec3& normal);

}  // namespace sim
