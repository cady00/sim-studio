#pragma once

#include "sim/collision/Likelihood.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/core/Scene.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sim {

// Engine-independent convex. Scene queries build these from meshes or Mech shapes.
// Jolt is not used here, so a later GPU or neural backend can replace the CPU query
// without taking a physics-engine type.
struct ConvexShape {
    enum class Kind { Sphere, Box, Capsule, Hull };

    Kind kind = Kind::Sphere;
    Vec3 center{0, 0, 0};
    Quat rotation = Quat::identity();
    Vec3 half{1, 1, 1};
    double radius = 1;
    double height = 2;
    Vec3 axis{0, 0, 1};
    // Hull vertices in the shape's local frame. World point is center + rotation * p,
    // unless `useMatrix` is set (scene meshes with scale).
    std::vector<Vec3> localPoints;
    bool useMatrix = false;
    Mat4 world = Mat4::identity();
    Vec3 localCenter{0, 0, 0};
    Vec3 localHalf{1, 1, 1};
};

enum class HitKind { Overlap, Clearance, Separation };

struct ContactHit {
    std::string a;
    std::string b;
    HitKind kind = HitKind::Separation;
    // Signed surface gap in millimeters. Negative means penetration.
    double distance = 0;
    Vec3 pointA{0, 0, 0};
    Vec3 pointB{0, 0, 0};
    Vec3 normal{1, 0, 0};
    double penetration = 0;
    double volume = 0;
    // True unless the volume is the closed-form sphere-sphere intersection.
    bool volumeEstimated = true;
};

struct DistanceResult {
    bool ok = false;
    double distance = 0;
    Vec3 pointA{0, 0, 0};
    Vec3 pointB{0, 0, 0};
    Vec3 normal{1, 0, 0};
    // Unit direction of increasing distance for shape A. From the narrow-band SDF when
    // `gradientFromSdf` is set, otherwise the GJK separating direction.
    Vec3 gradient{1, 0, 0};
    bool gradientFromSdf = false;
    std::string error;
};

struct SweepHit {
    bool hit = false;
    // 0 at the start pose, 1 at the end pose.
    double time = 1;
    std::string a;
    std::string b;
    Vec3 point{0, 0, 0};
    std::string error;
};

// Pair of convexes. Separated distance is GJK. Penetration depth is the minimum
// supporting-plane offset of the Minkowski difference (the quantity EPA would
// return). The CPU path samples directions; it does not expand an EPA polytope,
// because a simplex edge through the origin makes that expansion stall.
// Sphere-sphere volume is exact; every other volume is a coarse voxel estimate
// and `volumeEstimated` is true.
ContactHit queryConvex(const ConvexShape& a, const ConvexShape& b);

// Translate `moving.center` from its current value to `targetCenter`.
// Pure translation of a convex against a convex uses conservative advancement,
// so a thin obstacle between the endpoints is reported even when those endpoints miss.
SweepHit sweepConvex(const ConvexShape& moving, Vec3 targetCenter, const ConvexShape& fixed);

// V-HACD-style voxel split written here. This is not CoACD or V-HACD.
// A rectangular box mesh becomes one hull. The grid is at most 16^3.
std::vector<std::vector<Vec3>> decomposeMesh(const MeshData& mesh);
std::uint64_t meshFingerprint(const MeshData& mesh);

// Dense grid, narrow-band accuracy: |distance| is clamped to `band`.
// A later sparse brick can replace `values` without changing sampleSdf.
struct SdfGrid {
    Vec3 origin{0, 0, 0};
    double cell = 1;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    double band = 1;
    std::vector<float> values;
};

SdfGrid buildSdf(const std::vector<ConvexShape>& pieces, int resolution);
bool sampleSdf(const SdfGrid& grid, Vec3 point, double& distance, Vec3& gradient);

class ICollisionQuery {
public:
    virtual ~ICollisionQuery() = default;
    virtual const char* backendName() const = 0;
    virtual void rebuild(const Scene& scene) = 0;
    virtual void refit(const Scene& scene) = 0;
    virtual std::vector<ContactHit> query(const Scene& scene, double clearanceMm) = 0;
    virtual DistanceResult minDistance(const Scene& scene, const std::string& a, const std::string& b) = 0;
    // `from` and `to` are the node's local translation in millimeters. The node is restored.
    virtual SweepHit sweep(Scene& scene, const std::string& name, Vec3 from, Vec3 to) = 0;
    virtual void commitCache(Scene& scene) const = 0;
    // True when the exhaustive contact query would return at least one hit.
    // Candidates are ordered by likelihood and the walk stops at the first confirmed hit.
    virtual bool overlaps(const Scene& scene, double clearanceMm) = 0;
    // Empty pointer restores the distance-and-previous heuristic.
    virtual void setLikelihood(std::shared_ptr<ICollisionLikelihood> model) = 0;
};

// Empty seam. A later GPU LBVH backend would write the same SoA node arrays
// (minX/minY/minZ/maxX/maxY/maxZ, left, right, parent, prim) the CPU BVH uses.
// There is no factory and no implementation in this tree.
class IGpuBroadphase {
public:
    virtual ~IGpuBroadphase() = default;
    virtual const char* backendName() const = 0;
    virtual bool refit(float* minX, float* minY, float* minZ, float* maxX, float* maxY, float* maxZ, int count,
                       std::string& error) = 0;
};

// Empty seam for a learned / neural distance field. No implementation is shipped.
class ILearnedDistanceField {
public:
    virtual ~ILearnedDistanceField() = default;
    virtual const char* backendName() const = 0;
    virtual bool sample(Vec3 point, double& distance, Vec3& gradient, std::string& error) = 0;
};

std::unique_ptr<ICollisionQuery> createCollisionQuery();

// Overlap paints red, near-miss paints yellow. A part in both stays red.
void paintProximity(Scene& scene, const std::vector<ContactHit>& hits);

}  // namespace sim
