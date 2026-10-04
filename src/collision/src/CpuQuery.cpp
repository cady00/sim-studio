#include "sim/collision/Query.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace sim {
namespace {

struct Solid {
    ConvexShape local;
};

struct Part {
    std::string name;
    std::string id;
    std::string meshId;
    std::weak_ptr<SceneNode> node;
    bool primitive = false;
    const MeshData* mesh = nullptr;
    Mat4 world = Mat4::identity();
    std::vector<Solid> solids;
    std::vector<ConvexShape> worldShapes;
    Aabb bounds;
};

Vec3 representative(const ConvexShape& shape) {
    if (shape.useMatrix && shape.kind == ConvexShape::Kind::Box) {
        return transformPoint(shape.world, shape.localCenter);
    }
    if (shape.useMatrix && !shape.localPoints.empty()) {
        return transformPoint(shape.world, shape.localPoints.front());
    }
    return shape.center;
}

void expandShape(Aabb& box, const ConvexShape& shape) {
    if (shape.kind == ConvexShape::Kind::Sphere || shape.kind == ConvexShape::Kind::Capsule) {
        const double reach = shape.radius + std::max(0.0, shape.height * 0.5);
        const Vec3 c = representative(shape);
        box.expand(c - Vec3{reach, reach, reach});
        box.expand(c + Vec3{reach, reach, reach});
        return;
    }
    if (shape.kind == ConvexShape::Kind::Box) {
        const Vec3 signs[8] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                               {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
        for (const Vec3& sign : signs) {
            if (shape.useMatrix) {
                box.expand(transformPoint(shape.world, shape.localCenter + Vec3{sign.x * shape.localHalf.x, sign.y * shape.localHalf.y,
                                                                                sign.z * shape.localHalf.z}));
            } else {
                box.expand(shape.center + shape.rotation.rotate(Vec3{sign.x * shape.half.x, sign.y * shape.half.y, sign.z * shape.half.z}));
            }
        }
        return;
    }
    for (const Vec3& point : shape.localPoints) {
        box.expand(shape.useMatrix ? transformPoint(shape.world, point) : shape.center + shape.rotation.rotate(point));
    }
}

ConvexShape placed(const Solid& solid, const Mat4& world) {
    ConvexShape shape = solid.local;
    if (shape.kind == ConvexShape::Kind::Sphere || shape.kind == ConvexShape::Kind::Capsule) {
        const double sx = length(transformVector(world, {1, 0, 0}));
        const double sy = length(transformVector(world, {0, 1, 0}));
        const double sz = length(transformVector(world, {0, 0, 1}));
        const double scale = std::max(sx, std::max(sy, sz));
        shape.center = transformPoint(world, shape.center);
        shape.radius *= scale == 0 ? 1 : scale;
        shape.height *= scale == 0 ? 1 : scale;
        return shape;
    }
    shape.useMatrix = true;
    shape.world = world;
    return shape;
}

double pointTriangleDistance(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = p - a;
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) {
        return length(ap);
    }
    const Vec3 bp = p - b;
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) {
        return length(bp);
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const double v = d1 / (d1 - d3);
        return length(p - (a + ab * v));
    }
    const Vec3 cp = p - c;
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) {
        return length(cp);
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const double w = d2 / (d2 - d6);
        return length(p - (a + ac * w));
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return length(p - (b + (c - b) * w));
    }
    const double denom = va + vb + vc;
    if (std::abs(denom) < 1e-14) {
        return length(ap);
    }
    const Vec3 closest = a + ab * (vb / denom) + ac * (vc / denom);
    return length(p - closest);
}

double edgeEdgeDistance(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2) {
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const double a = dot(d1, d1);
    const double e = dot(d2, d2);
    const double f = dot(d2, r);
    double s = 0;
    double t = 0;
    if (a <= 1e-14 && e <= 1e-14) {
        return length(p1 - p2);
    }
    if (a <= 1e-14) {
        s = 0;
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = dot(d1, r);
        if (e <= 1e-14) {
            t = 0;
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = dot(d1, d2);
            const double denom = a * e - b * b;
            s = denom > 1e-14 ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0;
            t = (b * s + f) / e;
            if (t < 0) {
                t = 0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1) {
                t = 1;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return length(p1 + d1 * s - (p2 + d2 * t));
}

bool segmentHitsTriangle(Vec3 p, Vec3 q, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 dir = q - p;
    const Vec3 e1 = b - a;
    const Vec3 e2 = c - a;
    const Vec3 h = cross(dir, e2);
    const double det = dot(e1, h);
    if (std::abs(det) < 1e-10) {
        return false;
    }
    const double inv = 1.0 / det;
    const Vec3 s = p - a;
    const double u = dot(s, h) * inv;
    if (u < 0 || u > 1) {
        return false;
    }
    const Vec3 k = cross(s, e1);
    const double v = dot(dir, k) * inv;
    if (v < 0 || u + v > 1) {
        return false;
    }
    const double t = dot(e2, k) * inv;
    return t >= 0 && t <= 1;
}

double meshDistance(const MeshData& a, const Mat4& wa, const MeshData& b, const Mat4& wb) {
    if (a.triangleCount() * b.triangleCount() > 512 || a.triangleCount() == 0 || b.triangleCount() == 0) {
        return -1;
    }
    auto vert = [](const MeshData& mesh, const Mat4& world, std::uint32_t index) {
        const std::size_t base = static_cast<std::size_t>(index) * 3;
        return transformPoint(world, Vec3{mesh.positions[base], mesh.positions[base + 1], mesh.positions[base + 2]});
    };
    double best = 1e300;
    for (std::size_t ia = 0; ia + 2 < a.indices.size(); ia += 3) {
        const Vec3 a0 = vert(a, wa, a.indices[ia]);
        const Vec3 a1 = vert(a, wa, a.indices[ia + 1]);
        const Vec3 a2 = vert(a, wa, a.indices[ia + 2]);
        for (std::size_t ib = 0; ib + 2 < b.indices.size(); ib += 3) {
            const Vec3 b0 = vert(b, wb, b.indices[ib]);
            const Vec3 b1 = vert(b, wb, b.indices[ib + 1]);
            const Vec3 b2 = vert(b, wb, b.indices[ib + 2]);
            if (segmentHitsTriangle(a0, a1, b0, b1, b2) || segmentHitsTriangle(a1, a2, b0, b1, b2) ||
                segmentHitsTriangle(a2, a0, b0, b1, b2) || segmentHitsTriangle(b0, b1, a0, a1, a2) ||
                segmentHitsTriangle(b1, b2, a0, a1, a2) || segmentHitsTriangle(b2, b0, a0, a1, a2)) {
                return 0;
            }
            best = std::min(best, pointTriangleDistance(a0, b0, b1, b2));
            best = std::min(best, pointTriangleDistance(a1, b0, b1, b2));
            best = std::min(best, pointTriangleDistance(a2, b0, b1, b2));
            best = std::min(best, pointTriangleDistance(b0, a0, a1, a2));
            best = std::min(best, pointTriangleDistance(b1, a0, a1, a2));
            best = std::min(best, pointTriangleDistance(b2, a0, a1, a2));
            best = std::min(best, edgeEdgeDistance(a0, a1, b0, b1));
            best = std::min(best, edgeEdgeDistance(a0, a1, b1, b2));
            best = std::min(best, edgeEdgeDistance(a0, a1, b2, b0));
            best = std::min(best, edgeEdgeDistance(a1, a2, b0, b1));
            best = std::min(best, edgeEdgeDistance(a1, a2, b1, b2));
            best = std::min(best, edgeEdgeDistance(a1, a2, b2, b0));
            best = std::min(best, edgeEdgeDistance(a2, a0, b0, b1));
            best = std::min(best, edgeEdgeDistance(a2, a0, b1, b2));
            best = std::min(best, edgeEdgeDistance(a2, a0, b2, b0));
        }
    }
    return best;
}

bool inSet(const CollisionSet& set, const std::string& name) {
    return std::find(set.objects.begin(), set.objects.end(), name) != set.objects.end();
}

const CollisionSet* findSet(const ProximitySetup& setup, const std::string& name) {
    for (const CollisionSet& set : setup.sets) {
        if (set.name == name) {
            return &set;
        }
    }
    return nullptr;
}

}  // namespace

void paintProximity(Scene& scene, const std::vector<ContactHit>& hits) {
    const Vec3 red{0.86, 0.28, 0.22};
    const Vec3 yellow{0.92, 0.78, 0.20};
    std::unordered_map<std::string, int> rank;
    for (const ContactHit& hit : hits) {
        if (hit.kind == HitKind::Separation) {
            continue;
        }
        const int value = hit.kind == HitKind::Overlap ? 2 : 1;
        rank[hit.a] = std::max(rank[hit.a], value);
        rank[hit.b] = std::max(rank[hit.b], value);
    }
    for (const auto& [name, value] : rank) {
        if (auto node = scene.findByName(name)) {
            node->color = value == 2 ? red : yellow;
        }
    }
}

class CpuCollisionQuery final : public ICollisionQuery {
public:
    const char* backendName() const override { return "cpu-bvh-gjk"; }

    void rebuild(const Scene& scene) override {
        parts_.clear();
        sdf_.clear();
        ready_ = true;
        if (!scene.root()) {
            bvh_.clear();
            return;
        }
        std::vector<std::shared_ptr<SceneNode>> stack{scene.root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (!node->effectivelyVisible() || node->kind != NodeKind::Part) {
                continue;
            }
            Part part;
            part.name = node->name;
            part.id = node->id;
            part.node = node;
            part.meshId = node->meshId;
            part.world = node->worldMatrix();
            if (!node->meshId.empty()) {
                part.mesh = scene.mesh(node->meshId).get();
            }
            if (part.mesh != nullptr && !part.mesh->empty()) {
                const bool box = part.mesh->triangleCount() == 12 && part.mesh->bounds.valid;
                bool rectangular = box;
                if (box) {
                    const Vec3 lo = part.mesh->bounds.min;
                    const Vec3 hi = part.mesh->bounds.max;
                    for (std::size_t i = 0; i + 2 < part.mesh->positions.size(); i += 3) {
                        const auto on = [](float value, double a, double b) {
                            return std::abs(static_cast<double>(value) - a) < 1e-3 || std::abs(static_cast<double>(value) - b) < 1e-3;
                        };
                        if (!on(part.mesh->positions[i], lo.x, hi.x) || !on(part.mesh->positions[i + 1], lo.y, hi.y) ||
                            !on(part.mesh->positions[i + 2], lo.z, hi.z)) {
                            rectangular = false;
                            break;
                        }
                    }
                }
                if (rectangular) {
                    Solid solid;
                    solid.local.kind = ConvexShape::Kind::Box;
                    solid.local.localCenter = part.mesh->bounds.center();
                    solid.local.localHalf = part.mesh->bounds.size() * 0.5;
                    solid.local.center = solid.local.localCenter;
                    solid.local.half = solid.local.localHalf;
                    part.solids.push_back(std::move(solid));
                    part.primitive = true;
                } else {
                    const std::uint64_t fingerprint = meshFingerprint(*part.mesh);
                    const ConvexCacheEntry* cached = nullptr;
                    for (const ConvexCacheEntry& entry : scene.convexCache) {
                        if (entry.meshId == part.meshId && entry.fingerprint == fingerprint) {
                            cached = &entry;
                            break;
                        }
                    }
                    if (cached == nullptr) {
                        for (const ConvexCacheEntry& entry : pending_) {
                            if (entry.meshId == part.meshId && entry.fingerprint == fingerprint) {
                                cached = &entry;
                                break;
                            }
                        }
                    }
                    std::vector<std::vector<Vec3>> pieces;
                    if (cached != nullptr) {
                        for (const ConvexPiece& piece : cached->pieces) {
                            pieces.push_back(piece.vertices);
                        }
                    } else {
                        pieces = decomposeMesh(*part.mesh);
                        ConvexCacheEntry entry;
                        entry.meshId = part.meshId;
                        entry.fingerprint = fingerprint;
                        for (const auto& piece : pieces) {
                            entry.pieces.push_back(ConvexPiece{piece});
                        }
                        if (!entry.pieces.empty()) {
                            pending_.push_back(std::move(entry));
                        }
                    }
                    for (const auto& piece : pieces) {
                        Solid solid;
                        solid.local.kind = ConvexShape::Kind::Hull;
                        solid.local.localPoints = piece;
                        part.solids.push_back(std::move(solid));
                    }
                }
            } else if (node->mech.shape == CollisionShape::Sphere) {
                Solid solid;
                solid.local.kind = ConvexShape::Kind::Sphere;
                solid.local.radius = node->mech.radius;
                part.solids.push_back(std::move(solid));
                part.primitive = true;
            } else if (node->mech.shape == CollisionShape::Capsule) {
                Solid solid;
                solid.local.kind = ConvexShape::Kind::Capsule;
                solid.local.radius = node->mech.radius;
                solid.local.height = node->mech.height;
                solid.local.axis = node->mech.axis;
                part.solids.push_back(std::move(solid));
                part.primitive = true;
            } else if (node->mech.shape == CollisionShape::Box && node->mech.kind != MechKind::None) {
                Solid solid;
                solid.local.kind = ConvexShape::Kind::Box;
                solid.local.localHalf = node->mech.halfExtents;
                solid.local.half = node->mech.halfExtents;
                solid.local.center = {};
                part.solids.push_back(std::move(solid));
                part.primitive = true;
            }
            if (!part.solids.empty()) {
                parts_.push_back(std::move(part));
            }
        }
        refit(scene);
        buildBvh();
    }

    void posePart(Part& part, const Scene& scene) {
        if (auto node = part.node.lock()) {
            part.world = node->worldMatrix();
        } else if (auto node = scene.find(part.id)) {
            part.node = node;
            part.world = node->worldMatrix();
        }
        part.worldShapes.clear();
        part.bounds = {};
        for (const Solid& solid : part.solids) {
            part.worldShapes.push_back(placed(solid, part.world));
            expandShape(part.bounds, part.worldShapes.back());
        }
    }

    void refit(const Scene& scene) override {
        for (Part& part : parts_) {
            posePart(part, scene);
        }
        if (static_cast<int>(bvh_.prim.size()) == static_cast<int>(parts_.size()) * 2 - 1 || parts_.size() <= 1) {
            refitBvh();
        }
    }

    std::vector<ContactHit> query(const Scene& scene, double clearanceMm) override {
        if (!ready_) {
            rebuild(scene);
        } else {
            refit(scene);
        }
        std::vector<ContactHit> hits;
        if (parts_.size() < 2) {
            return hits;
        }
        std::vector<std::pair<int, int>> pairs;
        collectPairs(clearanceMm, pairs);
        for (const auto& [i, j] : pairs) {
            if (!allowed(scene, parts_[static_cast<std::size_t>(i)].name, parts_[static_cast<std::size_t>(j)].name)) {
                continue;
            }
            ContactHit hit = evaluate(parts_[static_cast<std::size_t>(i)], parts_[static_cast<std::size_t>(j)]);
            if (hit.distance > clearanceMm) {
                continue;
            }
            hit.kind = hit.distance <= 1e-4 ? HitKind::Overlap : HitKind::Clearance;
            hits.push_back(std::move(hit));
        }
        return hits;
    }

    DistanceResult minDistance(const Scene& scene, const std::string& a, const std::string& b) override {
        DistanceResult result;
        if (!ready_) {
            rebuild(scene);
        }
        Part* left = findPartMutable(a);
        Part* right = findPartMutable(b);
        if (left == nullptr || right == nullptr || left == right) {
            result.error = "need two named collidable parts";
            return result;
        }
        posePart(*left, scene);
        posePart(*right, scene);
        const ContactHit hit = evaluate(*left, *right);
        result.ok = true;
        result.distance = hit.distance;
        result.pointA = hit.pointA;
        result.pointB = hit.pointB;
        result.normal = hit.normal;
        result.gradient = length(hit.normal) > 0 ? hit.normal : Vec3{1, 0, 0};
        if (const SdfGrid* grid = sdfFor(*left)) {
            double distance = 0;
            Vec3 gradient;
            const Mat4 inverse = inverseAffine(left->world);
            if (sampleSdf(*grid, transformPoint(inverse, hit.pointA), distance, gradient)) {
                result.gradient = normalize(transformVector(left->world, gradient));
                result.gradientFromSdf = true;
            }
        }
        return result;
    }

    SweepHit sweep(Scene& scene, const std::string& name, Vec3 from, Vec3 to) override {
        SweepHit sweep;
        auto node = scene.findByName(name);
        if (!node) {
            sweep.error = "no object named " + name;
            return sweep;
        }
        const Transform saved = node->local;
        node->local.translation = from;
        rebuild(scene);
        const Part* moving = findPart(name);
        if (moving == nullptr) {
            node->local = saved;
            sweep.error = "object has no collision shape";
            return sweep;
        }
        int index = -1;
        for (int i = 0; i < static_cast<int>(parts_.size()); ++i) {
            if (parts_[static_cast<std::size_t>(i)].name == name) {
                index = i;
                break;
            }
        }
        const Vec3 delta = to - from;
        const double path = length(delta);
        double t = 0;
        for (int iter = 0; iter < 32; ++iter) {
            node->local.translation = from + delta * t;
            refit(scene);
            ContactHit closest;
            closest.distance = 1e300;
            bool any = false;
            for (int other = 0; other < static_cast<int>(parts_.size()); ++other) {
                if (other == index) {
                    continue;
                }
                if (!allowed(scene, parts_[static_cast<std::size_t>(index)].name, parts_[static_cast<std::size_t>(other)].name) &&
                    !scene.proximity.pairs.empty()) {
                    continue;
                }
                ContactHit hit = evaluate(parts_[static_cast<std::size_t>(index)], parts_[static_cast<std::size_t>(other)]);
                if (!any || hit.distance < closest.distance) {
                    any = true;
                    closest = std::move(hit);
                }
            }
            if (!any) {
                break;
            }
            if (closest.distance <= 1e-3 || path < 1e-9) {
                sweep.hit = closest.distance <= 1e-3;
                sweep.time = t;
                sweep.a = name;
                sweep.b = closest.a == name ? closest.b : closest.a;
                sweep.point = closest.pointA;
                node->local = saved;
                return sweep;
            }
            const double step = closest.distance / path;
            if (t + step >= 1.0) {
                node->local = saved;
                sweep.time = 1;
                return sweep;
            }
            t += step * 0.99;
        }
        node->local = saved;
        sweep.time = t;
        sweep.hit = false;
        return sweep;
    }

    void commitCache(Scene& scene) const override {
        for (const ConvexCacheEntry& entry : pending_) {
            bool replaced = false;
            for (ConvexCacheEntry& existing : scene.convexCache) {
                if (existing.meshId == entry.meshId) {
                    existing = entry;
                    replaced = true;
                    break;
                }
            }
            if (!replaced) {
                scene.convexCache.push_back(entry);
            }
        }
    }

private:
    struct Bvh {
        std::vector<float> minX, minY, minZ, maxX, maxY, maxZ;
        std::vector<int> left, right, parent, prim;
        int root = -1;

        void clear() {
            minX.clear();
            minY.clear();
            minZ.clear();
            maxX.clear();
            maxY.clear();
            maxZ.clear();
            left.clear();
            right.clear();
            parent.clear();
            prim.clear();
            root = -1;
        }
    };

    const Part* findPart(const std::string& name) const {
        for (const Part& part : parts_) {
            if (part.name == name) {
                return &part;
            }
        }
        return nullptr;
    }

    Part* findPartMutable(const std::string& name) {
        for (Part& part : parts_) {
            if (part.name == name) {
                return &part;
            }
        }
        return nullptr;
    }

    bool allowed(const Scene& scene, const std::string& a, const std::string& b) const {
        if (scene.proximity.pairs.empty()) {
            return true;
        }
        for (const auto& pair : scene.proximity.pairs) {
            const CollisionSet* left = findSet(scene.proximity, pair.first);
            const CollisionSet* right = findSet(scene.proximity, pair.second);
            if (left == nullptr || right == nullptr) {
                continue;
            }
            const bool aLeft = inSet(*left, a);
            const bool bLeft = inSet(*left, b);
            const bool aRight = inSet(*right, a);
            const bool bRight = inSet(*right, b);
            if ((aLeft && bRight) || (bLeft && aRight)) {
                return true;
            }
        }
        return false;
    }

    ContactHit evaluate(const Part& a, const Part& b) const {
        ContactHit best;
        best.distance = 1e300;
        best.a = a.name;
        best.b = b.name;
        bool any = false;
        for (const ConvexShape& left : a.worldShapes) {
            for (const ConvexShape& right : b.worldShapes) {
                ContactHit hit = queryConvex(left, right);
                if (!any || hit.distance < best.distance) {
                    any = true;
                    const std::string nameA = best.a;
                    const std::string nameB = best.b;
                    best = std::move(hit);
                    best.a = nameA;
                    best.b = nameB;
                }
            }
        }
        if (any && !a.primitive && !b.primitive && a.mesh != nullptr && b.mesh != nullptr && best.distance < 1.0) {
            const double exact = meshDistance(*a.mesh, a.world, *b.mesh, b.world);
            if (exact >= 0 && exact > best.distance + 1e-3) {
                best.distance = exact;
                best.penetration = 0;
                best.kind = HitKind::Separation;
                best.volume = 0;
                best.volumeEstimated = false;
            }
        }
        if (!any) {
            best.distance = 1e300;
        }
        return best;
    }

    const SdfGrid* sdfFor(const Part& part) {
        if (part.meshId.empty()) {
            return nullptr;
        }
        const auto found = sdf_.find(part.meshId);
        if (found != sdf_.end()) {
            return &found->second;
        }
        std::vector<ConvexShape> local;
        for (const Solid& solid : part.solids) {
            local.push_back(solid.local);
        }
        sdf_.emplace(part.meshId, buildSdf(local, 16));
        return &sdf_.at(part.meshId);
    }

    void buildBvh() {
        bvh_.clear();
        if (parts_.empty()) {
            return;
        }
        std::vector<int> order(parts_.size());
        for (int i = 0; i < static_cast<int>(parts_.size()); ++i) {
            order[static_cast<std::size_t>(i)] = i;
        }
        bvh_.root = buildNode(order, 0, static_cast<int>(order.size()));
        const int count = static_cast<int>(bvh_.prim.size());
        bvh_.parent.assign(static_cast<std::size_t>(count), -1);
        for (int i = 0; i < count; ++i) {
            if (bvh_.left[static_cast<std::size_t>(i)] >= 0) {
                bvh_.parent[static_cast<std::size_t>(bvh_.left[static_cast<std::size_t>(i)])] = i;
            }
            if (bvh_.right[static_cast<std::size_t>(i)] >= 0) {
                bvh_.parent[static_cast<std::size_t>(bvh_.right[static_cast<std::size_t>(i)])] = i;
            }
        }
    }

    int buildNode(std::vector<int>& order, int begin, int end) {
        const int node = static_cast<int>(bvh_.prim.size());
        bvh_.prim.push_back(-1);
        bvh_.left.push_back(-1);
        bvh_.right.push_back(-1);
        bvh_.minX.push_back(0);
        bvh_.minY.push_back(0);
        bvh_.minZ.push_back(0);
        bvh_.maxX.push_back(0);
        bvh_.maxY.push_back(0);
        bvh_.maxZ.push_back(0);
        if (end - begin == 1) {
            bvh_.prim[static_cast<std::size_t>(node)] = order[static_cast<std::size_t>(begin)];
            writeLeaf(node, order[static_cast<std::size_t>(begin)]);
            return node;
        }
        Aabb centroid;
        for (int i = begin; i < end; ++i) {
            const Aabb& box = parts_[static_cast<std::size_t>(order[static_cast<std::size_t>(i)])].bounds;
            if (box.valid) {
                centroid.expand(box.center());
            }
        }
        const Vec3 span = centroid.valid ? centroid.size() : Vec3{};
        int axis = 0;
        if (span.y > span.x && span.y >= span.z) {
            axis = 1;
        } else if (span.z > span.x && span.z >= span.y) {
            axis = 2;
        }
        const int mid = (begin + end) / 2;
        std::nth_element(order.begin() + begin, order.begin() + mid, order.begin() + end, [&](int lhs, int rhs) {
            const Vec3 a = parts_[static_cast<std::size_t>(lhs)].bounds.center();
            const Vec3 b = parts_[static_cast<std::size_t>(rhs)].bounds.center();
            const double av = axis == 0 ? a.x : axis == 1 ? a.y : a.z;
            const double bv = axis == 0 ? b.x : axis == 1 ? b.y : b.z;
            return av < bv;
        });
        const int left = buildNode(order, begin, mid);
        const int right = buildNode(order, mid, end);
        bvh_.left[static_cast<std::size_t>(node)] = left;
        bvh_.right[static_cast<std::size_t>(node)] = right;
        writeUnion(node);
        return node;
    }

    void writeLeaf(int node, int part) {
        const Aabb& box = parts_[static_cast<std::size_t>(part)].bounds;
        const Vec3 lo = box.valid ? box.min : Vec3{};
        const Vec3 hi = box.valid ? box.max : Vec3{};
        bvh_.minX[static_cast<std::size_t>(node)] = static_cast<float>(lo.x);
        bvh_.minY[static_cast<std::size_t>(node)] = static_cast<float>(lo.y);
        bvh_.minZ[static_cast<std::size_t>(node)] = static_cast<float>(lo.z);
        bvh_.maxX[static_cast<std::size_t>(node)] = static_cast<float>(hi.x);
        bvh_.maxY[static_cast<std::size_t>(node)] = static_cast<float>(hi.y);
        bvh_.maxZ[static_cast<std::size_t>(node)] = static_cast<float>(hi.z);
    }

    void writeUnion(int node) {
        const int left = bvh_.left[static_cast<std::size_t>(node)];
        const int right = bvh_.right[static_cast<std::size_t>(node)];
        bvh_.minX[static_cast<std::size_t>(node)] = std::min(bvh_.minX[static_cast<std::size_t>(left)], bvh_.minX[static_cast<std::size_t>(right)]);
        bvh_.minY[static_cast<std::size_t>(node)] = std::min(bvh_.minY[static_cast<std::size_t>(left)], bvh_.minY[static_cast<std::size_t>(right)]);
        bvh_.minZ[static_cast<std::size_t>(node)] = std::min(bvh_.minZ[static_cast<std::size_t>(left)], bvh_.minZ[static_cast<std::size_t>(right)]);
        bvh_.maxX[static_cast<std::size_t>(node)] = std::max(bvh_.maxX[static_cast<std::size_t>(left)], bvh_.maxX[static_cast<std::size_t>(right)]);
        bvh_.maxY[static_cast<std::size_t>(node)] = std::max(bvh_.maxY[static_cast<std::size_t>(left)], bvh_.maxY[static_cast<std::size_t>(right)]);
        bvh_.maxZ[static_cast<std::size_t>(node)] = std::max(bvh_.maxZ[static_cast<std::size_t>(left)], bvh_.maxZ[static_cast<std::size_t>(right)]);
    }

    void refitBvh() {
        if (bvh_.prim.empty()) {
            return;
        }
        for (int i = static_cast<int>(bvh_.prim.size()) - 1; i >= 0; --i) {
            if (bvh_.prim[static_cast<std::size_t>(i)] >= 0) {
                writeLeaf(i, bvh_.prim[static_cast<std::size_t>(i)]);
            } else if (bvh_.left[static_cast<std::size_t>(i)] >= 0) {
                writeUnion(i);
            }
        }
    }

    bool overlapNodes(int a, int b, float margin) const {
        return bvh_.minX[static_cast<std::size_t>(a)] <= bvh_.maxX[static_cast<std::size_t>(b)] + margin &&
               bvh_.maxX[static_cast<std::size_t>(a)] + margin >= bvh_.minX[static_cast<std::size_t>(b)] &&
               bvh_.minY[static_cast<std::size_t>(a)] <= bvh_.maxY[static_cast<std::size_t>(b)] + margin &&
               bvh_.maxY[static_cast<std::size_t>(a)] + margin >= bvh_.minY[static_cast<std::size_t>(b)] &&
               bvh_.minZ[static_cast<std::size_t>(a)] <= bvh_.maxZ[static_cast<std::size_t>(b)] + margin &&
               bvh_.maxZ[static_cast<std::size_t>(a)] + margin >= bvh_.minZ[static_cast<std::size_t>(b)];
    }

    void collectPairs(double clearance, std::vector<std::pair<int, int>>& pairs) const {
        if (bvh_.root < 0) {
            return;
        }
        const float margin = static_cast<float>(std::max(0.0, clearance));
        std::vector<std::pair<int, int>> stack;
        if (bvh_.prim[static_cast<std::size_t>(bvh_.root)] >= 0) {
            return;
        }
        stack.emplace_back(bvh_.left[static_cast<std::size_t>(bvh_.root)], bvh_.right[static_cast<std::size_t>(bvh_.root)]);
        std::vector<int> self;
        self.push_back(bvh_.left[static_cast<std::size_t>(bvh_.root)]);
        self.push_back(bvh_.right[static_cast<std::size_t>(bvh_.root)]);
        while (!self.empty()) {
            const int node = self.back();
            self.pop_back();
            if (node < 0 || bvh_.prim[static_cast<std::size_t>(node)] >= 0) {
                continue;
            }
            stack.emplace_back(bvh_.left[static_cast<std::size_t>(node)], bvh_.right[static_cast<std::size_t>(node)]);
            self.push_back(bvh_.left[static_cast<std::size_t>(node)]);
            self.push_back(bvh_.right[static_cast<std::size_t>(node)]);
        }
        while (!stack.empty()) {
            const auto [a, b] = stack.back();
            stack.pop_back();
            if (a < 0 || b < 0 || !overlapNodes(a, b, margin)) {
                continue;
            }
            const bool leafA = bvh_.prim[static_cast<std::size_t>(a)] >= 0;
            const bool leafB = bvh_.prim[static_cast<std::size_t>(b)] >= 0;
            if (leafA && leafB) {
                const int ia = bvh_.prim[static_cast<std::size_t>(a)];
                const int ib = bvh_.prim[static_cast<std::size_t>(b)];
                if (ia != ib) {
                    pairs.emplace_back(std::min(ia, ib), std::max(ia, ib));
                }
                continue;
            }
            if (leafA) {
                stack.emplace_back(a, bvh_.left[static_cast<std::size_t>(b)]);
                stack.emplace_back(a, bvh_.right[static_cast<std::size_t>(b)]);
                continue;
            }
            if (leafB) {
                stack.emplace_back(bvh_.left[static_cast<std::size_t>(a)], b);
                stack.emplace_back(bvh_.right[static_cast<std::size_t>(a)], b);
                continue;
            }
            stack.emplace_back(bvh_.left[static_cast<std::size_t>(a)], bvh_.left[static_cast<std::size_t>(b)]);
            stack.emplace_back(bvh_.left[static_cast<std::size_t>(a)], bvh_.right[static_cast<std::size_t>(b)]);
            stack.emplace_back(bvh_.right[static_cast<std::size_t>(a)], bvh_.left[static_cast<std::size_t>(b)]);
            stack.emplace_back(bvh_.right[static_cast<std::size_t>(a)], bvh_.right[static_cast<std::size_t>(b)]);
        }
        std::sort(pairs.begin(), pairs.end());
        pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    }

    std::vector<Part> parts_;
    std::vector<ConvexCacheEntry> pending_;
    std::unordered_map<std::string, SdfGrid> sdf_;
    Bvh bvh_;
    bool ready_ = false;
};

std::unique_ptr<ICollisionQuery> createCollisionQuery() { return std::make_unique<CpuCollisionQuery>(); }

}  // namespace sim
