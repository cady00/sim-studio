#include "sim/core/Pick.hpp"

namespace sim {
namespace {

bool rayTriangle(const Vec3& origin, const Vec3& direction, const Vec3& v0, const Vec3& v1, const Vec3& v2,
                 double& tOut, Vec3& hitOut) {
    const Vec3 edge1 = v1 - v0;
    const Vec3 edge2 = v2 - v0;
    const Vec3 pvec = cross(direction, edge2);
    const double det = dot(edge1, pvec);
    if (std::abs(det) < 1e-12) {
        return false;
    }
    const double invDet = 1.0 / det;
    const Vec3 tvec = origin - v0;
    const double u = dot(tvec, pvec) * invDet;
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    const Vec3 qvec = cross(tvec, edge1);
    const double v = dot(direction, qvec) * invDet;
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    const double t = dot(edge2, qvec) * invDet;
    if (t < 1e-6) {
        return false;
    }
    tOut = t;
    hitOut = origin + direction * t;
    return true;
}

}  // namespace

std::optional<PickHit> pickScene(const Scene& scene, const Ray& ray) {
    const auto root = scene.root();
    if (!root) {
        return std::nullopt;
    }
    const Vec3 direction = normalize(ray.direction);
    if (length(direction) < 0.5) {
        return std::nullopt;
    }

    std::optional<PickHit> best;
    std::vector<std::shared_ptr<SceneNode>> stack{root};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (!node->effectivelyVisible() || node->meshId.empty()) {
            continue;
        }
        const auto mesh = scene.mesh(node->meshId);
        if (!mesh) {
            continue;
        }
        const Mat4 world = node->worldMatrix();
        const Mat4 inverse = inverseAffine(world);
        const Vec3 localOrigin = transformPoint(inverse, ray.origin);
        const Vec3 localDirection = normalize(transformVector(inverse, direction));
        if (length(localDirection) < 0.5) {
            continue;
        }
        const auto& idx = mesh->indices;
        const auto& pos = mesh->positions;
        for (std::size_t i = 0; i + 2 < idx.size(); i += 3) {
            const auto i0 = static_cast<std::size_t>(idx[i]) * 3;
            const auto i1 = static_cast<std::size_t>(idx[i + 1]) * 3;
            const auto i2 = static_cast<std::size_t>(idx[i + 2]) * 3;
            if (i0 + 2 >= pos.size() || i1 + 2 >= pos.size() || i2 + 2 >= pos.size()) {
                continue;
            }
            const Vec3 v0{pos[i0], pos[i0 + 1], pos[i0 + 2]};
            const Vec3 v1{pos[i1], pos[i1 + 1], pos[i1 + 2]};
            const Vec3 v2{pos[i2], pos[i2 + 1], pos[i2 + 2]};
            double localT = 0;
            Vec3 localHit;
            if (!rayTriangle(localOrigin, localDirection, v0, v1, v2, localT, localHit)) {
                continue;
            }
            const Vec3 worldHit = transformPoint(world, localHit);
            const double distance = dot(worldHit - ray.origin, direction);
            if (distance < 0) {
                continue;
            }
            if (!best || distance < best->distance) {
                Vec3 normal = transformVector(world, cross(v1 - v0, v2 - v0));
                if (length(normal) > 1e-12) {
                    normal = normalize(normal);
                } else {
                    normal = {0, 0, 1};
                }
                best = PickHit{node->id, distance, worldHit, normal};
            }
        }
    }
    return best;
}

}  // namespace sim
