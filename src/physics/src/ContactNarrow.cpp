#include "sim/physics/ContactNarrow.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {
namespace {

struct Item {
    int triangle = 0;
    Vec3 bmin{0, 0, 0};
    Vec3 bmax{0, 0, 0};
    Vec3 center{0, 0, 0};
};

bool rangesOverlap(double a0, double a1, double b0, double b1) { return a0 <= b1 && b0 <= a1; }

bool boxOverlapsBounds(Vec3 center, Vec3 half, Vec3 bmin, Vec3 bmax) {
    return rangesOverlap(center.x - half.x, center.x + half.x, bmin.x, bmax.x) &&
           rangesOverlap(center.y - half.y, center.y + half.y, bmin.y, bmax.y) &&
           rangesOverlap(center.z - half.z, center.z + half.z, bmin.z, bmax.z);
}

void expand(Vec3& bmin, Vec3& bmax, Vec3 point) {
    bmin.x = std::min(bmin.x, point.x);
    bmin.y = std::min(bmin.y, point.y);
    bmin.z = std::min(bmin.z, point.z);
    bmax.x = std::max(bmax.x, point.x);
    bmax.y = std::max(bmax.y, point.y);
    bmax.z = std::max(bmax.z, point.z);
}

bool separatedOn(Vec3 axis, Vec3 center, Vec3 half, const Vec3 tri[3], double& overlap) {
    const double span = length(axis);
    if (span < 1e-10) {
        overlap = 1e300;
        return false;
    }
    axis = axis * (1.0 / span);
    const double radius = std::abs(axis.x) * half.x + std::abs(axis.y) * half.y + std::abs(axis.z) * half.z;
    const double boxCenter = dot(center, axis);
    double lo = dot(tri[0], axis);
    double hi = lo;
    for (int i = 1; i < 3; ++i) {
        const double projected = dot(tri[i], axis);
        lo = std::min(lo, projected);
        hi = std::max(hi, projected);
    }
    const double triWidth = hi - lo;
    if (triWidth < 1e-5) {
        // A triangle has no thickness. Interval overlap on its normal is zero even when the
        // plane cuts the box, so the depth is how far the box extends past that plane.
        const double plane = 0.5 * (lo + hi);
        const double dist = boxCenter - plane;
        if (std::abs(dist) > radius) {
            return true;
        }
        overlap = radius - std::abs(dist);
        return false;
    }
    if (boxCenter + radius < lo || boxCenter - radius > hi) {
        return true;
    }
    overlap = std::min(boxCenter + radius, hi) - std::max(boxCenter - radius, lo);
    return false;
}

bool satBoxTriangle(Vec3 center, Vec3 half, Vec3 a, Vec3 b, Vec3 c, NarrowContact& contact) {
    const Vec3 tri[3] = {a, b, c};
    const Vec3 e0 = b - a;
    const Vec3 e1 = c - b;
    const Vec3 e2 = a - c;
    if (length(cross(e0, c - a)) < 1e-8) {
        return false;
    }
    Vec3 axes[13];
    int count = 0;
    axes[count++] = {1, 0, 0};
    axes[count++] = {0, 1, 0};
    axes[count++] = {0, 0, 1};
    axes[count++] = cross(e0, c - a);
    const Vec3 boxEdges[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const Vec3 triEdges[3] = {e0, e1, e2};
    for (const Vec3& boxEdge : boxEdges) {
        for (const Vec3& triEdge : triEdges) {
            const Vec3 axis = cross(boxEdge, triEdge);
            if (length(axis) > 1e-8) {
                axes[count++] = axis;
            }
        }
    }
    double best = 1e300;
    Vec3 bestAxis{0, 0, 1};
    for (int i = 0; i < count; ++i) {
        double overlap = 0;
        if (separatedOn(axes[i], center, half, tri, overlap)) {
            return false;
        }
        if (overlap < best) {
            best = overlap;
            bestAxis = axes[i];
        }
    }
    if (!(best < 1e299) || best <= 1e-4) {
        return false;
    }
    bestAxis = normalize(bestAxis);
    const Vec3 centroid = (a + b + c) * (1.0 / 3.0);
    if (dot(center - centroid, bestAxis) < 0) {
        bestAxis = bestAxis * -1.0;
    }
    contact.normal = bestAxis;
    contact.depth = best;
    contact.point = center - Vec3{std::copysign(half.x, bestAxis.x), std::copysign(half.y, bestAxis.y), std::copysign(half.z, bestAxis.z)};
    return true;
}

}  // namespace

TriangleBvh buildTriangleBvh(const MeshData& mesh) {
    TriangleBvh bvh;
    const int triangles = static_cast<int>(mesh.triangleCount());
    if (triangles <= 0) {
        return bvh;
    }
    bvh.triangles.resize(static_cast<std::size_t>(triangles) * 3);
    std::vector<Item> items;
    items.reserve(static_cast<std::size_t>(triangles));
    for (int tri = 0; tri < triangles; ++tri) {
        Item item;
        item.triangle = tri;
        item.bmin = {1e300, 1e300, 1e300};
        item.bmax = {-1e300, -1e300, -1e300};
        bool ok = true;
        for (int k = 0; k < 3; ++k) {
            const std::size_t index = mesh.indices[static_cast<std::size_t>(tri * 3 + k)];
            const std::size_t base = index * 3;
            if (base + 2 >= mesh.positions.size()) {
                ok = false;
                break;
            }
            const Vec3 point{mesh.positions[base], mesh.positions[base + 1], mesh.positions[base + 2]};
            bvh.triangles[static_cast<std::size_t>(tri * 3 + k)] = point;
            expand(item.bmin, item.bmax, point);
        }
        if (!ok) {
            continue;
        }
        item.center = (item.bmin + item.bmax) * 0.5;
        items.push_back(item);
    }
    if (items.empty()) {
        return bvh;
    }

    std::vector<int> order(items.size());
    for (int i = 0; i < static_cast<int>(items.size()); ++i) {
        order[static_cast<std::size_t>(i)] = i;
    }
    const auto build = [&](auto&& self, int begin, int end, int depth) -> int {
        TriangleBvh::Node node;
        node.bmin = {1e300, 1e300, 1e300};
        node.bmax = {-1e300, -1e300, -1e300};
        for (int i = begin; i < end; ++i) {
            const Item& item = items[static_cast<std::size_t>(order[static_cast<std::size_t>(i)])];
            expand(node.bmin, node.bmax, item.bmin);
            expand(node.bmin, node.bmax, item.bmax);
        }
        const int index = static_cast<int>(bvh.nodes.size());
        bvh.nodes.push_back(node);
        if (end - begin <= 1 || depth > 24) {
            bvh.nodes[static_cast<std::size_t>(index)].triangle =
                items[static_cast<std::size_t>(order[static_cast<std::size_t>(begin)])].triangle;
            return index;
        }
        const Vec3 extent = node.bmax - node.bmin;
        int axis = 0;
        if (extent.y >= extent.x && extent.y >= extent.z) {
            axis = 1;
        } else if (extent.z >= extent.x && extent.z >= extent.y) {
            axis = 2;
        }
        const int mid = begin + (end - begin) / 2;
        std::nth_element(order.begin() + begin, order.begin() + mid, order.begin() + end, [&](int left, int right) {
            const Vec3& a = items[static_cast<std::size_t>(left)].center;
            const Vec3& b = items[static_cast<std::size_t>(right)].center;
            const double av = axis == 0 ? a.x : axis == 1 ? a.y : a.z;
            const double bv = axis == 0 ? b.x : axis == 1 ? b.y : b.z;
            return av < bv;
        });
        const int left = self(self, begin, mid, depth + 1);
        const int right = self(self, mid, end, depth + 1);
        bvh.nodes[static_cast<std::size_t>(index)].left = left;
        bvh.nodes[static_cast<std::size_t>(index)].right = right;
        return index;
    };
    bvh.root = build(build, 0, static_cast<int>(order.size()), 0);
    return bvh;
}

std::vector<NarrowContact> queryBoxMesh(const TriangleBvh& bvh, const Mat4& world, Vec3 boxCenter, Vec3 boxHalf, int limit) {
    std::vector<NarrowContact> hits;
    if (bvh.root < 0 || limit <= 0) {
        return hits;
    }
    const Mat4 inverse = inverseAffine(world);
    Vec3 localMin{1e300, 1e300, 1e300};
    Vec3 localMax{-1e300, -1e300, -1e300};
    for (int corner = 0; corner < 8; ++corner) {
        const Vec3 worldCorner{boxCenter.x + ((corner & 1) ? boxHalf.x : -boxHalf.x),
                               boxCenter.y + ((corner & 2) ? boxHalf.y : -boxHalf.y),
                               boxCenter.z + ((corner & 4) ? boxHalf.z : -boxHalf.z)};
        expand(localMin, localMax, transformPoint(inverse, worldCorner));
    }
    const Vec3 pad{0.5, 0.5, 0.5};
    localMin = localMin - pad;
    localMax = localMax + pad;

    std::vector<int> stack{bvh.root};
    while (!stack.empty()) {
        const int index = stack.back();
        stack.pop_back();
        if (index < 0 || index >= static_cast<int>(bvh.nodes.size())) {
            continue;
        }
        const TriangleBvh::Node& node = bvh.nodes[static_cast<std::size_t>(index)];
        if (!boxOverlapsBounds((localMin + localMax) * 0.5, (localMax - localMin) * 0.5, node.bmin, node.bmax)) {
            continue;
        }
        if (node.triangle >= 0) {
            const std::size_t base = static_cast<std::size_t>(node.triangle) * 3;
            if (base + 2 >= bvh.triangles.size()) {
                continue;
            }
            const Vec3 a = transformPoint(world, bvh.triangles[base]);
            const Vec3 b = transformPoint(world, bvh.triangles[base + 1]);
            const Vec3 c = transformPoint(world, bvh.triangles[base + 2]);
            NarrowContact contact;
            if (!satBoxTriangle(boxCenter, boxHalf, a, b, c, contact)) {
                continue;
            }
            bool merged = false;
            for (NarrowContact& kept : hits) {
                if (dot(kept.normal, contact.normal) > 0.85) {
                    if (contact.depth > kept.depth) {
                        kept = contact;
                    }
                    merged = true;
                    break;
                }
            }
            if (!merged) {
                if (static_cast<int>(hits.size()) < limit) {
                    hits.push_back(contact);
                } else {
                    auto weakest = std::min_element(hits.begin(), hits.end(), [](const NarrowContact& left, const NarrowContact& right) {
                        return left.depth < right.depth;
                    });
                    if (weakest != hits.end() && contact.depth > weakest->depth) {
                        *weakest = contact;
                    }
                }
            }
            continue;
        }
        if (node.left >= 0) {
            stack.push_back(node.left);
        }
        if (node.right >= 0) {
            stack.push_back(node.right);
        }
    }
    std::sort(hits.begin(), hits.end(), [](const NarrowContact& left, const NarrowContact& right) { return left.depth > right.depth; });
    return hits;
}

}  // namespace sim
