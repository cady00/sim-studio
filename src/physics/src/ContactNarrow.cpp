#include "sim/physics/ContactNarrow.hpp"

#include "sim/collision/Likelihood.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>
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

void meshLocalQueryBox(const Mat4& world, Vec3 boxCenter, Vec3 boxHalf, Vec3& localMin, Vec3& localMax) {
    localMin = {1e300, 1e300, 1e300};
    localMax = {-1e300, -1e300, -1e300};
    const Mat4 inverse = inverseAffine(world);
    for (int corner = 0; corner < 8; ++corner) {
        const Vec3 worldCorner{boxCenter.x + ((corner & 1) ? boxHalf.x : -boxHalf.x),
                               boxCenter.y + ((corner & 2) ? boxHalf.y : -boxHalf.y),
                               boxCenter.z + ((corner & 4) ? boxHalf.z : -boxHalf.z)};
        expand(localMin, localMax, transformPoint(inverse, worldCorner));
    }
    const Vec3 pad{0.5, 0.5, 0.5};
    localMin = localMin - pad;
    localMax = localMax + pad;
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

std::vector<NarrowContact> queryBoxMesh(const TriangleBvh& bvh, const Mat4& world, Vec3 boxCenter, Vec3 boxHalf, int limit,
                                                   int* satTests) {
    std::vector<NarrowContact> hits;
    if (bvh.root < 0 || limit <= 0) {
        return hits;
    }
    Vec3 localMin;
    Vec3 localMax;
    meshLocalQueryBox(world, boxCenter, boxHalf, localMin, localMax);

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
            if (satTests != nullptr) {
                ++(*satTests);
            }
            if (!satBoxTriangle(boxCenter, boxHalf, a, b, c, contact)) {
                continue;
            }
            const Vec3 centroid = (a + b + c) * (1.0 / 3.0);
            contact.point = {
                std::clamp(centroid.x, boxCenter.x - boxHalf.x, boxCenter.x + boxHalf.x),
                std::clamp(centroid.y, boxCenter.y - boxHalf.y, boxCenter.y + boxHalf.y),
                std::clamp(centroid.z, boxCenter.z - boxHalf.z, boxCenter.z + boxHalf.z),
            };
            const double apart = std::max(5.0, 0.35 * std::max(boxHalf.x, std::max(boxHalf.y, boxHalf.z)));
            bool merged = false;
            for (NarrowContact& kept : hits) {
                if (dot(kept.normal, contact.normal) > 0.85 && length(kept.point - contact.point) < apart) {
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

bool detectBoxMesh(const TriangleBvh& bvh, const Mat4& world, Vec3 boxCenter, Vec3 boxHalf, const ICollisionLikelihood* model,
                   NarrowMemory& memory, NarrowDetectStats* stats) {
    NarrowDetectStats local;
    if (stats == nullptr) {
        stats = &local;
    }
    *stats = {};
    if (bvh.root < 0) {
        memory.triangle = -1;
        return false;
    }
    Vec3 localMin;
    Vec3 localMax;
    meshLocalQueryBox(world, boxCenter, boxHalf, localMin, localMax);
    const Vec3 localCenter = (localMin + localMax) * 0.5;
    const Vec3 localHalf = (localMax - localMin) * 0.5;
    DistancePreviousLikelihood fallback;
    const ICollisionLikelihood& ranker = model != nullptr ? *model : fallback;
    const int remembered = memory.triangle;

    auto consider = [&](int triangle) {
        const std::size_t base = static_cast<std::size_t>(triangle) * 3;
        if (base + 2 >= bvh.triangles.size()) {
            return false;
        }
        const Vec3 a = transformPoint(world, bvh.triangles[base]);
        const Vec3 b = transformPoint(world, bvh.triangles[base + 1]);
        const Vec3 c = transformPoint(world, bvh.triangles[base + 2]);
        ++stats->tested;
        NarrowContact contact;
        if (!satBoxTriangle(boxCenter, boxHalf, a, b, c, contact)) {
            return false;
        }
        stats->hit = true;
        stats->triangle = triangle;
        memory.triangle = triangle;
        return true;
    };

    // The default score puts a previous hit strictly ahead of any geometric gap.
    // A learned ranker that returns a higher score for that hit keeps it in the walk below.
    if (remembered >= 0) {
        const double forced = ranker.score(0, true);
        const double other = ranker.score(-1.0e9, false);
        if (forced < other && consider(remembered)) {
            stats->candidates = 1;
            return true;
        }
    }

    struct Task {
        double score = 0;
        int index = -1;
        bool operator>(const Task& rhs) const {
            if (score != rhs.score) {
                return score > rhs.score;
            }
            return index > rhs.index;
        }
    };
    std::priority_queue<Task, std::vector<Task>, std::greater<Task>> queue;
    auto enqueue = [&](int index) {
        if (index < 0 || index >= static_cast<int>(bvh.nodes.size())) {
            return;
        }
        const TriangleBvh::Node& node = bvh.nodes[static_cast<std::size_t>(index)];
        if (!boxOverlapsBounds(localCenter, localHalf, node.bmin, node.bmax)) {
            return;
        }
        const double gap = aabbSeparation(node.bmin, node.bmax, localMin, localMax);
        const bool previous = node.triangle >= 0 && node.triangle == remembered;
        const double score = node.triangle >= 0 ? ranker.score(gap, previous) : gap;
        if (node.triangle >= 0) {
            ++stats->candidates;
        }
        queue.push(Task{score, index});
    };
    enqueue(bvh.root);
    while (!queue.empty()) {
        const Task task = queue.top();
        queue.pop();
        const TriangleBvh::Node& node = bvh.nodes[static_cast<std::size_t>(task.index)];
        if (node.triangle >= 0) {
            if (consider(node.triangle)) {
                return true;
            }
            continue;
        }
        enqueue(node.left);
        enqueue(node.right);
    }
    memory.triangle = -1;
    return false;
}

namespace {

bool boundsOverlap(Vec3 aMin, Vec3 aMax, Vec3 bMin, Vec3 bMax) {
    return rangesOverlap(aMin.x, aMax.x, bMin.x, bMax.x) && rangesOverlap(aMin.y, aMax.y, bMin.y, bMax.y) &&
           rangesOverlap(aMin.z, aMax.z, bMin.z, bMax.z);
}

void worldBoundsOf(const TriangleBvh::Node& node, const Mat4& world, Vec3& bmin, Vec3& bmax) {
    bmin = {1e300, 1e300, 1e300};
    bmax = {-1e300, -1e300, -1e300};
    for (int corner = 0; corner < 8; ++corner) {
        const Vec3 local{(corner & 1) ? node.bmax.x : node.bmin.x, (corner & 2) ? node.bmax.y : node.bmin.y,
                         (corner & 4) ? node.bmax.z : node.bmin.z};
        expand(bmin, bmax, transformPoint(world, local));
    }
}

bool projectInterval(const Vec3 tri[3], Vec3 axis, double& lo, double& hi) {
    const double span = length(axis);
    if (span < 1e-10) {
        return false;
    }
    axis = axis * (1.0 / span);
    lo = hi = dot(tri[0], axis);
    for (int i = 1; i < 3; ++i) {
        const double projected = dot(tri[i], axis);
        lo = std::min(lo, projected);
        hi = std::max(hi, projected);
    }
    return true;
}

bool satTriangleTriangle(const Vec3 a[3], const Vec3 b[3], NarrowContact& contact) {
    const Vec3 aEdges[3] = {a[1] - a[0], a[2] - a[1], a[0] - a[2]};
    const Vec3 bEdges[3] = {b[1] - b[0], b[2] - b[1], b[0] - b[2]};
    const Vec3 aNormal = cross(aEdges[0], a[2] - a[0]);
    const Vec3 bNormal = cross(bEdges[0], b[2] - b[0]);
    if (length(aNormal) < 1e-8 || length(bNormal) < 1e-8) {
        return false;
    }
    Vec3 axes[13];
    int count = 0;
    axes[count++] = aNormal;
    axes[count++] = bNormal;
    for (const Vec3& edgeA : aEdges) {
        for (const Vec3& edgeB : bEdges) {
            const Vec3 axis = cross(edgeA, edgeB);
            if (length(axis) > 1e-8) {
                axes[count++] = axis;
            }
        }
    }
    double best = 1e300;
    Vec3 bestAxis{0, 0, 1};
    for (int i = 0; i < count; ++i) {
        double aLo = 0;
        double aHi = 0;
        double bLo = 0;
        double bHi = 0;
        if (!projectInterval(a, axes[i], aLo, aHi) || !projectInterval(b, axes[i], bLo, bHi)) {
            continue;
        }
        if (aHi < bLo || bHi < aLo) {
            return false;
        }
        const double overlap = std::min(aHi, bHi) - std::max(aLo, bLo);
        if (overlap < best) {
            best = overlap;
            bestAxis = axes[i];
        }
    }
    if (!(best < 1e299) || best <= 1e-4) {
        return false;
    }
    bestAxis = normalize(bestAxis);
    const Vec3 aCenter = (a[0] + a[1] + a[2]) * (1.0 / 3.0);
    const Vec3 bCenter = (b[0] + b[1] + b[2]) * (1.0 / 3.0);
    if (dot(aCenter - bCenter, bestAxis) < 0) {
        bestAxis = bestAxis * -1.0;
    }
    contact.normal = bestAxis;
    contact.depth = best;
    contact.point = (aCenter + bCenter) * 0.5;
    return true;
}

bool rayEntersBox(Vec3 origin, Vec3 direction, double range, Vec3 center, Vec3 half, double& distance, Vec3& normal) {
    const Vec3 dir = normalize(direction);
    const double speed = length(direction);
    if (speed < 1e-12 || range < 0) {
        return false;
    }
    double tEnter = 0;
    double tExit = range;
    Vec3 enterNormal{0, 0, 1};
    const double originC[3] = {origin.x, origin.y, origin.z};
    const double dirC[3] = {dir.x, dir.y, dir.z};
    const double centerC[3] = {center.x, center.y, center.z};
    const double halfC[3] = {half.x, half.y, half.z};
    const Vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int axis = 0; axis < 3; ++axis) {
        const double minB = centerC[axis] - halfC[axis];
        const double maxB = centerC[axis] + halfC[axis];
        if (std::abs(dirC[axis]) < 1e-12) {
            if (originC[axis] < minB || originC[axis] > maxB) {
                return false;
            }
            continue;
        }
        double t0 = (minB - originC[axis]) / dirC[axis];
        double t1 = (maxB - originC[axis]) / dirC[axis];
        Vec3 n0 = axes[axis] * -1.0;
        Vec3 n1 = axes[axis];
        if (t0 > t1) {
            std::swap(t0, t1);
            std::swap(n0, n1);
        }
        if (t0 > tEnter) {
            tEnter = t0;
            enterNormal = n0;
        }
        tExit = std::min(tExit, t1);
        if (tEnter > tExit) {
            return false;
        }
    }
    if (tExit < 0 || tEnter > range) {
        return false;
    }
    distance = tEnter >= 0 ? tEnter : 0;
    if (tEnter >= 0) {
        normal = enterNormal;
        return true;
    }
    const Vec3 rel = origin - center;
    int axis = 0;
    double best = std::abs(rel.x) / std::max(half.x, 1e-9);
    const double yRatio = std::abs(rel.y) / std::max(half.y, 1e-9);
    const double zRatio = std::abs(rel.z) / std::max(half.z, 1e-9);
    if (yRatio > best) {
        best = yRatio;
        axis = 1;
    }
    if (zRatio > best) {
        axis = 2;
    }
    const double sign = axis == 0 ? rel.x : axis == 1 ? rel.y : rel.z;
    normal = axes[axis] * (sign < 0 ? -1.0 : 1.0);
    return true;
}

Vec3 closestOnTriangle(Vec3 point, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = point - a;
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) {
        return a;
    }
    const Vec3 bp = point - b;
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) {
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const double v = d1 / std::max(d1 - d3, 1e-15);
        return a + ab * v;
    }
    const Vec3 cp = point - c;
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) {
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const double w = d2 / std::max(d2 - d6, 1e-15);
        return a + ac * w;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const double w = (d4 - d3) / std::max((d4 - d3) + (d5 - d6), 1e-15);
        return b + (c - b) * w;
    }
    const double denom = 1.0 / std::max(va + vb + vc, 1e-15);
    const double v = vb * denom;
    const double w = vc * denom;
    return a + ab * v + ac * w;
}

bool sweepOneTriangle(Vec3 center, Vec3 half, Vec3 velocity, double dt, Vec3 a, Vec3 b, Vec3 c, double& toi, Vec3& normal) {
    Vec3 n = cross(b - a, c - a);
    if (length(n) < 1e-8) {
        return false;
    }
    n = normalize(n);
    double approach = dot(velocity, n);
    if (approach > 0) {
        n = n * -1.0;
        approach = -approach;
    }
    if (approach >= -1e-8) {
        return false;
    }
    const double radius = std::abs(n.x) * half.x + std::abs(n.y) * half.y + std::abs(n.z) * half.z;
    const double dist = dot(center - a, n);
    if (dist <= radius) {
        return false;
    }
    const double hitTime = (radius - dist) / approach;
    if (hitTime < 0 || hitTime > dt) {
        return false;
    }
    const Vec3 at = center + velocity * hitTime;
    const Vec3 onPlane = at - n * radius;
    const double margin = std::max(half.x, std::max(half.y, half.z));
    if (length(onPlane - closestOnTriangle(onPlane, a, b, c)) > margin) {
        return false;
    }
    toi = hitTime;
    normal = n;
    return true;
}

}  // namespace

bool rayHitsTriangle(Vec3 origin, Vec3 direction, Vec3 a, Vec3 b, Vec3 c, double& distance) {
    const Vec3 edge1 = b - a;
    const Vec3 edge2 = c - a;
    const Vec3 pvec = cross(direction, edge2);
    const double det = dot(edge1, pvec);
    if (std::abs(det) < 1e-10) {
        return false;
    }
    const double inv = 1.0 / det;
    const Vec3 tvec = origin - a;
    const double u = dot(tvec, pvec) * inv;
    if (u < -1e-6 || u > 1.0 + 1e-6) {
        return false;
    }
    const Vec3 qvec = cross(tvec, edge1);
    const double v = dot(direction, qvec) * inv;
    if (v < -1e-6 || u + v > 1.0 + 1e-6) {
        return false;
    }
    distance = dot(edge2, qvec) * inv;
    return distance > 1e-5;
}

bool pointInsideMesh(const TriangleBvh& bvh, const Mat4& world, Vec3 point) {
    if (bvh.root < 0) {
        return false;
    }
    const Vec3 direction{1.0, 0.017, 0.013};
    int crossings = 0;
    std::vector<int> stack{bvh.root};
    while (!stack.empty()) {
        const int index = stack.back();
        stack.pop_back();
        if (index < 0 || index >= static_cast<int>(bvh.nodes.size())) {
            continue;
        }
        const TriangleBvh::Node& node = bvh.nodes[static_cast<std::size_t>(index)];
        if (node.triangle >= 0) {
            const std::size_t base = static_cast<std::size_t>(node.triangle) * 3;
            if (base + 2 >= bvh.triangles.size()) {
                continue;
            }
            double distance = 0;
            if (rayHitsTriangle(point, direction, transformPoint(world, bvh.triangles[base]),
                                transformPoint(world, bvh.triangles[base + 1]), transformPoint(world, bvh.triangles[base + 2]),
                                distance)) {
                ++crossings;
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
    return (crossings % 2) == 1;
}

bool closestMeshPoint(const TriangleBvh& bvh, const Mat4& world, Vec3 point, Vec3& closest, double& distance) {
    if (bvh.root < 0) {
        return false;
    }
    bool found = false;
    distance = 1e300;
    std::vector<int> stack{bvh.root};
    while (!stack.empty()) {
        const int index = stack.back();
        stack.pop_back();
        if (index < 0 || index >= static_cast<int>(bvh.nodes.size())) {
            continue;
        }
        const TriangleBvh::Node& node = bvh.nodes[static_cast<std::size_t>(index)];
        Vec3 bmin, bmax;
        worldBoundsOf(node, world, bmin, bmax);
        const Vec3 clamped{std::clamp(point.x, bmin.x, bmax.x), std::clamp(point.y, bmin.y, bmax.y),
                           std::clamp(point.z, bmin.z, bmax.z)};
        if (found && length(point - clamped) > distance) {
            continue;
        }
        if (node.triangle >= 0) {
            const std::size_t base = static_cast<std::size_t>(node.triangle) * 3;
            if (base + 2 >= bvh.triangles.size()) {
                continue;
            }
            const Vec3 on = closestOnTriangle(point, transformPoint(world, bvh.triangles[base]),
                                              transformPoint(world, bvh.triangles[base + 1]),
                                              transformPoint(world, bvh.triangles[base + 2]));
            const double gap = length(point - on);
            if (gap < distance) {
                distance = gap;
                closest = on;
                found = true;
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
    return found;
}

void addSolidPairs(const TriangleBvh& first, const Mat4& firstWorld, const TriangleBvh& second, const Mat4& secondWorld,
                   int limit, std::vector<NarrowContact>& hits) {
    if (first.root < 0) {
        return;
    }
    std::vector<int> stack{first.root};
    while (!stack.empty() && static_cast<int>(hits.size()) + 2 <= limit) {
        const int index = stack.back();
        stack.pop_back();
        if (index < 0 || index >= static_cast<int>(first.nodes.size())) {
            continue;
        }
        const TriangleBvh::Node& node = first.nodes[static_cast<std::size_t>(index)];
        if (node.triangle < 0) {
            if (node.left >= 0) {
                stack.push_back(node.left);
            }
            if (node.right >= 0) {
                stack.push_back(node.right);
            }
            continue;
        }
        const std::size_t base = static_cast<std::size_t>(node.triangle) * 3;
        if (base + 2 >= first.triangles.size()) {
            continue;
        }
        const Vec3 a = transformPoint(firstWorld, first.triangles[base]);
        const Vec3 b = transformPoint(firstWorld, first.triangles[base + 1]);
        const Vec3 c = transformPoint(firstWorld, first.triangles[base + 2]);
        const Vec3 centroid = (a + b + c) * (1.0 / 3.0);
        if (!pointInsideMesh(second, secondWorld, centroid)) {
            continue;
        }
        Vec3 closest;
        double depth = 0;
        if (!closestMeshPoint(second, secondWorld, centroid, closest, depth) || depth <= 1e-4) {
            continue;
        }
        NarrowContact contact;
        contact.point = centroid;
        contact.depth = depth;
        contact.normal = normalize(closest - centroid);
        hits.push_back(contact);
        NarrowContact opposite = contact;
        opposite.normal = contact.normal * -1.0;
        hits.push_back(opposite);
    }
}

std::vector<NarrowContact> queryMeshMesh(const TriangleBvh& first, const Mat4& firstWorld, const TriangleBvh& second,
                                         const Mat4& secondWorld, int limit) {
    std::vector<NarrowContact> hits;
    if (first.root < 0 || second.root < 0 || limit <= 0) {
        return hits;
    }
    std::vector<std::pair<int, int>> stack{{first.root, second.root}};
    while (!stack.empty() && static_cast<int>(hits.size()) < limit) {
        const auto [ia, ib] = stack.back();
        stack.pop_back();
        if (ia < 0 || ib < 0 || ia >= static_cast<int>(first.nodes.size()) || ib >= static_cast<int>(second.nodes.size())) {
            continue;
        }
        const TriangleBvh::Node& nodeA = first.nodes[static_cast<std::size_t>(ia)];
        const TriangleBvh::Node& nodeB = second.nodes[static_cast<std::size_t>(ib)];
        Vec3 aMin, aMax, bMin, bMax;
        worldBoundsOf(nodeA, firstWorld, aMin, aMax);
        worldBoundsOf(nodeB, secondWorld, bMin, bMax);
        if (!boundsOverlap(aMin, aMax, bMin, bMax)) {
            continue;
        }
        if (nodeA.triangle >= 0 && nodeB.triangle >= 0) {
            const std::size_t baseA = static_cast<std::size_t>(nodeA.triangle) * 3;
            const std::size_t baseB = static_cast<std::size_t>(nodeB.triangle) * 3;
            if (baseA + 2 >= first.triangles.size() || baseB + 2 >= second.triangles.size()) {
                continue;
            }
            const Vec3 triA[3] = {transformPoint(firstWorld, first.triangles[baseA]),
                                  transformPoint(firstWorld, first.triangles[baseA + 1]),
                                  transformPoint(firstWorld, first.triangles[baseA + 2])};
            const Vec3 triB[3] = {transformPoint(secondWorld, second.triangles[baseB]),
                                  transformPoint(secondWorld, second.triangles[baseB + 1]),
                                  transformPoint(secondWorld, second.triangles[baseB + 2])};
            NarrowContact contact;
            if (!satTriangleTriangle(triA, triB, contact)) {
                continue;
            }
            if (static_cast<int>(hits.size()) + 2 > limit) {
                break;
            }
            hits.push_back(contact);
            NarrowContact opposite = contact;
            opposite.normal = contact.normal * -1.0;
            hits.push_back(opposite);
            continue;
        }
        if (nodeA.triangle >= 0) {
            if (nodeB.left >= 0) {
                stack.emplace_back(ia, nodeB.left);
            }
            if (nodeB.right >= 0) {
                stack.emplace_back(ia, nodeB.right);
            }
            continue;
        }
        if (nodeB.triangle >= 0) {
            if (nodeA.left >= 0) {
                stack.emplace_back(nodeA.left, ib);
            }
            if (nodeA.right >= 0) {
                stack.emplace_back(nodeA.right, ib);
            }
            continue;
        }
        const Vec3 extentA = aMax - aMin;
        const Vec3 extentB = bMax - bMin;
        const bool splitFirst = dot(extentA, extentA) >= dot(extentB, extentB);
        if (splitFirst) {
            if (nodeA.left >= 0) {
                stack.emplace_back(nodeA.left, ib);
            }
            if (nodeA.right >= 0) {
                stack.emplace_back(nodeA.right, ib);
            }
        } else {
            if (nodeB.left >= 0) {
                stack.emplace_back(ia, nodeB.left);
            }
            if (nodeB.right >= 0) {
                stack.emplace_back(ia, nodeB.right);
            }
        }
    }
    addSolidPairs(first, firstWorld, second, secondWorld, limit, hits);
    return hits;
}

bool sweepAabbToi(Vec3 center, Vec3 half, Vec3 velocity, double dt, Vec3 obstacleCenter, Vec3 obstacleHalf, double& toi,
                  Vec3& normal) {
    if (dt <= 0) {
        return false;
    }
    const double travel = length(velocity) * dt;
    double distance = 0;
    Vec3 hitNormal{0, 0, 1};
    if (!rayEntersBox(center, velocity, travel, obstacleCenter, obstacleHalf + half, distance, hitNormal)) {
        return false;
    }
    const double speed = length(velocity);
    toi = speed > 1e-12 ? distance / speed : 0;
    if (toi > dt) {
        return false;
    }
    normal = hitNormal;
    return true;
}

bool sweepAabbMesh(Vec3 center, Vec3 half, Vec3 velocity, double dt, const TriangleBvh& bvh, const Mat4& world, double& toi,
                   Vec3& normal) {
    if (bvh.root < 0 || dt <= 0) {
        return false;
    }
    const Vec3 end = center + velocity * dt;
    Vec3 regionMin{std::min(center.x, end.x), std::min(center.y, end.y), std::min(center.z, end.z)};
    Vec3 regionMax{std::max(center.x, end.x), std::max(center.y, end.y), std::max(center.z, end.z)};
    regionMin = regionMin - half;
    regionMax = regionMax + half;
    bool hit = false;
    double best = dt;
    Vec3 bestNormal{0, 0, 1};
    std::vector<int> stack{bvh.root};
    while (!stack.empty()) {
        const int index = stack.back();
        stack.pop_back();
        if (index < 0 || index >= static_cast<int>(bvh.nodes.size())) {
            continue;
        }
        const TriangleBvh::Node& node = bvh.nodes[static_cast<std::size_t>(index)];
        Vec3 bmin, bmax;
        worldBoundsOf(node, world, bmin, bmax);
        if (!boundsOverlap(regionMin, regionMax, bmin, bmax)) {
            continue;
        }
        if (node.triangle >= 0) {
            const std::size_t base = static_cast<std::size_t>(node.triangle) * 3;
            if (base + 2 >= bvh.triangles.size()) {
                continue;
            }
            double hitTime = 0;
            Vec3 hitNormal{0, 0, 1};
            if (!sweepOneTriangle(center, half, velocity, dt, transformPoint(world, bvh.triangles[base]),
                                  transformPoint(world, bvh.triangles[base + 1]), transformPoint(world, bvh.triangles[base + 2]),
                                  hitTime, hitNormal)) {
                continue;
            }
            if (hitTime < best) {
                best = hitTime;
                bestNormal = hitNormal;
                hit = true;
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
    if (!hit) {
        return false;
    }
    toi = best;
    normal = bestNormal;
    return true;
}

}  // namespace sim
