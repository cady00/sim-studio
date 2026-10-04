#include "sim/collision/Query.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {
namespace {

double lengthSq(Vec3 v) { return dot(v, v); }

Vec3 supportOf(const ConvexShape& shape, Vec3 direction) {
    if (lengthSq(direction) < 1e-24) {
        direction = {1, 0, 0};
    }
    if (shape.kind == ConvexShape::Kind::Sphere) {
        return shape.center + normalize(direction) * shape.radius;
    }
    if (shape.kind == ConvexShape::Kind::Capsule) {
        const Vec3 axis = normalize(shape.rotation.rotate(shape.axis));
        const double half = std::max(0.0, shape.height * 0.5 - shape.radius);
        const double side = dot(direction, axis) >= 0 ? half : -half;
        return shape.center + axis * side + normalize(direction) * shape.radius;
    }
    if (shape.kind == ConvexShape::Kind::Box && shape.useMatrix) {
        const double sx = dot(direction, transformVector(shape.world, {1, 0, 0})) >= 0 ? shape.localHalf.x : -shape.localHalf.x;
        const double sy = dot(direction, transformVector(shape.world, {0, 1, 0})) >= 0 ? shape.localHalf.y : -shape.localHalf.y;
        const double sz = dot(direction, transformVector(shape.world, {0, 0, 1})) >= 0 ? shape.localHalf.z : -shape.localHalf.z;
        return transformPoint(shape.world, shape.localCenter + Vec3{sx, sy, sz});
    }
    if (shape.kind == ConvexShape::Kind::Box) {
        const Vec3 local = shape.rotation.conjugate().rotate(direction);
        const Vec3 p{local.x >= 0 ? shape.half.x : -shape.half.x, local.y >= 0 ? shape.half.y : -shape.half.y,
                     local.z >= 0 ? shape.half.z : -shape.half.z};
        return shape.center + shape.rotation.rotate(p);
    }
    Vec3 best{};
    double bestDot = -1e300;
    const bool any = !shape.localPoints.empty();
    if (!any) {
        return shape.center;
    }
    for (const Vec3& point : shape.localPoints) {
        const Vec3 world = shape.useMatrix ? transformPoint(shape.world, point) : shape.center + shape.rotation.rotate(point);
        const double d = dot(world, direction);
        if (d > bestDot) {
            bestDot = d;
            best = world;
        }
    }
    return best;
}

struct Support {
    Vec3 p;
    Vec3 a;
    Vec3 b;
};

Support supportBoth(const ConvexShape& a, const ConvexShape& b, Vec3 direction) {
    const Vec3 pa = supportOf(a, direction);
    const Vec3 pb = supportOf(b, -direction);
    return {pa - pb, pa, pb};
}

struct Closest {
    Vec3 point;
    int index[4]{};
    double weight[4]{};
    int count = 0;
    bool containsOrigin = false;
};

Closest closestVertex(int index, Vec3 point) {
    Closest out;
    out.point = point;
    out.index[0] = index;
    out.weight[0] = 1;
    out.count = 1;
    out.containsOrigin = lengthSq(point) < 1e-16;
    return out;
}

Closest closestSegment(const Support* v, int i0, int i1) {
    const Vec3 a = v[i0].p;
    const Vec3 b = v[i1].p;
    const Vec3 ab = b - a;
    const double denom = dot(ab, ab);
    double t = denom > 1e-20 ? std::clamp(-dot(a, ab) / denom, 0.0, 1.0) : 0.0;
    Closest out;
    out.point = a + ab * t;
    out.count = 2;
    out.index[0] = i0;
    out.index[1] = i1;
    out.weight[0] = 1.0 - t;
    out.weight[1] = t;
    if (t <= 1e-10) {
        return closestVertex(i0, a);
    }
    if (t >= 1.0 - 1e-10) {
        return closestVertex(i1, b);
    }
    out.containsOrigin = lengthSq(out.point) < 1e-16;
    return out;
}

Closest closestTriangle(const Support* v, int i0, int i1, int i2) {
    const Vec3 a = v[i0].p;
    const Vec3 b = v[i1].p;
    const Vec3 c = v[i2].p;
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ao = -a;
    const double d1 = dot(ab, ao);
    const double d2 = dot(ac, ao);
    if (d1 <= 0 && d2 <= 0) {
        return closestVertex(i0, a);
    }
    const Vec3 bo = -b;
    const double d3 = dot(ab, bo);
    const double d4 = dot(ac, bo);
    if (d3 >= 0 && d4 <= d3) {
        return closestVertex(i1, b);
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const double t = d1 / (d1 - d3);
        Closest edge;
        edge.point = a + ab * t;
        edge.count = 2;
        edge.index[0] = i0;
        edge.index[1] = i1;
        edge.weight[0] = 1.0 - t;
        edge.weight[1] = t;
        edge.containsOrigin = lengthSq(edge.point) < 1e-16;
        return edge;
    }
    const Vec3 co = -c;
    const double d5 = dot(ab, co);
    const double d6 = dot(ac, co);
    if (d6 >= 0 && d5 <= d6) {
        return closestVertex(i2, c);
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const double w = d2 / (d2 - d6);
        Closest edge;
        edge.point = a + ac * w;
        edge.count = 2;
        edge.index[0] = i0;
        edge.index[1] = i2;
        edge.weight[0] = 1.0 - w;
        edge.weight[1] = w;
        edge.containsOrigin = lengthSq(edge.point) < 1e-16;
        return edge;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        Closest edge;
        edge.point = b + (c - b) * w;
        edge.count = 2;
        edge.index[0] = i1;
        edge.index[1] = i2;
        edge.weight[0] = 1.0 - w;
        edge.weight[1] = w;
        edge.containsOrigin = lengthSq(edge.point) < 1e-16;
        return edge;
    }
    const double denom = va + vb + vc;
    const double vWeight = denom != 0 ? vb / denom : 0;
    const double wWeight = denom != 0 ? vc / denom : 0;
    Closest face;
    face.point = a + ab * vWeight + ac * wWeight;
    face.count = 3;
    face.index[0] = i0;
    face.index[1] = i1;
    face.index[2] = i2;
    face.weight[0] = 1.0 - vWeight - wWeight;
    face.weight[1] = vWeight;
    face.weight[2] = wWeight;
    face.containsOrigin = lengthSq(face.point) < 1e-16;
    return face;
}

double triple(Vec3 a, Vec3 b, Vec3 c) { return dot(a, cross(b, c)); }

Closest closestTetra(const Support* v) {
    const Vec3 p{0, 0, 0};
    const Vec3 a = v[0].p;
    const Vec3 b = v[1].p;
    const Vec3 c = v[2].p;
    const Vec3 d = v[3].p;
    const double denom = triple(b - a, c - a, d - a);
    Closest best;
    best.point = a;
    best.count = 1;
    best.index[0] = 0;
    best.weight[0] = 1;
    if (std::abs(denom) > 1e-14) {
        // Weights of a,b,c,d for p. Replacing one vertex by p keeps a single orientation.
        const double w0 = triple(b - p, c - p, d - p) / denom;
        const double w1 = triple(p - a, c - a, d - a) / denom;
        const double w2 = triple(b - a, p - a, d - a) / denom;
        const double w3 = triple(b - a, c - a, p - a) / denom;
        if (w0 >= -1e-6 && w1 >= -1e-6 && w2 >= -1e-6 && w3 >= -1e-6 && std::abs(w0 + w1 + w2 + w3 - 1.0) < 1e-3) {
            Closest inside;
            inside.point = {};
            inside.count = 4;
            inside.index[0] = 0;
            inside.index[1] = 1;
            inside.index[2] = 2;
            inside.index[3] = 3;
            inside.weight[0] = w0;
            inside.weight[1] = w1;
            inside.weight[2] = w2;
            inside.weight[3] = w3;
            inside.containsOrigin = true;
            return inside;
        }
    }
    const int faces[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    double bestDist = 1e300;
    for (const auto& face : faces) {
        const Closest candidate = closestTriangle(v, face[0], face[1], face[2]);
        const double dist = lengthSq(candidate.point);
        if (dist < bestDist) {
            bestDist = dist;
            best = candidate;
        }
    }
    return best;
}

Closest closestOf(const Support* v, int n) {
    if (n <= 1) {
        return closestVertex(0, v[0].p);
    }
    if (n == 2) {
        return closestSegment(v, 0, 1);
    }
    if (n == 3) {
        return closestTriangle(v, 0, 1, 2);
    }
    return closestTetra(v);
}

Vec3 shapeCenter(const ConvexShape& shape) {
    if (shape.useMatrix && shape.kind == ConvexShape::Kind::Box) {
        return transformPoint(shape.world, shape.localCenter);
    }
    if (shape.useMatrix && !shape.localPoints.empty()) {
        return transformPoint(shape.world, shape.localPoints.front());
    }
    return shape.center;
}

// Minimum supporting-plane offset of A − B. That offset is the penetration depth.
// A polytope EPA stalls when a GJK edge already passes through the origin: new faces
// keep the origin on an edge, then side planes overshoot the true boundary.
void penetrationBySupport(const ConvexShape& shapeA, const ConvexShape& shapeB, ContactHit& hit) {
    Vec3 bestN = shapeCenter(shapeB) - shapeCenter(shapeA);
    if (lengthSq(bestN) < 1e-18) {
        bestN = {1, 0, 0};
    }
    bestN = normalize(bestN);
    double best = 1e300;
    Support bestSupport{};
    auto consider = [&](Vec3 direction) {
        if (lengthSq(direction) < 1e-18) {
            return;
        }
        direction = normalize(direction);
        const Support support = supportBoth(shapeA, shapeB, direction);
        const double offset = dot(support.p, direction);
        if (offset < best) {
            best = offset;
            bestN = direction;
            bestSupport = support;
        }
    };
    consider(bestN);
    constexpr int samples = 48;
    const double golden = kPi * (3.0 - std::sqrt(5.0));
    for (int i = 0; i < samples; ++i) {
        const double y = 1.0 - (2.0 * i + 1.0) / static_cast<double>(samples);
        const double radius = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double theta = golden * i;
        consider({std::cos(theta) * radius, y, std::sin(theta) * radius});
    }
    double step = 0.4;
    for (int pass = 0; pass < 6; ++pass) {
        Vec3 tangent = std::abs(bestN.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        tangent = normalize(cross(bestN, tangent));
        const Vec3 bitangent = cross(bestN, tangent);
        for (int iy = -1; iy <= 1; ++iy) {
            for (int ix = -1; ix <= 1; ++ix) {
                if (ix == 0 && iy == 0) {
                    continue;
                }
                consider(bestN + tangent * (ix * step) + bitangent * (iy * step));
            }
        }
        step *= 0.4;
    }
    hit.penetration = std::max(0.0, best);
    hit.distance = -hit.penetration;
    hit.pointA = bestSupport.a;
    hit.pointB = bestSupport.b;
    hit.normal = -bestN;
    hit.kind = HitKind::Overlap;
}

double sphereVolume(double radius) { return 4.0 / 3.0 * kPi * radius * radius * radius; }

double sphereIntersectionVolume(double r, double big, double distance) {
    if (distance >= r + big || r <= 0 || big <= 0) {
        return 0;
    }
    if (distance <= std::abs(big - r)) {
        return sphereVolume(std::min(r, big));
    }
    const double x = (distance * distance + r * r - big * big) / (2.0 * distance);
    const double h1 = r - x;
    const double h2 = big - (distance - x);
    const auto cap = [](double radius, double h) { return kPi * h * h * (3.0 * radius - h) / 3.0; };
    return cap(r, h1) + cap(big, h2);
}

bool pointInside(const ConvexShape& shape, Vec3 point) {
    ConvexShape probe;
    probe.kind = ConvexShape::Kind::Sphere;
    probe.center = point;
    probe.radius = 0;
    const ContactHit hit = queryConvex(probe, shape);
    return hit.distance <= 1e-3;
}

double estimatedVolume(const ConvexShape& a, const ConvexShape& b) {
    Aabb boxA;
    Aabb boxB;
    auto expand = [](Aabb& box, const ConvexShape& shape) {
        if (shape.kind == ConvexShape::Kind::Sphere || shape.kind == ConvexShape::Kind::Capsule) {
            const double r = shape.radius + std::max(0.0, shape.height * 0.5);
            box.expand(shape.center - Vec3{r, r, r});
            box.expand(shape.center + Vec3{r, r, r});
            return;
        }
        if (shape.kind == ConvexShape::Kind::Box && !shape.useMatrix) {
            for (int i = 0; i < 8; ++i) {
                const Vec3 local{(i & 1) ? shape.half.x : -shape.half.x, (i & 2) ? shape.half.y : -shape.half.y,
                                 (i & 4) ? shape.half.z : -shape.half.z};
                box.expand(shape.center + shape.rotation.rotate(local));
            }
            return;
        }
        const Vec3 corner = supportOf(shape, {1, 1, 1});
        box.expand(corner);
        box.expand(supportOf(shape, {-1, -1, -1}));
        box.expand(supportOf(shape, {1, -1, 1}));
        box.expand(supportOf(shape, {-1, 1, -1}));
    };
    expand(boxA, a);
    expand(boxB, b);
    if (!boxA.valid || !boxB.valid) {
        return 0;
    }
    const Vec3 lo{std::max(boxA.min.x, boxB.min.x), std::max(boxA.min.y, boxB.min.y), std::max(boxA.min.z, boxB.min.z)};
    const Vec3 hi{std::min(boxA.max.x, boxB.max.x), std::min(boxA.max.y, boxB.max.y), std::min(boxA.max.z, boxB.max.z)};
    if (lo.x >= hi.x || lo.y >= hi.y || lo.z >= hi.z) {
        return 0;
    }
    constexpr int n = 6;
    const Vec3 step = (hi - lo) / static_cast<double>(n);
    const double cell = step.x * step.y * step.z;
    int inside = 0;
    for (int iz = 0; iz < n; ++iz) {
        for (int iy = 0; iy < n; ++iy) {
            for (int ix = 0; ix < n; ++ix) {
                const Vec3 p = lo + Vec3{(ix + 0.5) * step.x, (iy + 0.5) * step.y, (iz + 0.5) * step.z};
                if (pointInside(a, p) && pointInside(b, p)) {
                    ++inside;
                }
            }
        }
    }
    return inside * cell;
}

}  // namespace

ContactHit queryConvex(const ConvexShape& a, const ConvexShape& b) {
    ContactHit hit;
    hit.kind = HitKind::Separation;
    Vec3 direction = shapeCenter(b) - shapeCenter(a);
    if (lengthSq(direction) < 1e-18) {
        direction = {1, 0, 0};
    }
    Support simplex[4];
    simplex[0] = supportBoth(a, b, direction);
    int count = 1;
    Vec3 closest = simplex[0].p;
    bool overlapping = false;
    const double scale = std::max(1.0, length(direction) + a.radius + b.radius);
    const double eps = 1e-8 * scale + 1e-7;

    for (int iter = 0; iter < 48; ++iter) {
        if (lengthSq(closest) <= eps * eps) {
            overlapping = true;
            break;
        }
        const Support next = supportBoth(a, b, -closest);
        const double vv = dot(closest, closest);
        const double vw = dot(next.p, closest);
        if (vv - vw <= eps * std::sqrt(std::max(vv, 0.0)) + 1e-12) {
            break;
        }
        if (count < 4) {
            simplex[count++] = next;
        } else {
            simplex[3] = next;
        }
        const Closest reduced = closestOf(simplex, count);
        if (reduced.containsOrigin || lengthSq(reduced.point) <= eps * eps) {
            overlapping = true;
            closest = reduced.point;
            break;
        }
        Support packed[4];
        int packedCount = 0;
        for (int i = 0; i < reduced.count; ++i) {
            if (reduced.weight[i] > 1e-8) {
                packed[packedCount++] = simplex[reduced.index[i]];
            }
        }
        if (packedCount == 0) {
            packed[0] = simplex[reduced.index[0]];
            packedCount = 1;
        }
        count = packedCount;
        for (int i = 0; i < count; ++i) {
            simplex[i] = packed[i];
        }
        closest = reduced.point;
    }

    auto witnesses = [&](const Closest& reduced) {
        hit.pointA = {};
        hit.pointB = {};
        for (int i = 0; i < reduced.count; ++i) {
            const Support& source = simplex[reduced.index[i]];
            hit.pointA = hit.pointA + source.a * reduced.weight[i];
            hit.pointB = hit.pointB + source.b * reduced.weight[i];
        }
    };

    if (!overlapping) {
        const Closest reduced = closestOf(simplex, count);
        witnesses(reduced);
        hit.distance = length(reduced.point);
        hit.penetration = 0;
        const Vec3 gap = hit.pointB - hit.pointA;
        hit.normal = lengthSq(gap) > 1e-16 ? normalize(gap) : (lengthSq(closest) > 1e-16 ? normalize(closest) : Vec3{1, 0, 0});
        // GJK's minkowski point is pointA - pointB, so the stored closest should match the gap flipped.
        if (dot(hit.normal, hit.pointB - hit.pointA) < 0) {
            hit.normal = -hit.normal;
        }
        hit.kind = HitKind::Separation;
        hit.volume = 0;
        hit.volumeEstimated = false;
        return hit;
    }

    penetrationBySupport(a, b, hit);

    const bool probe = (a.kind == ConvexShape::Kind::Sphere && a.radius <= 1e-9) ||
                       (b.kind == ConvexShape::Kind::Sphere && b.radius <= 1e-9);
    if (!probe && a.kind == ConvexShape::Kind::Sphere && b.kind == ConvexShape::Kind::Sphere) {
        const double centerDistance = length(a.center - b.center);
        hit.volume = sphereIntersectionVolume(a.radius, b.radius, centerDistance);
        hit.volumeEstimated = false;
    } else if (!probe && (hit.penetration > 0 || hit.distance <= 0)) {
        hit.volume = estimatedVolume(a, b);
        hit.volumeEstimated = true;
    }
    return hit;
}

SweepHit sweepConvex(const ConvexShape& moving, Vec3 targetCenter, const ConvexShape& fixed) {
    SweepHit sweep;
    ConvexShape posed = moving;
    const Vec3 start = moving.center;
    const Vec3 delta = targetCenter - start;
    const double path = length(delta);
    if (path < 1e-9) {
        const ContactHit hit = queryConvex(posed, fixed);
        sweep.hit = hit.distance <= 1e-4;
        sweep.time = 0;
        sweep.point = hit.pointA;
        return sweep;
    }
    double t = 0;
    for (int iter = 0; iter < 32; ++iter) {
        posed.center = start + delta * t;
        if (posed.useMatrix) {
            posed.world(3, 0) = moving.world(3, 0) + delta.x * t;
            posed.world(3, 1) = moving.world(3, 1) + delta.y * t;
            posed.world(3, 2) = moving.world(3, 2) + delta.z * t;
        }
        const ContactHit hit = queryConvex(posed, fixed);
        if (hit.distance <= 1e-3) {
            sweep.hit = true;
            sweep.time = t;
            sweep.point = hit.pointA;
            return sweep;
        }
        const double step = hit.distance / path;
        if (t + step >= 1.0 - 1e-9) {
            sweep.hit = false;
            sweep.time = 1;
            return sweep;
        }
        t += step * 0.99;
        if (step < 1e-8) {
            sweep.hit = true;
            sweep.time = t;
            sweep.point = hit.pointA;
            return sweep;
        }
    }
    sweep.hit = true;
    sweep.time = t;
    return sweep;
}

}  // namespace sim
