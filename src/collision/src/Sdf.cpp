#include "sim/collision/Query.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

double sdBox(Vec3 local, Vec3 half) {
    const Vec3 q{std::abs(local.x) - half.x, std::abs(local.y) - half.y, std::abs(local.z) - half.z};
    const double outside = length(Vec3{std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)});
    const double inside = std::min(std::max({q.x, q.y, q.z}), 0.0);
    return outside + inside;
}

double signedPiece(const ConvexShape& piece, Vec3 point) {
    if (piece.kind == ConvexShape::Kind::Sphere) {
        return length(point - piece.center) - piece.radius;
    }
    if (piece.kind == ConvexShape::Kind::Box) {
        Vec3 sample = point;
        Vec3 center = piece.center;
        Vec3 half = piece.half;
        Quat rotation = piece.rotation;
        if (piece.useMatrix) {
            sample = transformPoint(inverseAffine(piece.world), point);
            center = piece.localCenter;
            half = piece.localHalf;
            rotation = Quat::identity();
        }
        return sdBox(rotation.conjugate().rotate(sample - center), half);
    }
    if (piece.kind == ConvexShape::Kind::Capsule) {
        const double halfCylinder = std::max(0.0, piece.height * 0.5 - piece.radius);
        const Vec3 axis = normalize(piece.rotation.rotate(piece.axis));
        const Vec3 pa = piece.center - axis * halfCylinder;
        const Vec3 pb = piece.center + axis * halfCylinder;
        const Vec3 ab = pb - pa;
        const double denom = dot(ab, ab);
        const double t = denom > 1e-20 ? std::clamp(dot(point - pa, ab) / denom, 0.0, 1.0) : 0.0;
        return length(point - (pa + ab * t)) - piece.radius;
    }
    ConvexShape probe;
    probe.kind = ConvexShape::Kind::Sphere;
    probe.center = point;
    probe.radius = 0;
    return queryConvex(probe, piece).distance;
}

}  // namespace

SdfGrid buildSdf(const std::vector<ConvexShape>& pieces, int resolution) {
    SdfGrid grid;
    if (pieces.empty()) {
        return grid;
    }
    resolution = std::clamp(resolution, 4, 24);
    Aabb bounds;
    for (const ConvexShape& piece : pieces) {
        if (piece.kind == ConvexShape::Kind::Sphere) {
            bounds.expand(piece.center - Vec3{piece.radius, piece.radius, piece.radius});
            bounds.expand(piece.center + Vec3{piece.radius, piece.radius, piece.radius});
            continue;
        }
        if (piece.kind == ConvexShape::Kind::Box && !piece.useMatrix) {
            for (int i = 0; i < 8; ++i) {
                const Vec3 local{(i & 1) ? piece.half.x : -piece.half.x, (i & 2) ? piece.half.y : -piece.half.y,
                                 (i & 4) ? piece.half.z : -piece.half.z};
                bounds.expand(piece.center + piece.rotation.rotate(local));
            }
            continue;
        }
        for (const Vec3& point : piece.localPoints) {
            bounds.expand(piece.useMatrix ? transformPoint(piece.world, point) : piece.center + piece.rotation.rotate(point));
        }
        if (piece.localPoints.empty()) {
            bounds.expand(piece.center);
        }
    }
    if (!bounds.valid) {
        return grid;
    }
    const Vec3 size = bounds.size();
    const double pad = std::max(1.0, 0.15 * std::max(size.x, std::max(size.y, size.z)));
    const Vec3 lo = bounds.min - Vec3{pad, pad, pad};
    const Vec3 hi = bounds.max + Vec3{pad, pad, pad};
    const Vec3 span = hi - lo;
    const double cell = std::max(span.x, std::max(span.y, span.z)) / static_cast<double>(resolution);
    if (cell < 1e-6) {
        return grid;
    }
    grid.origin = lo;
    grid.cell = cell;
    grid.nx = std::max(1, static_cast<int>(std::ceil(span.x / cell)));
    grid.ny = std::max(1, static_cast<int>(std::ceil(span.y / cell)));
    grid.nz = std::max(1, static_cast<int>(std::ceil(span.z / cell)));
    grid.nx = std::min(grid.nx, resolution);
    grid.ny = std::min(grid.ny, resolution);
    grid.nz = std::min(grid.nz, resolution);
    grid.band = 3.0 * cell;
    grid.values.assign(static_cast<std::size_t>(grid.nx * grid.ny * grid.nz), 0.f);
    for (int z = 0; z < grid.nz; ++z) {
        for (int y = 0; y < grid.ny; ++y) {
            for (int x = 0; x < grid.nx; ++x) {
                const Vec3 point = lo + Vec3{(x + 0.5) * cell, (y + 0.5) * cell, (z + 0.5) * cell};
                double best = 1e300;
                for (const ConvexShape& piece : pieces) {
                    best = std::min(best, signedPiece(piece, point));
                }
                if (std::abs(best) > grid.band) {
                    best = std::copysign(grid.band, best == 0 ? 1.0 : best);
                }
                grid.values[static_cast<std::size_t>((z * grid.ny + y) * grid.nx + x)] = static_cast<float>(best);
            }
        }
    }
    return grid;
}

bool sampleSdf(const SdfGrid& grid, Vec3 point, double& distance, Vec3& gradient) {
    if (grid.values.empty() || grid.cell <= 0 || grid.nx < 1) {
        return false;
    }
    const auto at = [&](int x, int y, int z) {
        x = std::clamp(x, 0, grid.nx - 1);
        y = std::clamp(y, 0, grid.ny - 1);
        z = std::clamp(z, 0, grid.nz - 1);
        return static_cast<double>(grid.values[static_cast<std::size_t>((z * grid.ny + y) * grid.nx + x)]);
    };
    const Vec3 local = (point - grid.origin) / grid.cell - Vec3{0.5, 0.5, 0.5};
    const int x0 = static_cast<int>(std::floor(local.x));
    const int y0 = static_cast<int>(std::floor(local.y));
    const int z0 = static_cast<int>(std::floor(local.z));
    const double tx = local.x - x0;
    const double ty = local.y - y0;
    const double tz = local.z - z0;
    const auto lerp = [](double a, double b, double t) { return a + (b - a) * t; };
    const double c000 = at(x0, y0, z0);
    const double c100 = at(x0 + 1, y0, z0);
    const double c010 = at(x0, y0 + 1, z0);
    const double c110 = at(x0 + 1, y0 + 1, z0);
    const double c001 = at(x0, y0, z0 + 1);
    const double c101 = at(x0 + 1, y0, z0 + 1);
    const double c011 = at(x0, y0 + 1, z0 + 1);
    const double c111 = at(x0 + 1, y0 + 1, z0 + 1);
    const double c00 = lerp(c000, c100, tx);
    const double c10 = lerp(c010, c110, tx);
    const double c01 = lerp(c001, c101, tx);
    const double c11 = lerp(c011, c111, tx);
    distance = lerp(lerp(c00, c10, ty), lerp(c01, c11, ty), tz);
    const double step = grid.cell;
    const double dx = (at(x0 + 1, y0, z0) - at(x0 - 1, y0, z0)) / (2.0 * step);
    const double dy = (at(x0, y0 + 1, z0) - at(x0, y0 - 1, z0)) / (2.0 * step);
    const double dz = (at(x0, y0, z0 + 1) - at(x0, y0, z0 - 1)) / (2.0 * step);
    gradient = normalize(Vec3{dx, dy, dz});
    if (length(Vec3{dx, dy, dz}) < 1e-9) {
        gradient = {1, 0, 0};
    }
    return true;
}

}  // namespace sim
