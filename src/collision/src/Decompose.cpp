#include "sim/collision/Query.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace sim {
namespace {

struct Face {
    int a = 0;
    int b = 0;
    int c = 0;
};

std::vector<Vec3> uniquePoints(const std::vector<Vec3>& points) {
    std::vector<Vec3> out;
    for (const Vec3& point : points) {
        bool found = false;
        for (const Vec3& existing : out) {
            if (length(point - existing) < 1e-4) {
                found = true;
                break;
            }
        }
        if (!found) {
            out.push_back(point);
        }
    }
    return out;
}

std::vector<Vec3> convexHull(const std::vector<Vec3>& input) {
    std::vector<Vec3> points = uniquePoints(input);
    if (points.size() <= 4) {
        return points;
    }
    int i0 = 0;
    int i1 = 1;
    int i2 = -1;
    int i3 = -1;
    double bestEdge = 0;
    for (int i = 1; i < static_cast<int>(points.size()); ++i) {
        const double d = length(points[static_cast<std::size_t>(i)] - points[0]);
        if (d > bestEdge) {
            bestEdge = d;
            i1 = i;
        }
    }
    double bestArea = 0;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        if (i == i0 || i == i1) {
            continue;
        }
        const double area = length(cross(points[static_cast<std::size_t>(i1)] - points[0],
                                         points[static_cast<std::size_t>(i)] - points[0]));
        if (area > bestArea) {
            bestArea = area;
            i2 = i;
        }
    }
    double bestVol = 0;
    const Vec3 edge = points[static_cast<std::size_t>(i1)] - points[0];
    const Vec3 edge2 = i2 >= 0 ? points[static_cast<std::size_t>(i2)] - points[0] : Vec3{};
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        if (i == i0 || i == i1 || i == i2) {
            continue;
        }
        const double vol = std::abs(dot(points[static_cast<std::size_t>(i)] - points[0], cross(edge, edge2)));
        if (vol > bestVol) {
            bestVol = vol;
            i3 = i;
        }
    }
    if (i2 < 0 || i3 < 0 || bestVol < 1e-6) {
        return points;
    }
    std::vector<Vec3> hullPoints = {points[static_cast<std::size_t>(i0)], points[static_cast<std::size_t>(i1)],
                                    points[static_cast<std::size_t>(i2)], points[static_cast<std::size_t>(i3)]};
    std::vector<char> used(points.size(), 0);
    used[static_cast<std::size_t>(i0)] = 1;
    used[static_cast<std::size_t>(i1)] = 1;
    used[static_cast<std::size_t>(i2)] = 1;
    used[static_cast<std::size_t>(i3)] = 1;
    const Vec3 interior = (hullPoints[0] + hullPoints[1] + hullPoints[2] + hullPoints[3]) * 0.25;
    auto orient = [&](Face& face) {
        Vec3 normal = cross(hullPoints[static_cast<std::size_t>(face.b)] - hullPoints[static_cast<std::size_t>(face.a)],
                            hullPoints[static_cast<std::size_t>(face.c)] - hullPoints[static_cast<std::size_t>(face.a)]);
        if (dot(normal, interior - hullPoints[static_cast<std::size_t>(face.a)]) > 0) {
            std::swap(face.b, face.c);
        }
    };
    std::vector<Face> faces = {{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2}};
    for (Face& face : faces) {
        orient(face);
    }
    for (int pi = 0; pi < static_cast<int>(points.size()); ++pi) {
        if (used[static_cast<std::size_t>(pi)]) {
            continue;
        }
        const Vec3 point = points[static_cast<std::size_t>(pi)];
        std::vector<int> visible;
        for (int fi = 0; fi < static_cast<int>(faces.size()); ++fi) {
            const Face& face = faces[static_cast<std::size_t>(fi)];
            const Vec3 a = hullPoints[static_cast<std::size_t>(face.a)];
            const Vec3 normal = cross(hullPoints[static_cast<std::size_t>(face.b)] - a, hullPoints[static_cast<std::size_t>(face.c)] - a);
            if (dot(normal, point - a) > 1e-7) {
                visible.push_back(fi);
            }
        }
        if (visible.empty()) {
            continue;
        }
        std::vector<std::pair<int, int>> horizon;
        auto addEdge = [&](int a, int b) {
            for (auto it = horizon.begin(); it != horizon.end(); ++it) {
                if (it->first == b && it->second == a) {
                    horizon.erase(it);
                    return;
                }
            }
            horizon.emplace_back(a, b);
        };
        std::vector<char> drop(faces.size(), 0);
        for (int fi : visible) {
            drop[static_cast<std::size_t>(fi)] = 1;
            const Face& face = faces[static_cast<std::size_t>(fi)];
            addEdge(face.a, face.b);
            addEdge(face.b, face.c);
            addEdge(face.c, face.a);
        }
        std::vector<Face> kept;
        for (int fi = 0; fi < static_cast<int>(faces.size()); ++fi) {
            if (!drop[static_cast<std::size_t>(fi)]) {
                kept.push_back(faces[static_cast<std::size_t>(fi)]);
            }
        }
        const int index = static_cast<int>(hullPoints.size());
        hullPoints.push_back(point);
        used[static_cast<std::size_t>(pi)] = 1;
        for (const auto& edge : horizon) {
            Face face{edge.first, edge.second, index};
            orient(face);
            kept.push_back(face);
        }
        faces = std::move(kept);
    }
    return uniquePoints(hullPoints);
}

bool axisBox(const MeshData& mesh) {
    if (mesh.triangleCount() != 12 || !mesh.bounds.valid) {
        return false;
    }
    const Vec3 lo = mesh.bounds.min;
    const Vec3 hi = mesh.bounds.max;
    const auto on = [](float value, double a, double b) {
        return std::abs(static_cast<double>(value) - a) < 1e-3 || std::abs(static_cast<double>(value) - b) < 1e-3;
    };
    for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        if (!on(mesh.positions[i], lo.x, hi.x) || !on(mesh.positions[i + 1], lo.y, hi.y) ||
            !on(mesh.positions[i + 2], lo.z, hi.z)) {
            return false;
        }
    }
    return true;
}

bool rayTriangle(Vec3 origin, Vec3 direction, Vec3 v0, Vec3 v1, Vec3 v2, double& t) {
    const Vec3 e1 = v1 - v0;
    const Vec3 e2 = v2 - v0;
    const Vec3 p = cross(direction, e2);
    const double det = dot(e1, p);
    if (std::abs(det) < 1e-10) {
        return false;
    }
    const double inv = 1.0 / det;
    const Vec3 s = origin - v0;
    const double u = dot(s, p) * inv;
    if (u < -1e-6 || u > 1.0 + 1e-6) {
        return false;
    }
    const Vec3 q = cross(s, e1);
    const double v = dot(direction, q) * inv;
    if (v < -1e-6 || u + v > 1.0 + 1e-6) {
        return false;
    }
    t = dot(e2, q) * inv;
    return t > 1e-6;
}

int indexOf(int x, int y, int z, int n) { return (z * n + y) * n + x; }

}  // namespace

std::uint64_t meshFingerprint(const MeshData& mesh) {
    std::uint64_t hash = 14695981039346656037ull;
    auto mix = [&](std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ull;
    };
    mix(mesh.positions.size());
    mix(mesh.indices.size());
    const std::size_t stride = std::max<std::size_t>(1, mesh.positions.size() / 32);
    for (std::size_t i = 0; i < mesh.positions.size(); i += stride) {
        mix(static_cast<std::uint64_t>(std::llround(static_cast<double>(mesh.positions[i]) * 1000.0)));
    }
    return hash;
}

std::vector<std::vector<Vec3>> decomposeMesh(const MeshData& mesh) {
    if (mesh.empty() || !mesh.bounds.valid) {
        return {};
    }
    if (axisBox(mesh)) {
        const Vec3 lo = mesh.bounds.min;
        const Vec3 hi = mesh.bounds.max;
        return {{{lo.x, lo.y, lo.z},
                 {hi.x, lo.y, lo.z},
                 {hi.x, hi.y, lo.z},
                 {lo.x, hi.y, lo.z},
                 {lo.x, lo.y, hi.z},
                 {hi.x, lo.y, hi.z},
                 {hi.x, hi.y, hi.z},
                 {lo.x, hi.y, hi.z}}};
    }

    constexpr int n = 16;
    const Vec3 lo = mesh.bounds.min;
    const Vec3 hi = mesh.bounds.max;
    const Vec3 extent = hi - lo;
    const double cell = std::max(extent.x, std::max(extent.y, extent.z)) / static_cast<double>(n);
    if (cell < 1e-6) {
        return {};
    }
    std::vector<Vec3> triangles;
    triangles.reserve(mesh.indices.size());
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t index = mesh.indices[i + static_cast<std::size_t>(k)];
            const std::size_t base = static_cast<std::size_t>(index) * 3;
            if (base + 2 >= mesh.positions.size()) {
                triangles.push_back({});
            } else {
                triangles.push_back({mesh.positions[base], mesh.positions[base + 1], mesh.positions[base + 2]});
            }
        }
    }
    auto inside = [&](Vec3 point) {
        const Vec3 origin = point + Vec3{0, 1e-4 * cell, 2e-4 * cell};
        int hits = 0;
        for (std::size_t i = 0; i + 2 < triangles.size(); i += 3) {
            double t = 0;
            if (rayTriangle(origin, {1, 0, 0}, triangles[i], triangles[i + 1], triangles[i + 2], t)) {
                ++hits;
            }
        }
        return hits % 2 == 1;
    };

    std::vector<int> occupied;
    occupied.reserve(static_cast<std::size_t>(n * n));
    for (int z = 0; z < n; ++z) {
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const Vec3 point = lo + Vec3{(x + 0.5) * cell, (y + 0.5) * cell, (z + 0.5) * cell};
                if (point.x > hi.x + cell || point.y > hi.y + cell || point.z > hi.z + cell) {
                    continue;
                }
                if (inside(point)) {
                    occupied.push_back(indexOf(x, y, z, n));
                }
            }
        }
    }
    if (occupied.empty()) {
        std::vector<Vec3> vertices;
        for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
            vertices.push_back({mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]});
        }
        auto hull = convexHull(vertices);
        return hull.empty() ? std::vector<std::vector<Vec3>>{} : std::vector<std::vector<Vec3>>{std::move(hull)};
    }

    std::vector<std::vector<Vec3>> pieces;
    auto emit = [&](const std::vector<int>& voxels) {
        std::vector<Vec3> samples;
        for (int id : voxels) {
            const int x = id % n;
            const int y = (id / n) % n;
            const int z = id / (n * n);
            samples.push_back(lo + Vec3{(x + 0.5) * cell, (y + 0.5) * cell, (z + 0.5) * cell});
        }
        std::vector<Vec3> vertices;
        for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
            const Vec3 vertex{mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]};
            const int x = std::clamp(static_cast<int>((vertex.x - lo.x) / cell), 0, n - 1);
            const int y = std::clamp(static_cast<int>((vertex.y - lo.y) / cell), 0, n - 1);
            const int z = std::clamp(static_cast<int>((vertex.z - lo.z) / cell), 0, n - 1);
            const int id = indexOf(x, y, z, n);
            if (std::find(voxels.begin(), voxels.end(), id) != voxels.end()) {
                vertices.push_back(vertex);
            }
        }
        if (vertices.size() < 4) {
            vertices = samples;
        }
        auto hull = convexHull(vertices);
        if (hull.size() >= 4) {
            pieces.push_back(std::move(hull));
        }
    };

    std::vector<std::pair<std::vector<int>, int>> stack;
    stack.push_back({occupied, 0});
    while (!stack.empty() && pieces.size() < 8) {
        auto [voxels, depth] = std::move(stack.back());
        stack.pop_back();
        if (voxels.size() < 8 || depth >= 4) {
            emit(voxels);
            continue;
        }
        int minX = n, minY = n, minZ = n, maxX = 0, maxY = 0, maxZ = 0;
        for (int id : voxels) {
            const int x = id % n;
            const int y = (id / n) % n;
            const int z = id / (n * n);
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            minZ = std::min(minZ, z);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
            maxZ = std::max(maxZ, z);
        }
        const int spanX = maxX - minX;
        const int spanY = maxY - minY;
        const int spanZ = maxZ - minZ;
        int axis = 0;
        if (spanY >= spanX && spanY >= spanZ) {
            axis = 1;
        } else if (spanZ >= spanX && spanZ >= spanY) {
            axis = 2;
        }
        std::vector<int> coords;
        coords.reserve(voxels.size());
        for (int id : voxels) {
            const int x = id % n;
            const int y = (id / n) % n;
            const int z = id / (n * n);
            coords.push_back(axis == 0 ? x : axis == 1 ? y : z);
        }
        std::vector<int> order = coords;
        std::nth_element(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(order.size() / 2), order.end());
        const int mid = order[order.size() / 2];
        std::vector<int> left;
        std::vector<int> right;
        for (std::size_t i = 0; i < voxels.size(); ++i) {
            if (coords[i] <= mid) {
                left.push_back(voxels[i]);
            } else {
                right.push_back(voxels[i]);
            }
        }
        if (left.empty() || right.empty() || left.size() == voxels.size() || right.size() == voxels.size()) {
            emit(voxels);
            continue;
        }
        const double filled = static_cast<double>(voxels.size());
        const double box = static_cast<double>(spanX + 1) * static_cast<double>(spanY + 1) * static_cast<double>(spanZ + 1);
        if (filled > box * 0.92) {
            emit(voxels);
            continue;
        }
        stack.push_back({std::move(left), depth + 1});
        stack.push_back({std::move(right), depth + 1});
    }
    if (pieces.empty()) {
        emit(occupied);
    }
    return pieces;
}

}  // namespace sim
