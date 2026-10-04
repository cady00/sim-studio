#include "sim/core/Mesh.hpp"

#include <cmath>
#include <unordered_map>

namespace sim {
namespace {

void pushVertex(MeshData& mesh, Vec3 position, Vec3 normal) {
    mesh.positions.push_back(static_cast<float>(position.x));
    mesh.positions.push_back(static_cast<float>(position.y));
    mesh.positions.push_back(static_cast<float>(position.z));
    mesh.normals.push_back(static_cast<float>(normal.x));
    mesh.normals.push_back(static_cast<float>(normal.y));
    mesh.normals.push_back(static_cast<float>(normal.z));
}

void addQuad(MeshData& mesh, Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
    const Vec3 normal = normalize(cross(b - a, d - a));
    const auto base = static_cast<std::uint32_t>(mesh.vertexCount());
    pushVertex(mesh, a, normal);
    pushVertex(mesh, b, normal);
    pushVertex(mesh, c, normal);
    pushVertex(mesh, d, normal);
    mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

}  // namespace

std::shared_ptr<MeshData> makeBoxMesh(Vec3 halfExtents) {
    const Vec3 h{std::abs(halfExtents.x), std::abs(halfExtents.y), std::abs(halfExtents.z)};
    auto mesh = std::make_shared<MeshData>();
    const Vec3 p000{-h.x, -h.y, -h.z};
    const Vec3 p100{h.x, -h.y, -h.z};
    const Vec3 p110{h.x, h.y, -h.z};
    const Vec3 p010{-h.x, h.y, -h.z};
    const Vec3 p001{-h.x, -h.y, h.z};
    const Vec3 p101{h.x, -h.y, h.z};
    const Vec3 p111{h.x, h.y, h.z};
    const Vec3 p011{-h.x, h.y, h.z};
    addQuad(*mesh, p001, p101, p111, p011);
    addQuad(*mesh, p100, p000, p010, p110);
    addQuad(*mesh, p000, p001, p011, p010);
    addQuad(*mesh, p101, p100, p110, p111);
    addQuad(*mesh, p010, p011, p111, p110);
    addQuad(*mesh, p000, p100, p101, p001);
    mesh->recomputeBounds();
    return mesh;
}

std::shared_ptr<MeshData> makeTriangleGrid(int triangles, double size) {
    const int count = std::max(2, triangles + (triangles % 2));
    const int quads = count / 2;
    const int side = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(quads)))));
    auto mesh = std::make_shared<MeshData>();
    const double cell = size / static_cast<double>(side);
    for (int q = 0; q < quads; ++q) {
        const int column = q % side;
        const int row = q / side;
        const double x = column * cell;
        const double y = row * cell;
        addQuad(*mesh, {x, y, 0}, {x + cell, y, 0}, {x + cell, y + cell, 0}, {x, y + cell, 0});
    }
    mesh->recomputeBounds();
    return mesh;
}

std::shared_ptr<MeshData> simplifyMesh(const MeshData& source, double cell) {
    auto mesh = std::make_shared<MeshData>();
    if (source.triangleCount() == 0 || cell <= 1e-9) {
        *mesh = source;
        return mesh;
    }
    struct Key {
        int x = 0;
        int y = 0;
        int z = 0;
        bool operator==(const Key& other) const { return x == other.x && y == other.y && z == other.z; }
    };
    struct Hash {
        std::size_t operator()(const Key& key) const {
            return (static_cast<std::size_t>(key.x) * 73856093u) ^ (static_cast<std::size_t>(key.y) * 19349663u) ^
                   (static_cast<std::size_t>(key.z) * 83492791u);
        }
    };
    std::unordered_map<Key, std::uint32_t, Hash> indexOf;
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    auto quantize = [&](Vec3 point, Vec3 normal) -> std::uint32_t {
        Key key{static_cast<int>(std::floor(point.x / cell)), static_cast<int>(std::floor(point.y / cell)),
                static_cast<int>(std::floor(point.z / cell))};
        const auto it = indexOf.find(key);
        if (it != indexOf.end()) {
            normals[it->second] = normals[it->second] + normal;
            return it->second;
        }
        const auto id = static_cast<std::uint32_t>(positions.size());
        indexOf.emplace(key, id);
        positions.push_back({(key.x + 0.5) * cell, (key.y + 0.5) * cell, (key.z + 0.5) * cell});
        normals.push_back(normal);
        return id;
    };
    for (std::size_t t = 0; t + 2 < source.indices.size(); t += 3) {
        std::uint32_t ids[3];
        Vec3 corners[3];
        for (int c = 0; c < 3; ++c) {
            const std::uint32_t index = source.indices[t + static_cast<std::size_t>(c)];
            corners[c] = {source.positions[index * 3], source.positions[index * 3 + 1], source.positions[index * 3 + 2]};
        }
        Vec3 normal = cross(corners[1] - corners[0], corners[2] - corners[0]);
        if (length(normal) < 1e-12) {
            continue;
        }
        normal = normalize(normal);
        for (int c = 0; c < 3; ++c) {
            ids[c] = quantize(corners[c], normal);
        }
        if (ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2]) {
            continue;
        }
        mesh->indices.push_back(ids[0]);
        mesh->indices.push_back(ids[1]);
        mesh->indices.push_back(ids[2]);
    }
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const Vec3 normal = normalize(normals[i]);
        mesh->positions.push_back(static_cast<float>(positions[i].x));
        mesh->positions.push_back(static_cast<float>(positions[i].y));
        mesh->positions.push_back(static_cast<float>(positions[i].z));
        mesh->normals.push_back(static_cast<float>(normal.x));
        mesh->normals.push_back(static_cast<float>(normal.y));
        mesh->normals.push_back(static_cast<float>(normal.z));
    }
    if (mesh->empty()) {
        return makeBoxMesh(source.bounds.valid ? source.bounds.size() * 0.5 : Vec3{1, 1, 1});
    }
    mesh->recomputeBounds();
    return mesh;
}

void MeshData::recomputeBounds() {
    bounds = {};
    for (std::size_t i = 0; i + 2 < positions.size(); i += 3) {
        bounds.expand(Vec3{positions[i], positions[i + 1], positions[i + 2]});
    }
}

}  // namespace sim
