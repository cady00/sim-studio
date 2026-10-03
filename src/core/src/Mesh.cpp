#include "sim/core/Mesh.hpp"

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

void MeshData::recomputeBounds() {
    bounds = {};
    for (std::size_t i = 0; i + 2 < positions.size(); i += 3) {
        bounds.expand(Vec3{positions[i], positions[i + 1], positions[i + 2]});
    }
}

}  // namespace sim
