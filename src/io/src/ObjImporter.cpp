#include "sim/io/ObjImporter.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace sim {
namespace {

struct ObjVertex {
    int position = 0;
    int normal = 0;
};

ObjVertex parseCorner(const std::string& token) {
    ObjVertex corner;
    std::stringstream stream(token);
    std::string item;
    int field = 0;
    while (std::getline(stream, item, '/')) {
        if (!item.empty()) {
            int value = 0;
            try {
                value = std::stoi(item);
            } catch (const std::exception&) {
                value = 0;
            }
            if (field == 0) {
                corner.position = value;
            } else if (field == 2) {
                corner.normal = value;
            }
        }
        ++field;
    }
    return corner;
}

int resolveIndex(int index, int count) {
    if (index > 0) {
        return index - 1;
    }
    if (index < 0) {
        return count + index;
    }
    return -1;
}

}  // namespace

ImportResult ObjImporter::importFile(const std::string& path, Scene& scene) {
    ImportResult result;
    std::ifstream input(path);
    if (!input) {
        result.message = "cannot open " + path;
        return result;
    }

    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<ObjVertex> face;
    auto mesh = std::make_shared<MeshData>();
    std::string objectName = std::filesystem::path(path).stem().string();

    auto flushFace = [&] {
        if (face.size() < 3) {
            face.clear();
            return;
        }
        for (std::size_t i = 1; i + 1 < face.size(); ++i) {
            const ObjVertex corners[3] = {face[0], face[i], face[i + 1]};
            Vec3 points[3];
            bool ok = true;
            for (int c = 0; c < 3; ++c) {
                const int index = resolveIndex(corners[c].position, static_cast<int>(positions.size()));
                if (index < 0 || index >= static_cast<int>(positions.size())) {
                    ok = false;
                    break;
                }
                points[c] = positions[static_cast<std::size_t>(index)];
            }
            if (!ok) {
                continue;
            }
            Vec3 normal = cross(points[1] - points[0], points[2] - points[0]);
            if (length(normal) < 1e-12) {
                continue;
            }
            normal = normalize(normal);
            const auto base = static_cast<std::uint32_t>(mesh->vertexCount());
            for (int c = 0; c < 3; ++c) {
                const int normalIndex = resolveIndex(corners[c].normal, static_cast<int>(normals.size()));
                const Vec3 n = normalIndex >= 0 && normalIndex < static_cast<int>(normals.size())
                                   ? normalize(normals[static_cast<std::size_t>(normalIndex)])
                                   : normal;
                mesh->positions.push_back(static_cast<float>(points[c].x));
                mesh->positions.push_back(static_cast<float>(points[c].y));
                mesh->positions.push_back(static_cast<float>(points[c].z));
                mesh->normals.push_back(static_cast<float>(n.x));
                mesh->normals.push_back(static_cast<float>(n.y));
                mesh->normals.push_back(static_cast<float>(n.z));
            }
            mesh->indices.push_back(base);
            mesh->indices.push_back(base + 1);
            mesh->indices.push_back(base + 2);
        }
        face.clear();
    };

    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream stream(line);
        std::string tag;
        stream >> tag;
        if (tag == "o" || tag == "g") {
            std::string name;
            stream >> name;
            if (!name.empty()) {
                objectName = name;
            }
        } else if (tag == "v") {
            double x = 0, y = 0, z = 0;
            stream >> x >> y >> z;
            positions.push_back({x, y, z});
        } else if (tag == "vn") {
            double x = 0, y = 0, z = 0;
            stream >> x >> y >> z;
            normals.push_back({x, y, z});
        } else if (tag == "f") {
            face.clear();
            std::string token;
            while (stream >> token) {
                face.push_back(parseCorner(token));
            }
            flushFace();
        }
    }

    if (mesh->empty()) {
        result.message = "OBJ file has no triangles";
        return result;
    }
    mesh->recomputeBounds();
    scene.beginEdit();
    auto part = scene.makeNode(NodeKind::Part, objectName);
    part->meshId = scene.addMesh(mesh);
    part->color = {0.62, 0.66, 0.74};
    scene.addChild(scene.root(), part);
    scene.name = objectName;
    scene.sourceFiles.push_back(path);
    scene.endEdit();
    result.ok = true;
    result.nodeCount = 1;
    result.meshCount = 1;
    result.triangleCount = static_cast<int>(mesh->triangleCount());
    result.message = "imported";
    classifyImportResult(result);
    return result;
}

}  // namespace sim
