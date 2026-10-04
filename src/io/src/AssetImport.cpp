#include "sim/io/AssetImport.hpp"

#include "sim/core/LineModel.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/core/Xml.hpp"
#include "sim/io/ObjImporter.hpp"
#include "sim/io/StepImporter.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace sim {
namespace {

std::string lowerCopy(std::string text) {
    for (char& c : text) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return text;
}

std::string extensionOf(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    const std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
    const auto dot = name.find_last_of('.');
    if (dot == std::string::npos) {
        return {};
    }
    return lowerCopy(name.substr(dot + 1));
}

std::string readFile(const std::string& path, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot open " + path;
        return {};
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    error.clear();
    return buffer.str();
}

void addTriangle(MeshData& mesh, Vec3 p0, Vec3 p1, Vec3 p2) {
    Vec3 normal = cross(p1 - p0, p2 - p0);
    const double span = length(normal);
    if (span < 1e-9) {
        return;
    }
    normal = normal / span;
    const auto base = static_cast<std::uint32_t>(mesh.vertexCount());
    const Vec3 points[3] = {p0, p1, p2};
    for (const Vec3& point : points) {
        mesh.positions.push_back(static_cast<float>(point.x));
        mesh.positions.push_back(static_cast<float>(point.y));
        mesh.positions.push_back(static_cast<float>(point.z));
        mesh.normals.push_back(static_cast<float>(normal.x));
        mesh.normals.push_back(static_cast<float>(normal.y));
        mesh.normals.push_back(static_cast<float>(normal.z));
    }
    mesh.indices.push_back(base);
    mesh.indices.push_back(base + 1);
    mesh.indices.push_back(base + 2);
}

AssetImportResult adoptMesh(Scene& scene, std::shared_ptr<MeshData> mesh, const std::string& name, const char* format) {
    AssetImportResult result;
    result.format = format;
    if (!mesh || mesh->empty()) {
        result.message = std::string(format) + " produced an empty mesh";
        return result;
    }
    mesh->recomputeBounds();
    scene.beginEdit();
    auto node = scene.makeNode(NodeKind::Part, name);
    node->meshId = scene.addMesh(mesh);
    node->color = {0.55, 0.62, 0.74};
    if (scene.root()) {
        scene.addChild(scene.root(), node);
    }
    scene.endEdit();
    result.ok = true;
    result.nodeCount = 1;
    result.meshCount = 1;
    result.triangleCount = static_cast<int>(mesh->triangleCount());
    result.message = "imported";
    return result;
}

int base64Value(char c) {
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
        return c - '0' + 52;
    }
    if (c == '+') {
        return 62;
    }
    if (c == '/') {
        return 63;
    }
    return -1;
}

std::vector<std::uint8_t> decodeBase64(const std::string& text) {
    std::vector<std::uint8_t> out;
    int value = 0;
    int bits = 0;
    for (char c : text) {
        if (c == '=' || c == '\n' || c == '\r' || c == ' ') {
            continue;
        }
        const int digit = base64Value(c);
        if (digit < 0) {
            continue;
        }
        value = (value << 6) | digit;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<std::uint8_t>((value >> bits) & 0xff));
        }
    }
    return out;
}

std::string encodeBase64(const std::uint8_t* data, std::size_t size) {
    static const char* alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((size + 2) / 3 * 4);
    for (std::size_t i = 0; i < size; i += 3) {
        const unsigned a = data[i];
        const unsigned b = i + 1 < size ? data[i + 1] : 0;
        const unsigned c = i + 2 < size ? data[i + 2] : 0;
        const unsigned triple = (a << 16) | (b << 8) | c;
        out.push_back(alphabet[(triple >> 18) & 63]);
        out.push_back(alphabet[(triple >> 12) & 63]);
        out.push_back(i + 1 < size ? alphabet[(triple >> 6) & 63] : '=');
        out.push_back(i + 2 < size ? alphabet[triple & 63] : '=');
    }
    return out;
}

std::shared_ptr<MeshData> meshFromStl(const std::string& bytes, std::string& error) {
    auto mesh = std::make_shared<MeshData>();
    const bool ascii = bytes.size() >= 5 && lowerCopy(bytes.substr(0, 5)) == "solid" &&
                       bytes.find("facet") != std::string::npos;
    if (ascii) {
        std::istringstream in(bytes);
        std::string word;
        Vec3 verts[3];
        int count = 0;
        while (in >> word) {
            if (word == "vertex" && count < 3) {
                in >> verts[count].x >> verts[count].y >> verts[count].z;
                ++count;
                if (count == 3) {
                    addTriangle(*mesh, verts[0], verts[1], verts[2]);
                    count = 0;
                }
            }
        }
    } else {
        if (bytes.size() < 84) {
            error = "stl is truncated";
            return {};
        }
        std::uint32_t triangles = 0;
        std::memcpy(&triangles, bytes.data() + 80, 4);
        const std::size_t need = 84ull + static_cast<std::size_t>(triangles) * 50ull;
        if (bytes.size() < need) {
            error = "stl triangle count does not match the file";
            return {};
        }
        for (std::uint32_t i = 0; i < triangles; ++i) {
            const char* face = bytes.data() + 84 + static_cast<std::size_t>(i) * 50 + 12;
            float xyz[9];
            std::memcpy(xyz, face, sizeof(xyz));
            addTriangle(*mesh, {xyz[0], xyz[1], xyz[2]}, {xyz[3], xyz[4], xyz[5]}, {xyz[6], xyz[7], xyz[8]});
        }
    }
    if (mesh->empty()) {
        error = "stl has no triangles";
        return {};
    }
    error.clear();
    return mesh;
}

std::shared_ptr<MeshData> meshFromPly(const std::string& bytes, std::string& error) {
    if (bytes.size() < 3 || bytes.compare(0, 3, "ply") != 0) {
        error = "ply header is missing";
        return {};
    }
    const auto headerEnd = bytes.find("end_header");
    if (headerEnd == std::string::npos) {
        error = "ply header is incomplete";
        return {};
    }
    const bool binary = bytes.find("binary_little_endian") != std::string::npos && bytes.find("binary_little_endian") < headerEnd;
    int vertexCount = 0;
    int faceCount = 0;
    std::istringstream header(bytes.substr(0, headerEnd));
    std::string line;
    while (std::getline(header, line)) {
        if (line.rfind("element vertex", 0) == 0) {
            vertexCount = std::stoi(line.substr(15));
        } else if (line.rfind("element face", 0) == 0) {
            faceCount = std::stoi(line.substr(13));
        }
    }
    auto mesh = std::make_shared<MeshData>();
    std::size_t cursor = headerEnd + std::string("end_header").size();
    if (cursor < bytes.size() && bytes[cursor] == '\r') {
        ++cursor;
    }
    if (cursor < bytes.size() && bytes[cursor] == '\n') {
        ++cursor;
    }
    std::vector<Vec3> vertices(static_cast<std::size_t>(std::max(vertexCount, 0)));
    if (binary) {
        for (int i = 0; i < vertexCount; ++i) {
            if (cursor + 12 > bytes.size()) {
                error = "ply vertices are truncated";
                return {};
            }
            float xyz[3];
            std::memcpy(xyz, bytes.data() + cursor, 12);
            cursor += 12;
            vertices[static_cast<std::size_t>(i)] = {xyz[0], xyz[1], xyz[2]};
        }
        for (int i = 0; i < faceCount; ++i) {
            if (cursor >= bytes.size()) {
                error = "ply faces are truncated";
                return {};
            }
            const int n = static_cast<unsigned char>(bytes[cursor++]);
            std::vector<int> index(static_cast<std::size_t>(n));
            for (int k = 0; k < n; ++k) {
                if (cursor + 4 > bytes.size()) {
                    error = "ply faces are truncated";
                    return {};
                }
                std::uint32_t value = 0;
                std::memcpy(&value, bytes.data() + cursor, 4);
                cursor += 4;
                index[static_cast<std::size_t>(k)] = static_cast<int>(value);
            }
            for (int k = 1; k + 1 < n; ++k) {
                addTriangle(*mesh, vertices[static_cast<std::size_t>(index[0])], vertices[static_cast<std::size_t>(index[k])],
                            vertices[static_cast<std::size_t>(index[k + 1])]);
            }
        }
    } else {
        std::istringstream body(bytes.substr(cursor));
        for (int i = 0; i < vertexCount; ++i) {
            body >> vertices[static_cast<std::size_t>(i)].x >> vertices[static_cast<std::size_t>(i)].y >>
                vertices[static_cast<std::size_t>(i)].z;
        }
        for (int i = 0; i < faceCount; ++i) {
            int n = 0;
            body >> n;
            std::vector<int> index(static_cast<std::size_t>(std::max(n, 0)));
            for (int k = 0; k < n; ++k) {
                body >> index[static_cast<std::size_t>(k)];
            }
            for (int k = 1; k + 1 < n; ++k) {
                addTriangle(*mesh, vertices[static_cast<std::size_t>(index[0])], vertices[static_cast<std::size_t>(index[k])],
                            vertices[static_cast<std::size_t>(index[k + 1])]);
            }
        }
    }
    if (mesh->empty()) {
        error = "ply has no triangles";
        return {};
    }
    error.clear();
    return mesh;
}

Mat4 gltfNodeMatrix(const nlohmann::json& node) {
    Mat4 matrix = Mat4::identity();
    if (node.contains("matrix") && node.at("matrix").is_array() && node.at("matrix").size() >= 16) {
        for (int i = 0; i < 16; ++i) {
            matrix.m[i] = node.at("matrix").at(i).get<double>();
        }
        return matrix;
    }
    Vec3 scale{1, 1, 1};
    Vec3 translation;
    Quat rotation = Quat::identity();
    if (node.contains("scale") && node.at("scale").size() >= 3) {
        scale = {node.at("scale")[0].get<double>(), node.at("scale")[1].get<double>(), node.at("scale")[2].get<double>()};
    }
    if (node.contains("translation") && node.at("translation").size() >= 3) {
        translation = {node.at("translation")[0].get<double>(), node.at("translation")[1].get<double>(),
                       node.at("translation")[2].get<double>()};
    }
    if (node.contains("rotation") && node.at("rotation").size() >= 4) {
        rotation.x = node.at("rotation")[0].get<double>();
        rotation.y = node.at("rotation")[1].get<double>();
        rotation.z = node.at("rotation")[2].get<double>();
        rotation.w = node.at("rotation")[3].get<double>();
        rotation = rotation.normalized();
    }
    const double x = rotation.x, y = rotation.y, z = rotation.z, w = rotation.w;
    Mat4 local = Mat4::identity();
    local(0, 0) = (1 - 2 * (y * y + z * z)) * scale.x;
    local(1, 0) = (2 * (x * y + w * z)) * scale.y;
    local(2, 0) = (2 * (x * z - w * y)) * scale.z;
    local(0, 1) = (2 * (x * y - w * z)) * scale.x;
    local(1, 1) = (1 - 2 * (x * x + z * z)) * scale.y;
    local(2, 1) = (2 * (y * z + w * x)) * scale.z;
    local(0, 2) = (2 * (x * z + w * y)) * scale.x;
    local(1, 2) = (2 * (y * z - w * x)) * scale.y;
    local(2, 2) = (1 - 2 * (x * x + y * y)) * scale.z;
    local(3, 0) = translation.x;
    local(3, 1) = translation.y;
    local(3, 2) = translation.z;
    return local;
}

bool loadAccessor(const nlohmann::json& json, const std::vector<std::vector<std::uint8_t>>& buffers, int index,
                  std::vector<double>& values, int& components, std::string& error) {
    if (!json.contains("accessors") || index < 0 || index >= static_cast<int>(json.at("accessors").size())) {
        error = "gltf accessor is missing";
        return false;
    }
    const auto& accessor = json.at("accessors").at(index);
    const int viewIndex = accessor.value("bufferView", -1);
    if (viewIndex < 0 || !json.contains("bufferViews") || viewIndex >= static_cast<int>(json.at("bufferViews").size())) {
        error = "gltf buffer view is missing";
        return false;
    }
    const auto& view = json.at("bufferViews").at(viewIndex);
    const int bufferIndex = view.value("buffer", 0);
    if (bufferIndex < 0 || bufferIndex >= static_cast<int>(buffers.size())) {
        error = "gltf buffer is missing";
        return false;
    }
    const std::string type = accessor.value("type", std::string("SCALAR"));
    components = type == "VEC3" ? 3 : type == "VEC2" ? 2 : type == "VEC4" ? 4 : 1;
    const int componentType = accessor.value("componentType", 5126);
    const int count = accessor.value("count", 0);
    std::size_t cursor = static_cast<std::size_t>(view.value("byteOffset", 0) + accessor.value("byteOffset", 0));
    const int componentBytes = componentType == 5123 ? 2 : 4;
    const std::size_t stride = static_cast<std::size_t>(view.value("byteStride", componentBytes * components));
    const auto& buffer = buffers[static_cast<std::size_t>(bufferIndex)];
    values.clear();
    values.reserve(static_cast<std::size_t>(count * components));
    for (int i = 0; i < count; ++i) {
        if (cursor + static_cast<std::size_t>(componentBytes * components) > buffer.size()) {
            error = "gltf accessor is truncated";
            return false;
        }
        for (int c = 0; c < components; ++c) {
            const std::uint8_t* ptr = buffer.data() + cursor + static_cast<std::size_t>(c * componentBytes);
            if (componentType == 5126) {
                float value = 0;
                std::memcpy(&value, ptr, 4);
                values.push_back(value);
            } else if (componentType == 5125) {
                std::uint32_t value = 0;
                std::memcpy(&value, ptr, 4);
                values.push_back(value);
            } else if (componentType == 5123) {
                std::uint16_t value = 0;
                std::memcpy(&value, ptr, 2);
                values.push_back(value);
            } else {
                error = "gltf component type is not float or an index";
                return false;
            }
        }
        cursor += stride;
    }
    error.clear();
    return true;
}

AssetImportResult importGltfBytes(const nlohmann::json& json, const std::vector<std::vector<std::uint8_t>>& buffers, Scene& scene) {
    AssetImportResult result;
    result.format = "gltf";
    if (!json.contains("meshes")) {
        result.message = "gltf has no meshes";
        return result;
    }
    auto mesh = std::make_shared<MeshData>();
    const auto visit = [&](auto&& self, int nodeIndex, const Mat4& parent) -> void {
        if (!json.contains("nodes") || nodeIndex < 0 || nodeIndex >= static_cast<int>(json.at("nodes").size())) {
            return;
        }
        const auto& node = json.at("nodes").at(nodeIndex);
        const Mat4 world = mul(parent, gltfNodeMatrix(node));
        if (node.contains("mesh")) {
            const int meshIndex = node.at("mesh").get<int>();
            const auto& primitives = json.at("meshes").at(meshIndex).at("primitives");
            for (const auto& primitive : primitives) {
                if (primitive.value("mode", 4) != 4 || !primitive.contains("attributes") ||
                    !primitive.at("attributes").contains("POSITION")) {
                    continue;
                }
                std::string error;
                std::vector<double> positions;
                int components = 0;
                if (!loadAccessor(json, buffers, primitive.at("attributes").at("POSITION").get<int>(), positions, components, error) ||
                    components != 3) {
                    continue;
                }
                std::vector<double> indices;
                int indexComponents = 0;
                const bool indexed = primitive.contains("indices") &&
                                     loadAccessor(json, buffers, primitive.at("indices").get<int>(), indices, indexComponents, error);
                const auto pointAt = [&](int vertex) {
                    const std::size_t base = static_cast<std::size_t>(vertex) * 3;
                    Vec3 point{positions[base], positions[base + 1], positions[base + 2]};
                    point = transformPoint(world, point) * 1000.0;
                    return point;
                };
                if (indexed) {
                    for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
                        addTriangle(*mesh, pointAt(static_cast<int>(indices[i])), pointAt(static_cast<int>(indices[i + 1])),
                                    pointAt(static_cast<int>(indices[i + 2])));
                    }
                } else {
                    const int count = static_cast<int>(positions.size() / 3);
                    for (int i = 0; i + 2 < count; i += 3) {
                        addTriangle(*mesh, pointAt(i), pointAt(i + 1), pointAt(i + 2));
                    }
                }
            }
        }
        if (node.contains("children")) {
            for (const auto& child : node.at("children")) {
                self(self, child.get<int>(), world);
            }
        }
    };
    std::vector<int> roots;
    if (json.contains("scenes")) {
        const int sceneIndex = json.value("scene", 0);
        if (sceneIndex >= 0 && sceneIndex < static_cast<int>(json.at("scenes").size()) &&
            json.at("scenes").at(sceneIndex).contains("nodes")) {
            for (const auto& node : json.at("scenes").at(sceneIndex).at("nodes")) {
                roots.push_back(node.get<int>());
            }
        }
    }
    if (roots.empty() && json.contains("nodes")) {
        roots.push_back(0);
    }
    for (int root : roots) {
        visit(visit, root, Mat4::identity());
    }
    return adoptMesh(scene, mesh, "gltf", "gltf");
}

AssetImportResult importGltfFile(const std::string& path, Scene& scene) {
    AssetImportResult result;
    result.format = "gltf";
    std::string error;
    const std::string bytes = readFile(path, error);
    if (!error.empty()) {
        result.message = error;
        return result;
    }
    nlohmann::json json;
    std::vector<std::vector<std::uint8_t>> buffers;
    if (bytes.size() >= 12 && bytes.compare(0, 4, "glTF") == 0) {
        std::uint32_t length = 0;
        std::memcpy(&length, bytes.data() + 8, 4);
        std::size_t cursor = 12;
        std::vector<std::uint8_t> bin;
        while (cursor + 8 <= bytes.size() && cursor < length) {
            std::uint32_t chunkLength = 0;
            std::uint32_t chunkType = 0;
            std::memcpy(&chunkLength, bytes.data() + cursor, 4);
            std::memcpy(&chunkType, bytes.data() + cursor + 4, 4);
            cursor += 8;
            if (cursor + chunkLength > bytes.size()) {
                result.message = "glb chunk is truncated";
                return result;
            }
            if (chunkType == 0x4E4F534Au) {
                json = nlohmann::json::parse(bytes.data() + cursor, bytes.data() + cursor + chunkLength, nullptr, false);
            } else if (chunkType == 0x004E4942u) {
                bin.assign(bytes.begin() + static_cast<std::ptrdiff_t>(cursor),
                           bytes.begin() + static_cast<std::ptrdiff_t>(cursor + chunkLength));
            }
            cursor += chunkLength;
        }
        if (json.is_discarded() || json.is_null()) {
            result.message = "glb json was not read";
            return result;
        }
        buffers.push_back(std::move(bin));
    } else {
        json = nlohmann::json::parse(bytes, nullptr, false);
        if (json.is_discarded()) {
            result.message = "gltf json was not read";
            return result;
        }
        if (json.contains("buffers")) {
            const auto slash = path.find_last_of("/\\");
            const std::string dir = slash == std::string::npos ? std::string() : path.substr(0, slash + 1);
            for (const auto& buffer : json.at("buffers")) {
                const std::string uri = buffer.value("uri", std::string{});
                const auto comma = uri.find(',');
                if (uri.rfind("data:", 0) == 0 && comma != std::string::npos) {
                    buffers.push_back(decodeBase64(uri.substr(comma + 1)));
                } else if (!uri.empty()) {
                    std::string fileError;
                    const std::string file = readFile(dir + uri, fileError);
                    buffers.emplace_back(file.begin(), file.end());
                } else {
                    buffers.emplace_back();
                }
            }
        }
    }
    return importGltfBytes(json, buffers, scene);
}

std::vector<double> splitNumbers(const std::string& text) {
    std::vector<double> values;
    std::istringstream in(text);
    double value = 0;
    while (in >> value) {
        values.push_back(value);
    }
    return values;
}

const XmlNode* childNamed(const XmlNode& node, const std::string& name) {
    for (const XmlNode& child : node.children) {
        std::string local = child.name;
        const auto colon = local.rfind(':');
        if (colon != std::string::npos) {
            local = local.substr(colon + 1);
        }
        if (lowerCopy(local) == name) {
            return &child;
        }
    }
    return nullptr;
}

AssetImportResult importUrdfText(const std::string& text, Scene& scene) {
    AssetImportResult result;
    result.format = "urdf";
    XmlNode root;
    std::string error;
    if (!parseXml(text, root, error)) {
        result.message = error.empty() ? "urdf was not read" : error;
        return result;
    }
    const XmlNode* robot = &root;
    if (lowerCopy(root.name) != "robot") {
        robot = nullptr;
        for (const XmlNode& child : root.children) {
            if (lowerCopy(child.name) == "robot") {
                robot = &child;
                break;
            }
        }
    }
    if (robot == nullptr) {
        result.message = "urdf has no robot element";
        return result;
    }
    scene.beginEdit();
    KinematicResource resource;
    resource.name = robot->attributes.count("name") ? robot->attributes.at("name") : "robot";
    resource.ik = "numerical";
    std::unordered_map<std::string, std::shared_ptr<SceneNode>> links;
    auto linkOf = [&](const std::string& name) {
        const auto it = links.find(name);
        if (it != links.end()) {
            return it->second;
        }
        auto node = scene.makeNode(NodeKind::Assembly, name);
        if (scene.root()) {
            scene.addChild(scene.root(), node);
        }
        links.emplace(name, node);
        KinLink link;
        link.name = name;
        link.nodeId = node->id;
        resource.links.push_back(link);
        return node;
    };
    for (const XmlNode& element : robot->children) {
        if (lowerCopy(element.name) != "link") {
            continue;
        }
        const std::string name = element.attributes.count("name") ? element.attributes.at("name") : "link";
        auto node = linkOf(name);
        const XmlNode* visual = childNamed(element, "visual");
        const XmlNode* geometry = visual == nullptr ? nullptr : childNamed(*visual, "geometry");
        const XmlNode* box = geometry == nullptr ? nullptr : childNamed(*geometry, "box");
        if (box == nullptr || !box->attributes.count("size")) {
            continue;
        }
        const std::vector<double> size = splitNumbers(box->attributes.at("size"));
        if (size.size() < 3) {
            continue;
        }
        const Vec3 half{size[0] * 500.0, size[1] * 500.0, size[2] * 500.0};
        auto mesh = makeBoxMesh(half);
        auto visualNode = scene.makeNode(NodeKind::Part, name + "_visual");
        visualNode->meshId = scene.addMesh(mesh);
        visualNode->color = {0.45, 0.62, 0.84};
        if (visual != nullptr) {
            if (const XmlNode* origin = childNamed(*visual, "origin")) {
                if (origin->attributes.count("xyz")) {
                    const std::vector<double> xyz = splitNumbers(origin->attributes.at("xyz"));
                    if (xyz.size() >= 3) {
                        visualNode->local.translation = {xyz[0] * 1000.0, xyz[1] * 1000.0, xyz[2] * 1000.0};
                    }
                }
            }
        }
        scene.addChild(node, visualNode);
        result.triangleCount += static_cast<int>(mesh->triangleCount());
        ++result.meshCount;
        ++result.nodeCount;
    }
    for (const XmlNode& element : robot->children) {
        if (lowerCopy(element.name) != "joint") {
            continue;
        }
        const std::string type = lowerCopy(element.attributes.count("type") ? element.attributes.at("type") : "");
        const XmlNode* parent = childNamed(element, "parent");
        const XmlNode* child = childNamed(element, "child");
        if (parent == nullptr || child == nullptr) {
            continue;
        }
        const std::string parentName = parent->attributes.count("link") ? parent->attributes.at("link") : "";
        const std::string childName = child->attributes.count("link") ? child->attributes.at("link") : "";
        auto parentNode = linkOf(parentName);
        auto childNode = linkOf(childName);
        Transform origin;
        if (const XmlNode* originNode = childNamed(element, "origin")) {
            if (originNode->attributes.count("xyz")) {
                const std::vector<double> xyz = splitNumbers(originNode->attributes.at("xyz"));
                if (xyz.size() >= 3) {
                    origin.translation = {xyz[0] * 1000.0, xyz[1] * 1000.0, xyz[2] * 1000.0};
                }
            }
            if (originNode->attributes.count("rpy")) {
                const std::vector<double> rpy = splitNumbers(originNode->attributes.at("rpy"));
                if (rpy.size() >= 3) {
                    origin.rotation = Quat::fromEulerXYZ(rpy[0], rpy[1], rpy[2]);
                }
            }
        }
        scene.reparent(childNode->id, parentNode->id, false);
        childNode->local = origin;
        if (type == "fixed") {
            for (KinLink& link : resource.links) {
                if (link.name == childName) {
                    link.mountedOn = parentName;
                    link.mountRest = origin;
                }
            }
            continue;
        }
        if (type != "revolute" && type != "continuous" && type != "prismatic") {
            continue;
        }
        KinJoint joint;
        joint.name = element.attributes.count("name") ? element.attributes.at("name") : childName;
        joint.parentLink = parentName;
        joint.childLink = childName;
        joint.motion = type == "prismatic" ? JointMotion::Prismatic : JointMotion::Revolute;
        joint.restLocal = origin;
        joint.anchor = origin.translation;
        if (const XmlNode* axis = childNamed(element, "axis")) {
            if (axis->attributes.count("xyz")) {
                const std::vector<double> xyz = splitNumbers(axis->attributes.at("xyz"));
                if (xyz.size() >= 3) {
                    joint.axis = {xyz[0], xyz[1], xyz[2]};
                }
            }
        }
        if (const XmlNode* limit = childNamed(element, "limit")) {
            const double scale = type == "prismatic" ? 1000.0 : 180.0 / 3.14159265358979323846;
            if (limit->attributes.count("lower")) {
                joint.lower = std::stod(limit->attributes.at("lower")) * scale;
            }
            if (limit->attributes.count("upper")) {
                joint.upper = std::stod(limit->attributes.at("upper")) * scale;
            }
            if (limit->attributes.count("velocity")) {
                joint.maxSpeed = std::stod(limit->attributes.at("velocity")) * scale;
            }
            joint.limited = type != "continuous";
        }
        resource.joints.push_back(std::move(joint));
    }
    scene.resources.push_back(std::move(resource));
    scene.endEdit();
    result.ok = true;
    result.nodeCount = std::max(result.nodeCount, static_cast<int>(links.size()));
    result.message = "imported";
    return result;
}

class StepAssetImporter final : public IAssetImporter {
public:
    const char* format() const override { return "step"; }
    std::vector<std::string> extensions() const override { return {"step", "stp"}; }
    bool probeContent(const std::string& head) const override { return head.find("ISO-10303-21") != std::string::npos; }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        StepImporter importer;
        const ImportResult imported = importer.importFile(path, scene);
        AssetImportResult result;
        result.format = format();
        result.ok = imported.ok;
        result.message = imported.message;
        result.nodeCount = imported.nodeCount;
        result.meshCount = imported.meshCount;
        result.triangleCount = imported.triangleCount;
        return result;
    }
};

class StlImporter final : public IAssetImporter {
public:
    const char* format() const override { return "stl"; }
    std::vector<std::string> extensions() const override { return {"stl"}; }
    bool probeContent(const std::string& head) const override { return lowerCopy(head.substr(0, std::min<std::size_t>(5, head.size()))) == "solid"; }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        std::string error;
        const std::string bytes = readFile(path, error);
        if (!error.empty()) {
            AssetImportResult result;
            result.format = format();
            result.message = error;
            return result;
        }
        return adoptMesh(scene, meshFromStl(bytes, error), "stl", format());
    }
};

class ObjAssetImporter final : public IAssetImporter {
public:
    const char* format() const override { return "obj"; }
    std::vector<std::string> extensions() const override { return {"obj"}; }
    bool probeContent(const std::string& head) const override { return head.rfind("v ", 0) == 0 || head.find("\nv ") != std::string::npos; }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        ObjImporter importer;
        const ImportResult imported = importer.importFile(path, scene);
        AssetImportResult result;
        result.format = format();
        result.ok = imported.ok;
        result.message = imported.message;
        result.nodeCount = imported.nodeCount;
        result.meshCount = imported.meshCount;
        result.triangleCount = imported.triangleCount;
        return result;
    }
};

class PlyImporter final : public IAssetImporter {
public:
    const char* format() const override { return "ply"; }
    std::vector<std::string> extensions() const override { return {"ply"}; }
    bool probeContent(const std::string& head) const override { return head.rfind("ply", 0) == 0; }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        std::string error;
        const std::string bytes = readFile(path, error);
        if (!error.empty()) {
            AssetImportResult result;
            result.format = format();
            result.message = error;
            return result;
        }
        auto mesh = meshFromPly(bytes, error);
        if (!mesh) {
            AssetImportResult result;
            result.format = format();
            result.message = error;
            return result;
        }
        return adoptMesh(scene, mesh, "ply", format());
    }
};

class GltfAssetImporter final : public IAssetImporter {
public:
    const char* format() const override { return "gltf"; }
    std::vector<std::string> extensions() const override { return {"gltf", "glb"}; }
    bool probeContent(const std::string& head) const override {
        return head.compare(0, 4, "glTF") == 0 || (head.find("\"asset\"") != std::string::npos && head.find('{') != std::string::npos);
    }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override { return importGltfFile(path, scene); }
};

class UrdfImporter final : public IAssetImporter {
public:
    const char* format() const override { return "urdf"; }
    std::vector<std::string> extensions() const override { return {"urdf"}; }
    bool probeContent(const std::string& head) const override { return head.find("<robot") != std::string::npos; }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        std::string error;
        const std::string bytes = readFile(path, error);
        if (!error.empty()) {
            AssetImportResult result;
            result.format = format();
            result.message = error;
            return result;
        }
        return importUrdfText(bytes, scene);
    }
};

class DxfAssetImporter final : public IAssetImporter {
public:
    const char* format() const override { return "dxf"; }
    std::vector<std::string> extensions() const override { return {"dxf"}; }
    bool probeContent(const std::string& head) const override {
        return head.find("SECTION") != std::string::npos || head.compare(0, 18, "AutoCAD Binary DXF") == 0;
    }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        AssetImportResult result;
        result.format = format();
        std::string error;
        if (!loadDxfUnderlay(scene.layout, path, scene.layout.underlay.mmPerUnit > 0 ? scene.layout.underlay.mmPerUnit : 1, error)) {
            result.message = error;
            return result;
        }
        result.ok = true;
        result.message = "imported " + std::to_string(scene.layout.underlay.lines.size()) + " segments";
        return result;
    }
};

class ImageImporter final : public IAssetImporter {
public:
    const char* format() const override { return "image"; }
    std::vector<std::string> extensions() const override { return {"png", "jpg", "jpeg", "bmp"}; }
    bool probeContent(const std::string& head) const override {
        return (head.size() >= 8 && static_cast<unsigned char>(head[0]) == 0x89 && head[1] == 'P') ||
               (head.size() >= 3 && static_cast<unsigned char>(head[0]) == 0xff && static_cast<unsigned char>(head[1]) == 0xd8);
    }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        AssetImportResult result;
        result.format = format();
        setImageUnderlay(scene.layout, path, 0.45);
        if (scene.layout.underlay.bytes.empty()) {
            result.message = "image was not read";
            return result;
        }
        result.ok = true;
        result.message = "stored " + std::to_string(scene.layout.underlay.bytes.size()) + " image bytes";
        return result;
    }
};

class XmlPprImporter final : public IAssetImporter {
public:
    const char* format() const override { return "ppr-xml"; }
    std::vector<std::string> extensions() const override { return {"xml"}; }
    bool probeContent(const std::string& head) const override {
        return head.find('<') != std::string::npos && head.find("<robot") == std::string::npos;
    }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        std::string error;
        const std::string bytes = readFile(path, error);
        AssetImportResult result;
        result.format = format();
        if (!error.empty()) {
            result.message = error;
            return result;
        }
        if (bytes.find("<robot") != std::string::npos) {
            return importUrdfText(bytes, scene);
        }
        const EmsImportResult imported = importEmsXml(path, scene);
        result.ok = imported.ok;
        result.message = imported.message;
        result.nodeCount = static_cast<int>(scene.ppr.products.size() + scene.ppr.processes.size() + scene.ppr.plants.size());
        return result;
    }
};

class JtImporter final : public IAssetImporter {
public:
    const char* format() const override { return "jt"; }
    std::vector<std::string> extensions() const override { return {"jt"}; }
    bool probeContent(const std::string&) const override { return false; }
    AssetImportResult importFile(const std::string& path, Scene&) const override {
        AssetImportResult result;
        result.format = format();
        result.message =
            "JT has no decoder in this build (" + path +
            "). Siemens JT Open Toolkit is commercial, and XT B-rep plus LOD segments are not implemented here. "
            "Register an IAssetImporter named jt to replace this slot.";
        return result;
    }
};

void appendBytes(std::vector<std::uint8_t>& out, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    out.insert(out.end(), bytes, bytes + size);
}

}  // namespace

void AssetRegistry::add(std::unique_ptr<IAssetImporter> importer) {
    if (importer) {
        importers_.push_back(std::move(importer));
    }
}

const IAssetImporter* AssetRegistry::match(const std::string& path, const std::string& head) const {
    const std::string ext = extensionOf(path);
    if (!ext.empty()) {
        for (const auto& importer : importers_) {
            for (const std::string& candidate : importer->extensions()) {
                if (candidate == ext) {
                    return importer.get();
                }
            }
        }
    }
    for (const auto& importer : importers_) {
        if (importer->probeContent(head)) {
            return importer.get();
        }
    }
    return nullptr;
}

AssetImportResult AssetRegistry::importPath(const std::string& path, Scene& scene) const {
    AssetImportResult result;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        result.message = "cannot open " + path;
        return result;
    }
    std::string head(512, '\0');
    in.read(head.data(), static_cast<std::streamsize>(head.size()));
    head.resize(static_cast<std::size_t>(in.gcount()));
    const IAssetImporter* importer = match(path, head);
    if (importer == nullptr) {
        result.message = "no importer for " + path;
        return result;
    }
    result = importer->importFile(path, scene);
    if (result.format.empty()) {
        result.format = importer->format();
    }
    return result;
}

std::vector<std::string> AssetRegistry::formats() const {
    std::vector<std::string> names;
    for (const auto& importer : importers_) {
        names.emplace_back(importer->format());
    }
    return names;
}

std::unique_ptr<AssetRegistry> createBuiltinRegistry() {
    auto registry = std::make_unique<AssetRegistry>();
    registry->add(std::make_unique<StepAssetImporter>());
    registerOcctAssetImporters(*registry);
    registry->add(std::make_unique<StlImporter>());
    registry->add(std::make_unique<ObjAssetImporter>());
    registry->add(std::make_unique<PlyImporter>());
    registry->add(std::make_unique<GltfAssetImporter>());
    registry->add(std::make_unique<UrdfImporter>());
    registry->add(std::make_unique<DxfAssetImporter>());
    registry->add(std::make_unique<ImageImporter>());
    registry->add(std::make_unique<XmlPprImporter>());
    registry->add(std::make_unique<JtImporter>());
    return registry;
}

bool exportSceneStl(const Scene& scene, const std::string& path, std::string& error) {
    std::vector<std::uint8_t> body;
    std::uint32_t triangles = 0;
    if (scene.root()) {
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
            if (!node->effectivelyVisible() || node->meshId.empty()) {
                continue;
            }
            const auto mesh = scene.mesh(node->meshId);
            if (!mesh) {
                continue;
            }
            const Mat4 world = node->worldMatrix();
            for (std::size_t i = 0; i + 2 < mesh->indices.size(); i += 3) {
                Vec3 points[3];
                for (int k = 0; k < 3; ++k) {
                    const std::uint32_t index = mesh->indices[i + static_cast<std::size_t>(k)];
                    points[k] = transformPoint(world, {mesh->positions[index * 3], mesh->positions[index * 3 + 1],
                                                       mesh->positions[index * 3 + 2]});
                }
                const Vec3 normal = normalize(cross(points[1] - points[0], points[2] - points[0]));
                float record[12] = {static_cast<float>(normal.x), static_cast<float>(normal.y), static_cast<float>(normal.z)};
                for (int k = 0; k < 3; ++k) {
                    record[3 + k * 3] = static_cast<float>(points[k].x);
                    record[4 + k * 3] = static_cast<float>(points[k].y);
                    record[5 + k * 3] = static_cast<float>(points[k].z);
                }
                appendBytes(body, record, sizeof(record));
                const std::uint16_t attr = 0;
                appendBytes(body, &attr, sizeof(attr));
                ++triangles;
            }
        }
    }
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        error = "cannot write " + path;
        return false;
    }
    char header[80] = {};
    std::snprintf(header, sizeof(header), "Sim Studio STL");
    out.write(header, 80);
    out.write(reinterpret_cast<const char*>(&triangles), 4);
    if (!body.empty()) {
        out.write(reinterpret_cast<const char*>(body.data()), static_cast<std::streamsize>(body.size()));
    }
    error.clear();
    return true;
}

bool exportSceneGltf(const Scene& scene, const std::string& path, std::string& error) {
    std::vector<float> positions;
    std::vector<std::uint32_t> indices;
    if (scene.root()) {
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
            if (!node->effectivelyVisible() || node->meshId.empty()) {
                continue;
            }
            const auto mesh = scene.mesh(node->meshId);
            if (!mesh) {
                continue;
            }
            const Mat4 world = node->worldMatrix();
            for (std::size_t i = 0; i + 2 < mesh->indices.size(); i += 3) {
                const auto base = static_cast<std::uint32_t>(positions.size() / 3);
                for (int k = 0; k < 3; ++k) {
                    const std::uint32_t index = mesh->indices[i + static_cast<std::size_t>(k)];
                    const Vec3 point = transformPoint(world, {mesh->positions[index * 3], mesh->positions[index * 3 + 1],
                                                              mesh->positions[index * 3 + 2]}) /
                                       1000.0;
                    positions.push_back(static_cast<float>(point.x));
                    positions.push_back(static_cast<float>(point.y));
                    positions.push_back(static_cast<float>(point.z));
                }
                indices.push_back(base);
                indices.push_back(base + 1);
                indices.push_back(base + 2);
            }
        }
    }
    std::vector<std::uint8_t> bin(positions.size() * 4 + indices.size() * 4);
    if (!positions.empty()) {
        std::memcpy(bin.data(), positions.data(), positions.size() * 4);
    }
    if (!indices.empty()) {
        std::memcpy(bin.data() + positions.size() * 4, indices.data(), indices.size() * 4);
    }
    const int vertexCount = static_cast<int>(positions.size() / 3);
    nlohmann::json json = {{"asset", {{"version", "2.0"}}},
                           {"scene", 0},
                           {"scenes", {{{"nodes", {0}}}}},
                           {"nodes", {{{"mesh", 0}}}},
                           {"meshes", {{{"primitives", {{{"attributes", {{"POSITION", 0}}}, {"indices", 1}, {"mode", 4}}}}}}},
                           {"accessors",
                            {{{"bufferView", 0}, {"componentType", 5126}, {"count", vertexCount}, {"type", "VEC3"}},
                             {{"bufferView", 1}, {"componentType", 5125}, {"count", static_cast<int>(indices.size())}, {"type", "SCALAR"}}}},
                           {"bufferViews",
                            {{{"buffer", 0}, {"byteOffset", 0}, {"byteLength", static_cast<int>(positions.size() * 4)}},
                             {{"buffer", 0},
                              {"byteOffset", static_cast<int>(positions.size() * 4)},
                              {"byteLength", static_cast<int>(indices.size() * 4)}}}},
                           {"buffers", {{{"byteLength", static_cast<int>(bin.size())}}}}};
    const bool glb = extensionOf(path) == "glb";
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        error = "cannot write " + path;
        return false;
    }
    if (!glb) {
        json["buffers"][0]["uri"] = "data:application/octet-stream;base64," + encodeBase64(bin.data(), bin.size());
        const std::string text = json.dump();
        out << text;
        error.clear();
        return true;
    }
    std::string text = json.dump();
    while (text.size() % 4 != 0) {
        text.push_back(' ');
    }
    while (bin.size() % 4 != 0) {
        bin.push_back(0);
    }
    const std::uint32_t jsonLength = static_cast<std::uint32_t>(text.size());
    const std::uint32_t binLength = static_cast<std::uint32_t>(bin.size());
    const std::uint32_t total = 12 + 8 + jsonLength + 8 + binLength;
    const std::uint32_t magic = 0x46546C67u;
    const std::uint32_t version = 2;
    const std::uint32_t jsonType = 0x4E4F534Au;
    const std::uint32_t binType = 0x004E4942u;
    out.write(reinterpret_cast<const char*>(&magic), 4);
    out.write(reinterpret_cast<const char*>(&version), 4);
    out.write(reinterpret_cast<const char*>(&total), 4);
    out.write(reinterpret_cast<const char*>(&jsonLength), 4);
    out.write(reinterpret_cast<const char*>(&jsonType), 4);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.write(reinterpret_cast<const char*>(&binLength), 4);
    out.write(reinterpret_cast<const char*>(&binType), 4);
    if (!bin.empty()) {
        out.write(reinterpret_cast<const char*>(bin.data()), static_cast<std::streamsize>(bin.size()));
    }
    error.clear();
    return true;
}

ImportResult GltfImporter::importFile(const std::string& path, Scene& scene) {
    const AssetImportResult imported = importGltfFile(path, scene);
    ImportResult result;
    result.ok = imported.ok;
    result.message = imported.message;
    result.nodeCount = imported.nodeCount;
    result.meshCount = imported.meshCount;
    result.triangleCount = imported.triangleCount;
    return result;
}

}  // namespace sim
