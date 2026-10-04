#include "sim/core/Weld.hpp"

#include "sim/core/Scene.hpp"
#include "sim/core/Xml.hpp"

#include <nlohmann/json.hpp>

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace sim {
namespace {

std::vector<std::string> split(const std::string& text, char separator) {
    std::vector<std::string> out;
    std::string cell;
    for (char ch : text) {
        if (ch == separator) {
            out.push_back(cell);
            cell.clear();
        } else if (ch != '\r') {
            cell.push_back(ch);
        }
    }
    out.push_back(cell);
    return out;
}

std::string trim(std::string text) {
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) {
        text.erase(text.begin());
    }
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) {
        text.pop_back();
    }
    return text;
}

bool parseDouble(const std::string& text, double& value) {
    if (text.empty()) {
        return false;
    }
    char* end = nullptr;
    value = std::strtod(text.c_str(), &end);
    return end != text.c_str() && end != nullptr && *end == '\0';
}

std::vector<std::string> splitLayers(const std::string& text) {
    std::vector<std::string> layers;
    for (std::string layer : split(text, '|')) {
        layer = trim(layer);
        if (!layer.empty()) {
            layers.push_back(std::move(layer));
        }
    }
    return layers;
}

int columnIndex(const std::vector<std::string>& header, const char* name, int fallback) {
    for (int i = 0; i < static_cast<int>(header.size()); ++i) {
        if (header[static_cast<std::size_t>(i)] == name) {
            return i;
        }
    }
    return fallback;
}

bool rayTriangle(Vec3 origin, Vec3 direction, Vec3 a, Vec3 b, Vec3 c, double& distance, Vec3& normal) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 p = cross(direction, ac);
    const double det = dot(ab, p);
    if (std::abs(det) < 1e-10) {
        return false;
    }
    const double inv = 1.0 / det;
    const Vec3 s = origin - a;
    const double u = dot(s, p) * inv;
    if (u < -1e-8 || u > 1.0 + 1e-8) {
        return false;
    }
    const Vec3 q = cross(s, ab);
    const double v = dot(direction, q) * inv;
    if (v < -1e-8 || u + v > 1.0 + 1e-8) {
        return false;
    }
    const double t = dot(ac, q) * inv;
    if (t < 1e-6) {
        return false;
    }
    distance = t;
    const Vec3 face = cross(ab, ac);
    if (length(face) < 1e-12) {
        return false;
    }
    normal = normalize(face);
    return true;
}

struct SurfaceHit {
    bool hit = false;
    double distance = 0;
    Vec3 point{0, 0, 0};
    Vec3 normal{0, 0, 1};
};

SurfaceHit closestOnMesh(const MeshData& mesh, const Mat4& world, Vec3 origin, Vec3 direction) {
    SurfaceHit best;
    best.distance = 1e300;
    const bool ray = length(direction) > 0.5;
    const Vec3 dir = ray ? normalize(direction) : Vec3{0, 0, 0};
    for (std::size_t triangle = 0; triangle + 2 < mesh.indices.size(); triangle += 3) {
        auto vertex = [&](std::uint32_t index) {
            const std::size_t base = static_cast<std::size_t>(index) * 3;
            Vec3 local{mesh.positions[base], mesh.positions[base + 1], mesh.positions[base + 2]};
            return transformPoint(world, local);
        };
        const Vec3 a = vertex(mesh.indices[triangle]);
        const Vec3 b = vertex(mesh.indices[triangle + 1]);
        const Vec3 c = vertex(mesh.indices[triangle + 2]);
        if (ray) {
            double distance = 0;
            Vec3 normal;
            if (!rayTriangle(origin, dir, a, b, c, distance, normal)) {
                continue;
            }
            if (distance < best.distance) {
                best.hit = true;
                best.distance = distance;
                best.point = origin + dir * distance;
                best.normal = normal;
            }
            continue;
        }
        const Vec3 ab = b - a;
        const Vec3 ac = c - a;
        const Vec3 face = cross(ab, ac);
        const double area2 = length(face);
        if (area2 < 1e-12) {
            continue;
        }
        const Vec3 normal = face / area2;
        const double plane = dot(origin - a, normal);
        Vec3 projected = origin - normal * plane;
        const Vec3 ap = projected - a;
        const double d00 = dot(ab, ab);
        const double d01 = dot(ab, ac);
        const double d11 = dot(ac, ac);
        const double d20 = dot(ap, ab);
        const double d21 = dot(ap, ac);
        const double denom = d00 * d11 - d01 * d01;
        double u = 0;
        double v = 0;
        if (std::abs(denom) > 1e-12) {
            u = (d11 * d20 - d01 * d21) / denom;
            v = (d00 * d21 - d01 * d20) / denom;
        }
        if (u < 0 || v < 0 || u + v > 1) {
            const Vec3 corners[3] = {a, b, c};
            double bestEdge = 1e300;
            Vec3 snapped = a;
            for (int i = 0; i < 3; ++i) {
                const Vec3 p0 = corners[i];
                const Vec3 p1 = corners[(i + 1) % 3];
                const Vec3 edge = p1 - p0;
                const double elen = dot(edge, edge);
                double t = elen < 1e-12 ? 0 : std::clamp(dot(origin - p0, edge) / elen, 0.0, 1.0);
                const Vec3 point = p0 + edge * t;
                const double gap = length(origin - point);
                if (gap < bestEdge) {
                    bestEdge = gap;
                    snapped = point;
                }
            }
            projected = snapped;
        }
        const double gap = length(origin - projected);
        if (gap < best.distance) {
            best.hit = true;
            best.distance = gap;
            best.point = projected;
            best.normal = normal;
        }
    }
    return best;
}

const SceneNode* findPart(const Scene& scene, const std::string& name) {
    if (name.empty()) {
        return nullptr;
    }
    const auto node = scene.findByName(name);
    if (!node || node->meshId.empty()) {
        return nullptr;
    }
    return node.get();
}

}  // namespace

bool importWeldCsv(const std::string& text, std::vector<WeldSpot>& spots, std::string& error) {
    spots.clear();
    std::vector<std::string> header;
    bool headerSeen = false;
    int row = 0;
    std::stringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const std::vector<std::string> cells = split(line, ',');
        auto cell = [&](int index) {
            if (index < 0 || index >= static_cast<int>(cells.size())) {
                return std::string{};
            }
            return trim(cells[static_cast<std::size_t>(index)]);
        };
        if (!headerSeen) {
            double ignored = 0;
            const bool numeric = parseDouble(cell(1), ignored) || parseDouble(cell(0), ignored);
            if (!numeric) {
                for (const std::string& name : cells) {
                    header.push_back(trim(name));
                }
                headerSeen = true;
                continue;
            }
            header = {"id", "x", "y", "z", "nx", "ny", "nz", "layers", "electrode", "example"};
            headerSeen = true;
        }
        WeldSpot spot;
        const int idCol = columnIndex(header, "id", 0);
        const int xCol = columnIndex(header, "x", 1);
        const int yCol = columnIndex(header, "y", 2);
        const int zCol = columnIndex(header, "z", 3);
        spot.id = cell(idCol);
        if (spot.id.empty()) {
            spot.id = "w" + std::to_string(++row);
        }
        spot.name = spot.id;
        if (!parseDouble(cell(xCol), spot.position.x) || !parseDouble(cell(yCol), spot.position.y) ||
            !parseDouble(cell(zCol), spot.position.z)) {
            error = "weld csv row needs x y z: " + spot.id;
            spots.clear();
            return false;
        }
        double nx = 0;
        double ny = 0;
        double nz = 1;
        parseDouble(cell(columnIndex(header, "nx", 4)), nx);
        parseDouble(cell(columnIndex(header, "ny", 5)), ny);
        parseDouble(cell(columnIndex(header, "nz", 6)), nz);
        if (length(Vec3{nx, ny, nz}) < 1e-8) {
            nz = 1;
        }
        spot.normal = normalize(Vec3{nx, ny, nz});
        spot.layers = splitLayers(cell(columnIndex(header, "layers", 7)));
        spot.electrode = cell(columnIndex(header, "electrode", 8));
        const std::string example = cell(columnIndex(header, "example", 9));
        spot.example = example == "1" || example == "true" || example == "yes" || example == "示例";
        if (!spot.layers.empty()) {
            spot.part = spot.layers.front();
        }
        spots.push_back(std::move(spot));
        ++row;
    }
    if (spots.empty()) {
        error = "weld csv has no points";
        return false;
    }
    error.clear();
    return true;
}

bool importWeldXml(const std::string& text, std::vector<WeldSpot>& spots, std::string& error) {
    spots.clear();
    XmlNode root;
    if (!parseXml(text, root, error)) {
        return false;
    }
    const XmlNode* list = &root;
    if (root.name != "welds" && root.name != "WeldPoints") {
        list = nullptr;
        for (const XmlNode& child : root.children) {
            if (child.name == "welds" || child.name == "WeldPoints") {
                list = &child;
                break;
            }
        }
        if (list == nullptr && root.name == "spot") {
            list = &root;
        }
    }
    if (list == nullptr) {
        error = "weld xml needs a welds element";
        return false;
    }
    const auto readSpot = [&](const XmlNode& node) {
        WeldSpot spot;
        spot.id = node.attributes.count("id") ? node.attributes.at("id") : "";
        spot.electrode = node.attributes.count("electrode") ? node.attributes.at("electrode") : "";
        const std::string example = node.attributes.count("example") ? node.attributes.at("example") : "";
        spot.example = example == "1" || example == "true" || example == "示例";
        if (node.attributes.count("layers")) {
            spot.layers = splitLayers(node.attributes.at("layers"));
        }
        bool havePoint = false;
        for (const XmlNode& child : node.children) {
            if (child.name == "point" || child.name == "position") {
                parseDouble(child.attributes.count("x") ? child.attributes.at("x") : "", spot.position.x);
                parseDouble(child.attributes.count("y") ? child.attributes.at("y") : "", spot.position.y);
                parseDouble(child.attributes.count("z") ? child.attributes.at("z") : "", spot.position.z);
                havePoint = true;
            } else if (child.name == "normal") {
                double nx = 0;
                double ny = 0;
                double nz = 1;
                parseDouble(child.attributes.count("x") ? child.attributes.at("x") : "", nx);
                parseDouble(child.attributes.count("y") ? child.attributes.at("y") : "", ny);
                parseDouble(child.attributes.count("z") ? child.attributes.at("z") : "", nz);
                if (length(Vec3{nx, ny, nz}) > 1e-8) {
                    spot.normal = normalize(Vec3{nx, ny, nz});
                }
            } else if (child.name == "layer" && !trim(child.text).empty()) {
                spot.layers.push_back(trim(child.text));
            }
        }
        if (spot.id.empty()) {
            spot.id = "w" + std::to_string(spots.size() + 1);
        }
        spot.name = spot.id;
        if (!spot.layers.empty() && spot.part.empty()) {
            spot.part = spot.layers.front();
        }
        if (!havePoint) {
            return false;
        }
        spots.push_back(std::move(spot));
        return true;
    };
    if (list->name == "spot") {
        if (!readSpot(*list)) {
            error = "weld spot is missing a point";
            spots.clear();
            return false;
        }
    } else {
        for (const XmlNode& child : list->children) {
            if (child.name != "spot") {
                continue;
            }
            if (!readSpot(child)) {
                error = "weld spot is missing a point";
                spots.clear();
                return false;
            }
        }
    }
    if (spots.empty()) {
        error = "weld xml has no points";
        return false;
    }
    error.clear();
    return true;
}

nlohmann::json weldsToJson(const std::vector<WeldSpot>& spots) {
    nlohmann::json out = nlohmann::json::array();
    for (const WeldSpot& spot : spots) {
        out.push_back({{"id", spot.id},
                       {"name", spot.name},
                       {"position", {spot.position.x, spot.position.y, spot.position.z}},
                       {"normal", {spot.normal.x, spot.normal.y, spot.normal.z}},
                       {"layers", spot.layers},
                       {"electrode", spot.electrode},
                       {"part", spot.part},
                       {"onSurface", spot.onSurface},
                       {"example", spot.example},
                       {"checked", spot.checked},
                       {"reachable", spot.reachable},
                       {"margin", spot.margin},
                       {"spinDeg", spot.spinDeg},
                       {"manipulability", spot.manipulability},
                       {"singular", spot.singular},
                       {"collision", spot.collision}});
    }
    return out;
}

void weldsFromJson(const nlohmann::json& json, std::vector<WeldSpot>& spots) {
    spots.clear();
    if (!json.is_array()) {
        return;
    }
    for (const auto& item : json) {
        if (!item.is_object()) {
            continue;
        }
        WeldSpot spot;
        spot.id = item.value("id", std::string{});
        spot.name = item.value("name", spot.id);
        if (item.contains("position") && item.at("position").is_array() && item.at("position").size() >= 3) {
            spot.position = {item.at("position").at(0).get<double>(), item.at("position").at(1).get<double>(),
                             item.at("position").at(2).get<double>()};
        }
        if (item.contains("normal") && item.at("normal").is_array() && item.at("normal").size() >= 3) {
            spot.normal = normalize(Vec3{item.at("normal").at(0).get<double>(), item.at("normal").at(1).get<double>(),
                                         item.at("normal").at(2).get<double>()});
        }
        if (length(spot.normal) < 0.5) {
            spot.normal = {0, 0, 1};
        }
        if (item.contains("layers") && item.at("layers").is_array()) {
            for (const auto& layer : item.at("layers")) {
                if (layer.is_string()) {
                    spot.layers.push_back(layer.get<std::string>());
                }
            }
        }
        spot.electrode = item.value("electrode", std::string{});
        spot.part = item.value("part", std::string{});
        spot.onSurface = item.value("onSurface", false);
        spot.example = item.value("example", false);
        spot.checked = item.value("checked", false);
        spot.reachable = item.value("reachable", false);
        spot.margin = item.value("margin", 0.0);
        spot.spinDeg = item.value("spinDeg", 0.0);
        spot.manipulability = item.value("manipulability", 0.0);
        spot.singular = item.value("singular", false);
        spot.collision = item.value("collision", false);
        if (!spot.id.empty()) {
            spots.push_back(std::move(spot));
        }
    }
}

int projectWeldSpots(Scene& scene) {
    int projected = 0;
    for (WeldSpot& spot : scene.welds) {
        const SceneNode* part = findPart(scene, spot.part);
        if (part == nullptr) {
            for (const std::string& layer : spot.layers) {
                part = findPart(scene, layer);
                if (part != nullptr) {
                    spot.part = layer;
                    break;
                }
            }
        }
        if (part == nullptr) {
            spot.onSurface = false;
            continue;
        }
        const auto mesh = scene.mesh(part->meshId);
        if (!mesh || mesh->empty()) {
            spot.onSurface = false;
            continue;
        }
        const Vec3 normal = length(spot.normal) > 0.5 ? normalize(spot.normal) : Vec3{0, 0, 1};
        const Vec3 origin = spot.position + normal * 500.0;
        SurfaceHit hit = closestOnMesh(*mesh, part->worldMatrix(), origin, -normal);
        if (!hit.hit) {
            hit = closestOnMesh(*mesh, part->worldMatrix(), spot.position, {});
        }
        if (!hit.hit) {
            spot.onSurface = false;
            continue;
        }
        if (dot(hit.normal, normal) < 0) {
            hit.normal = -hit.normal;
        }
        spot.position = hit.point;
        spot.normal = hit.normal;
        spot.onSurface = true;
        ++projected;
    }
    return projected;
}

}  // namespace sim
