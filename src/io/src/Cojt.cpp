#include "sim/io/Cojt.hpp"

#include "JtImporter.hpp"

#include "sim/core/Xml.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
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

std::string readTextFile(const std::filesystem::path& path, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot open " + path.string();
        return {};
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    error.clear();
    return buffer.str();
}

std::string stemOf(std::string name) {
    const auto slash = name.find_last_of("/\\");
    if (slash != std::string::npos) {
        name = name.substr(slash + 1);
    }
    const auto semi = name.find(';');
    if (semi != std::string::npos) {
        name = name.substr(0, semi);
    }
    const auto dot = name.find('.');
    if (dot != std::string::npos) {
        name = name.substr(0, dot);
    }
    return lowerCopy(name);
}

std::string guidKey(std::string text) {
    std::string out;
    for (char c : text) {
        if (c == '{' || c == '}' || c == '-' || std::isspace(static_cast<unsigned char>(c))) {
            continue;
        }
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return out;
}

const XmlNode* childNamed(const XmlNode& node, const std::string& name) {
    for (const XmlNode& child : node.children) {
        if (lowerCopy(child.name) == name) {
            return &child;
        }
    }
    return nullptr;
}

std::string childText(const XmlNode& node, const std::string& name) {
    if (const XmlNode* child = childNamed(node, name)) {
        return child->text;
    }
    return {};
}

std::string attribute(const XmlNode& node, const std::string& name) {
    for (const auto& [key, value] : node.attributes) {
        if (lowerCopy(key) == name) {
            return value;
        }
    }
    return {};
}

bool parseDouble(const std::string& text, double& out) {
    if (text.empty()) {
        return false;
    }
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str()) {
        return false;
    }
    out = value;
    return true;
}

std::vector<std::string> words(const std::string& text) {
    std::vector<std::string> out;
    std::string current;
    for (char c : text) {
        if (std::isspace(static_cast<unsigned char>(c)) || c == ',') {
            if (!current.empty()) {
                out.push_back(current);
                current.clear();
            }
        } else {
            current.push_back(c);
        }
    }
    if (!current.empty()) {
        out.push_back(current);
    }
    return out;
}

void readEdgeTooltip(CojtGraphEdge& edge) {
    const std::vector<std::string> token = words(lowerCopy(edge.tooltip));
    for (std::size_t i = 0; i < token.size(); ++i) {
        if (token[i] == "prismatic" || token[i] == "linear" || token[i] == "lin") {
            edge.prismatic = true;
        } else if (token[i] == "axis" && i + 3 < token.size()) {
            double x = 0;
            double y = 0;
            double z = 0;
            if (parseDouble(token[i + 1], x) && parseDouble(token[i + 2], y) && parseDouble(token[i + 3], z)) {
                edge.axis = {x, y, z};
                edge.hasAxis = length(edge.axis) > 1e-9;
                i += 3;
            }
        } else if (token[i] == "limit" && i + 2 < token.size()) {
            double lower = 0;
            double upper = 0;
            if (parseDouble(token[i + 1], lower) && parseDouble(token[i + 2], upper)) {
                edge.lower = lower;
                edge.upper = upper;
                edge.hasLimits = true;
                i += 2;
            }
        } else if (token[i] == "speed" && i + 1 < token.size()) {
            double speed = 0;
            if (parseDouble(token[i + 1], speed)) {
                edge.speed = speed;
                edge.hasSpeed = true;
                ++i;
            }
        } else if (token[i] == "couple" && i + 2 < token.size()) {
            double ratio = 0;
            if (parseDouble(token[i + 2], ratio)) {
                edge.coupledTo = token[i + 1];
                edge.coupleRatio = ratio;
                i += 2;
            }
        }
    }
}

int indexOr(const std::string& text, int fallback) {
    if (text.empty()) {
        return fallback;
    }
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str()) {
        return fallback;
    }
    return static_cast<int>(value);
}

MotionAxisRate& axisRate(MotionParameters& out, const std::string& name) {
    const std::string key = lowerCopy(name);
    for (MotionAxisRate& rate : out.joints) {
        if (rate.name == key) {
            return rate;
        }
    }
    MotionAxisRate created;
    created.name = key;
    out.joints.push_back(created);
    return out.joints.back();
}

MotionZone& zoneOf(MotionParameters& out, const std::string& name) {
    const std::string key = lowerCopy(name);
    for (MotionZone& zone : out.zones) {
        if (zone.name == key) {
            return zone;
        }
    }
    MotionZone created;
    created.name = key;
    out.zones.push_back(created);
    return out.zones.back();
}

int jointIndex(const std::string& name) {
    const std::string key = lowerCopy(name);
    if (key.size() >= 2 && key[0] == 'j') {
        char* end = nullptr;
        const long value = std::strtol(key.c_str() + 1, &end, 10);
        if (end != key.c_str() + 1 && value > 0) {
            return static_cast<int>(value);
        }
    }
    return -1;
}

std::string stripComment(std::string line) {
    bool quote = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '"') {
            quote = !quote;
        } else if (line[i] == ';' && !quote) {
            return line.substr(0, i);
        }
    }
    return line;
}

bool fieldNumber(const std::string& body, const std::string& key, double& out) {
    const std::string hay = lowerCopy(body);
    const std::string needle = lowerCopy(key);
    std::size_t from = 0;
    while (from < hay.size()) {
        const auto at = hay.find(needle, from);
        if (at == std::string::npos) {
            return false;
        }
        const bool left = at == 0 || !(std::isalnum(static_cast<unsigned char>(hay[at - 1])) || hay[at - 1] == '_');
        const std::size_t after = at + needle.size();
        const bool right = after >= hay.size() || !(std::isalnum(static_cast<unsigned char>(hay[after])) || hay[after] == '_');
        if (left && right) {
            return parseDouble(body.substr(after), out);
        }
        from = at + 1;
    }
    return false;
}

MachineAxis& machineAxis(MachineData& out, int index) {
    if (out.axes.empty()) {
        out.axes.resize(12);
        for (int i = 0; i < 12; ++i) {
            out.axes[static_cast<std::size_t>(i)].index = i + 1;
        }
    }
    return out.axes[static_cast<std::size_t>(index - 1)];
}

int parseBinaryMask(const std::string& text) {
    std::string bits;
    for (char c : text) {
        if (c == '0' || c == '1') {
            bits.push_back(c);
        }
    }
    if (bits.empty()) {
        return -1;
    }
    int mask = 0;
    for (char bit : bits) {
        mask = (mask << 1) | (bit == '1' ? 1 : 0);
    }
    return mask;
}

void finishMachine(MachineData& out) {
    if (out.axes.empty()) {
        out.ok = out.hasLengthA || out.hasLengthB || out.tiroro.present || out.tx3p3.present;
        return;
    }
    int used = 0;
    for (MachineAxis& axis : out.axes) {
        const bool masked = out.activeMask < 0 || (out.activeMask & (1 << (axis.index - 1))) != 0;
        const bool spanned = std::abs(axis.upper - axis.lower) > 1e-9;
        const bool spinning = std::abs(axis.motorRpm) > 1e-9;
        axis.rotational = axis.type != 1;
        axis.endless = axis.type == 5;
        axis.used = masked && axis.type != 0 && (spanned || spinning || axis.endless);
        if (!axis.rotational) {
            axis.hasSpeed = false;
            axis.speed = 0;
        } else if (std::abs(axis.gearDen) > 1e-12 && std::abs(axis.gearNum) > 1e-12 && std::abs(axis.motorRpm) > 1e-9) {
            const double gear = std::abs(axis.gearNum / axis.gearDen);
            axis.speed = std::abs(axis.motorRpm) * 6.0 / gear;
            axis.hasSpeed = true;
        }
        if (axis.used) {
            ++used;
        }
    }
    out.ok = used > 0 || out.hasLengthA || out.hasLengthB || out.tiroro.present || out.tx3p3.present;
}

Quat quatFromAffine(const Mat4& m) {
    const double m00 = m(0, 0);
    const double m01 = m(1, 0);
    const double m02 = m(2, 0);
    const double m10 = m(0, 1);
    const double m11 = m(1, 1);
    const double m12 = m(2, 1);
    const double m20 = m(0, 2);
    const double m21 = m(1, 2);
    const double m22 = m(2, 2);
    const double trace = m00 + m11 + m22;
    Quat q;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q.w = (m21 - m12) / s;
        q.x = 0.25 * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25 * s;
        q.z = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25 * s;
    }
    return q.normalized();
}

Quat kukaEuler(double a, double b, double c) {
    const Quat rz = Quat::fromAxisAngle({0, 0, 1}, deg2rad(a));
    const Quat ry = Quat::fromAxisAngle({0, 1, 0}, deg2rad(b));
    const Quat rx = Quat::fromAxisAngle({1, 0, 0}, deg2rad(c));
    return (rz * ry * rx).normalized();
}

int depthOf(const std::shared_ptr<SceneNode>& node) {
    int depth = 0;
    for (auto cursor = node; cursor; cursor = cursor->parent.lock()) {
        ++depth;
    }
    return depth;
}

bool isAncestor(const std::shared_ptr<SceneNode>& ancestor, const std::shared_ptr<SceneNode>& node) {
    for (auto cursor = node; cursor; cursor = cursor->parent.lock()) {
        if (cursor == ancestor) {
            return true;
        }
    }
    return false;
}

struct SceneIndex {
    std::vector<std::shared_ptr<SceneNode>> nodes;
};

SceneIndex indexScene(const Scene& scene) {
    SceneIndex index;
    if (!scene.root()) {
        return index;
    }
    std::vector<std::shared_ptr<SceneNode>> stack{scene.root()};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        index.nodes.push_back(node);
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
    }
    return index;
}

std::shared_ptr<SceneNode> findStem(const SceneIndex& index, const std::string& stem, const std::string& guid,
                                    const std::vector<std::string>& used) {
    const std::string want = lowerCopy(stem);
    const std::string guidWant = guidKey(guid);
    std::shared_ptr<SceneNode> best;
    int bestScore = -1;
    int bestDepth = 1000000;
    for (const auto& node : index.nodes) {
        if (!node || node->name.empty()) {
            continue;
        }
        if (std::find(used.begin(), used.end(), node->id) != used.end()) {
            continue;
        }
        int score = 0;
        if (stemOf(node->name) == want) {
            score = 100;
        } else if (!guidWant.empty() && guidKey(node->name).find(guidWant) != std::string::npos) {
            score = 80;
        }
        if (score == 0) {
            continue;
        }
        const int depth = depthOf(node);
        if (score > bestScore || (score == bestScore && depth < bestDepth)) {
            best = node;
            bestScore = score;
            bestDepth = depth;
        }
    }
    return best;
}

Aabb meshBounds(const Scene& scene) {
    Aabb box;
    if (!scene.root()) {
        return box;
    }
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
        if (node->meshId.empty()) {
            continue;
        }
        const auto mesh = scene.mesh(node->meshId);
        if (!mesh || !mesh->bounds.valid) {
            continue;
        }
        const Mat4 world = node->worldMatrix();
        const Vec3 corners[2] = {mesh->bounds.min, mesh->bounds.max};
        for (int i = 0; i < 8; ++i) {
            const Vec3 local{(i & 1) ? corners[1].x : corners[0].x, (i & 2) ? corners[1].y : corners[0].y,
                             (i & 4) ? corners[1].z : corners[0].z};
            box.expand(transformPoint(world, local));
        }
    }
    return box;
}

double nominalMillimeters(const std::string& name) {
    const std::string text = lowerCopy(name);
    double last = 0;
    bool found = false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(text[i]))) {
            continue;
        }
        double value = 0;
        if (!parseDouble(text.substr(i), value)) {
            continue;
        }
        std::size_t j = i;
        while (j < text.size() && (std::isdigit(static_cast<unsigned char>(text[j])) || text[j] == '.')) {
            ++j;
        }
        if (j < text.size() && text[j] == 'm') {
            last = value * 1000.0;
            found = true;
        }
        i = j;
    }
    return found ? last : 0;
}

bool looksLikeTrack(const std::string& folderName, const CojtGraph& graph, const MotionParameters& motion,
                    const MachineData& machine, const std::vector<InfoAxisLimit>& info) {
    if (machine.ok) {
        for (const MachineAxis& axis : machine.axes) {
            if (axis.used) {
                return false;
            }
        }
    }
    if (!motion.joints.empty() || !info.empty() || graph.links.size() > 2 || !graph.edges.empty()) {
        return false;
    }
    const std::string name = lowerCopy(folderName);
    return name.find("irbt") != std::string::npos || name.find("track") != std::string::npos ||
           name.find("rail") != std::string::npos || nominalMillimeters(name) > 0;
}

std::filesystem::path componentFolder(const std::filesystem::path& path) {
    std::error_code error;
    if (std::filesystem::is_directory(path, error)) {
        return path;
    }
    return path.parent_path();
}

bool folderIsCojt(const std::filesystem::path& folder) {
    return lowerCopy(folder.filename().string()).size() > 5 &&
           lowerCopy(folder.filename().string()).rfind(".cojt") == lowerCopy(folder.filename().string()).size() - 5;
}

std::filesystem::path findMachineDat(const std::filesystem::path& folder) {
    std::filesystem::path fallback;
    std::error_code error;
    if (!std::filesystem::is_directory(folder, error)) {
        return {};
    }
    for (const auto& entry : std::filesystem::recursive_directory_iterator(folder, error)) {
        if (error || !entry.is_regular_file()) {
            continue;
        }
        if (entry.path().filename() != "$machine.dat") {
            continue;
        }
        const std::string text = lowerCopy(entry.path().generic_string());
        if (text.find("/krc/r1/mada/") != std::string::npos) {
            return entry.path();
        }
        if (fallback.empty()) {
            fallback = entry.path();
        }
    }
    return fallback;
}

std::filesystem::path findJt(const std::filesystem::path& folder) {
    std::error_code error;
    if (!std::filesystem::is_directory(folder, error)) {
        return {};
    }
    const std::string want = stemOf(folder.filename().string());
    std::filesystem::path fallback;
    for (const auto& entry : std::filesystem::directory_iterator(folder, error)) {
        if (error || !entry.is_regular_file()) {
            continue;
        }
        if (lowerCopy(entry.path().extension().string()) != ".jt") {
            continue;
        }
        if (stemOf(entry.path().filename().string()) == want) {
            return entry.path();
        }
        if (fallback.empty()) {
            fallback = entry.path();
        }
    }
    return fallback;
}

struct BuiltLink {
    std::string name;
    std::shared_ptr<SceneNode> node;
    bool matched = false;
};

Vec3 axisFromFrame(const Scene& scene, const std::shared_ptr<SceneNode>& frame, const Mat4& parentWorld, Vec3& anchor,
                   bool& accepted) {
    accepted = false;
    Vec3 direction{0, 0, 1};
    bool fromMesh = false;
    if (!frame->meshId.empty()) {
        if (const auto mesh = scene.mesh(frame->meshId); mesh && mesh->bounds.valid) {
            const Vec3 size = mesh->bounds.size();
            const double dims[3] = {size.x, size.y, size.z};
            int longest = 0;
            if (dims[1] > dims[longest]) {
                longest = 1;
            }
            if (dims[2] > dims[longest]) {
                longest = 2;
            }
            const double others = dims[(longest + 1) % 3] + dims[(longest + 2) % 3];
            if (dims[longest] > 3.0 * std::max(others, 1e-6)) {
                direction = {longest == 0 ? 1.0 : 0.0, longest == 1 ? 1.0 : 0.0, longest == 2 ? 1.0 : 0.0};
                fromMesh = true;
            }
        }
    }
    const Mat4 world = frame->worldMatrix();
    if (!fromMesh) {
        direction = {world(2, 0), world(2, 1), world(2, 2)};
    } else {
        direction = transformVector(world, direction);
    }
    const Mat4 local = mul(inverseAffine(parentWorld), world);
    anchor = {local(3, 0), local(3, 1), local(3, 2)};
    const Vec3 localDirection = transformVector(inverseAffine(parentWorld), direction);
    if (length(localDirection) < 1e-8) {
        return {0, 0, 1};
    }
    direction = normalize(localDirection);
    // An unrotated frame whose Z is the parent Z does not measure a joint. An elongated mesh does.
    accepted = fromMesh || std::abs(direction.z) < 0.99;
    return direction;
}

void setRest(KinJoint& joint, const std::shared_ptr<SceneNode>& parent, const std::shared_ptr<SceneNode>& child, Scene& scene) {
    if (!parent || !child) {
        return;
    }
    const Mat4 relative = mul(inverseAffine(parent->worldMatrix()), child->worldMatrix());
    if (!isAncestor(child, parent) && scene.reparent(child->id, parent->id, true)) {
        joint.restLocal = child->local;
        return;
    }
    joint.restLocal.translation = {relative(3, 0), relative(3, 1), relative(3, 2)};
    joint.restLocal.rotation = quatFromAffine(relative);
}

std::string componentName(const std::filesystem::path& folder) {
    std::string name = folder.filename().string();
    const std::string lower = lowerCopy(name);
    if (lower.size() > 5 && lower.rfind(".cojt") == lower.size() - 5) {
        name = name.substr(0, name.size() - 5);
    }
    return name.empty() ? "cojt" : name;
}

}  // namespace

bool parseKinGraphXml(const std::string& text, CojtGraph& graph, std::string& error) {
    graph = {};
    XmlNode root;
    if (!parseXml(text, root, error)) {
        return false;
    }
    if (lowerCopy(root.name) != "addflow") {
        error = "kin_graph root is not AddFlow";
        return false;
    }
    for (const XmlNode& node : root.children) {
        const std::string kind = lowerCopy(node.name);
        if (kind == "node") {
            CojtGraphLink link;
            link.name = childText(node, "text");
            if (link.name.empty()) {
                link.name = childText(node, "tooltip");
            }
            link.guid = childText(node, "url");
            if (!link.name.empty()) {
                graph.links.push_back(link);
            }
        }
    }
    for (const XmlNode& node : root.children) {
        if (lowerCopy(node.name) != "link") {
            continue;
        }
        CojtGraphEdge edge;
        edge.name = childText(node, "text");
        edge.tooltip = childText(node, "tooltip");
        const std::string org = !attribute(node, "org").empty() ? attribute(node, "org") : attribute(node, "orgindex");
        const std::string dst = !attribute(node, "dst").empty() ? attribute(node, "dst") : attribute(node, "dstindex");
        edge.from = indexOr(org, -1);
        edge.to = indexOr(dst, -1);
        readEdgeTooltip(edge);
        if (edge.name.empty()) {
            edge.name = "J" + std::to_string(graph.edges.size() + 1);
        }
        graph.edges.push_back(edge);
    }
    error.clear();
    return true;
}

bool parseMotionParameters(const std::string& text, MotionParameters& out, std::string& error) {
    out = {};
    const auto flush = [&](std::string item) {
        const auto hash = item.find('%');
        if (hash != std::string::npos) {
            item = item.substr(0, hash);
        }
        const std::vector<std::string> token = words(item);
        if (token.empty()) {
            return;
        }
        const std::string key = lowerCopy(token[0]);
        if (key == "joint_max_speed" || key == "joint_basic_speed") {
            if (token.size() < 3) {
                return;
            }
            double value = 0;
            if (!parseDouble(token[2], value)) {
                return;
            }
            MotionAxisRate& rate = axisRate(out, token[1]);
            if (key == "joint_basic_speed" && rate.hasSpeed) {
                return;
            }
            if (std::abs(value) < 20.0) {
                value = rad2deg(value);
            }
            rate.speedDeg = value;
            rate.hasSpeed = true;
        } else if (key == "joint_acc_time" || key == "joint_dec_time" || key == "joint_settle" || key == "joint_delay") {
            if (token.size() < 3) {
                return;
            }
            double value = 0;
            if (!parseDouble(token[2], value)) {
                return;
            }
            MotionAxisRate& rate = axisRate(out, token[1]);
            if (key == "joint_acc_time") {
                rate.accTime = value;
                rate.hasAccTime = true;
            } else if (key == "joint_dec_time") {
                rate.decTime = value;
            } else if (key == "joint_settle") {
                rate.settle = value;
            } else {
                rate.delay = value;
            }
        } else if (key == "cart_max_lin_speed" && token.size() >= 2) {
            double value = 0;
            if (parseDouble(token[1], value)) {
                out.cartMaxLinSpeed = value;
                out.hasCartSpeed = true;
            }
        } else if (key == "cart_max_lin_acc" && token.size() >= 2) {
            double value = 0;
            if (parseDouble(token[1], value)) {
                out.cartMaxLinAcc = value;
                out.hasCartAcc = true;
            }
        } else if (key == "config_family" && token.size() >= 2) {
            out.configFamily = token[1];
        } else if (key == "zone_joint_type" && token.size() >= 3) {
            zoneOf(out, token[1]).jointType = token[2];
        } else if (key == "zone_joints_param" && token.size() >= 3) {
            MotionZone& zone = zoneOf(out, token[1]);
            for (std::size_t i = 2; i < token.size(); ++i) {
                double value = 0;
                if (parseDouble(token[i], value)) {
                    zone.jointParam.push_back(value);
                }
            }
        } else if (key == "zone_type" && token.size() >= 3) {
            zoneOf(out, token[1]).cartType = token[2];
        } else if ((key == "zone_cart_param" || key == "zone_define") && token.size() >= 2) {
            if (key == "zone_define") {
                MotionZone& zone = zoneOf(out, token.size() >= 2 ? token[1] : token[0]);
                for (std::size_t i = 2; i < token.size(); ++i) {
                    double value = 0;
                    if (parseDouble(token[i], value)) {
                        zone.cartParam = value;
                        zone.hasCart = true;
                        zone.cartType = "speed";
                    }
                }
            } else if (token.size() >= 3) {
                double value = 0;
                if (parseDouble(token[2], value)) {
                    MotionZone& zone = zoneOf(out, token[1]);
                    zone.cartParam = value;
                    zone.hasCart = true;
                }
            }
        }
    };
    std::stringstream lines(text);
    std::string line;
    std::string pending;
    while (std::getline(lines, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        const auto first = line.find_first_not_of(" \t");
        if (first != std::string::npos && (line[first] == '*' || line[first] == '!' || line[first] == '%')) {
            continue;
        }
        pending += line;
        pending.push_back(' ');
        std::size_t semi = std::string::npos;
        while ((semi = pending.find(';')) != std::string::npos) {
            flush(pending.substr(0, semi));
            pending = pending.substr(semi + 1);
        }
    }
    flush(pending);
    std::sort(out.joints.begin(), out.joints.end(), [](const MotionAxisRate& a, const MotionAxisRate& b) {
        const int ia = jointIndex(a.name);
        const int ib = jointIndex(b.name);
        if (ia < 0 || ib < 0) {
            return a.name < b.name;
        }
        return ia < ib;
    });
    error.clear();
    return true;
}

bool parseMachineDat(const std::string& text, MachineData& out, std::string& error) {
    out = {};
    std::stringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        const auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == ';' || line[first] == '&') {
            continue;
        }
        line = stripComment(line.substr(first));
        const auto eq = line.find('=');
        const auto dollar = line.find('$');
        if (eq == std::string::npos || dollar == std::string::npos || dollar > eq) {
            continue;
        }
        if (dollar > 0) {
            bool typePrefix = true;
            for (std::size_t i = 0; i < dollar; ++i) {
                const unsigned char c = static_cast<unsigned char>(line[i]);
                if (!(std::isalpha(c) || std::isspace(c))) {
                    typePrefix = false;
                    break;
                }
            }
            if (!typePrefix) {
                continue;
            }
            line = line.substr(dollar);
        }
        const auto split = line.find('=');
        if (split == std::string::npos) {
            continue;
        }
        const std::string lhs = lowerCopy(line.substr(0, split));
        std::string rhs = line.substr(split + 1);
        const auto trim = rhs.find_first_not_of(" \t");
        if (trim != std::string::npos) {
            rhs = rhs.substr(trim);
        }
        int index = 0;
        int indexB = 0;
        std::string symbol = lhs;
        const auto bracket = lhs.find('[');
        if (bracket != std::string::npos) {
            symbol = lhs.substr(0, bracket);
            const auto comma = lhs.find(',', bracket);
            const auto end = lhs.find(']', bracket);
            if (comma != std::string::npos && end != std::string::npos) {
                index = indexOr(lhs.substr(bracket + 1, comma - bracket - 1), 0);
                indexB = indexOr(lhs.substr(comma + 1, end - comma - 1), 0);
            } else if (end != std::string::npos) {
                index = indexOr(lhs.substr(bracket + 1, end - bracket - 1), 0);
            }
        }
        auto frameOf = [&](MachineFrame& frame) {
            frame.present = true;
            fieldNumber(rhs, "x", frame.x);
            fieldNumber(rhs, "y", frame.y);
            fieldNumber(rhs, "z", frame.z);
            fieldNumber(rhs, "a", frame.a);
            fieldNumber(rhs, "b", frame.b);
            fieldNumber(rhs, "c", frame.c);
        };
        if (symbol == "$axis_type" && index >= 1 && index <= 12) {
            double value = 0;
            if (parseDouble(rhs, value)) {
                machineAxis(out, index).type = static_cast<int>(std::lround(value));
            }
        } else if (symbol == "$softn_end" && index >= 1 && index <= 12) {
            parseDouble(rhs, machineAxis(out, index).lower);
        } else if (symbol == "$softp_end" && index >= 1 && index <= 12) {
            parseDouble(rhs, machineAxis(out, index).upper);
        } else if (symbol == "$rat_mot_ax" && index >= 1 && index <= 12) {
            MachineAxis& axis = machineAxis(out, index);
            fieldNumber(rhs, "n", axis.gearNum);
            fieldNumber(rhs, "d", axis.gearDen);
        } else if (symbol == "$vel_axis_ma" && index >= 1 && index <= 12) {
            parseDouble(rhs, machineAxis(out, index).motorRpm);
        } else if (symbol == "$axis_jerk" && index >= 1 && index <= 12) {
            parseDouble(rhs, machineAxis(out, index).jerk);
        } else if (symbol == "$axis_dir" && index >= 1 && index <= 12) {
            double value = 0;
            if (parseDouble(rhs, value)) {
                machineAxis(out, index).direction = static_cast<int>(std::lround(value));
            }
        } else if (symbol == "$coup_comp" && index >= 1 && index <= 6 && indexB >= 1 && indexB <= 6) {
            double numerator = 0;
            double denominator = 1;
            fieldNumber(rhs, "n", numerator);
            fieldNumber(rhs, "d", denominator);
            if (std::abs(numerator) > 1e-12 && std::abs(denominator) > 1e-12) {
                machineAxis(out, index).couples.emplace_back(indexB, numerator / denominator);
            }
        } else if (symbol == "$tiroro") {
            frameOf(out.tiroro);
        } else if (symbol == "$tx3p3") {
            frameOf(out.tx3p3);
        } else if (symbol == "$tflwp") {
            frameOf(out.tflwp);
        } else if (symbol == "$length_a") {
            out.hasLengthA = parseDouble(rhs, out.lengthA);
        } else if (symbol == "$length_b") {
            out.hasLengthB = parseDouble(rhs, out.lengthB);
        } else if (symbol == "$dh_4") {
            out.hasDh4 = fieldNumber(rhs, "dhart_alpha", out.dh4Alpha);
        } else if (symbol == "$dh_5") {
            out.hasDh5 = fieldNumber(rhs, "dhart_alpha", out.dh5Alpha);
        } else if (symbol == "$trafoname") {
            const auto quote = rhs.find('"');
            const auto end = rhs.find('"', quote == std::string::npos ? 0 : quote + 1);
            if (quote != std::string::npos && end != std::string::npos) {
                out.trafoName = rhs.substr(quote + 1, end - quote - 1);
            }
        } else if (symbol == "$kinclass") {
            out.kinClass = rhs;
        } else if (symbol == "$active_axis_mask") {
            out.activeMask = parseBinaryMask(rhs);
        }
    }
    finishMachine(out);
    error.clear();
    return true;
}

bool parseComponentInfo(const std::string& text, std::vector<InfoAxisLimit>& axes, std::string& error) {
    axes.clear();
    std::stringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        const std::string lowered = lowerCopy(line);
        const auto at = lowered.find("axis");
        if (at == std::string::npos) {
            continue;
        }
        std::size_t i = at + 4;
        while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) {
            ++i;
        }
        if (i >= line.size() || !std::isdigit(static_cast<unsigned char>(line[i]))) {
            continue;
        }
        int index = 0;
        while (i < line.size() && std::isdigit(static_cast<unsigned char>(line[i]))) {
            index = index * 10 + (line[i] - '0');
            ++i;
        }
        double lower = 0;
        double upper = 0;
        const std::vector<std::string> token = words(line.substr(i));
        if (index <= 0 || token.size() < 2 || !parseDouble(token[0], lower) || !parseDouble(token[1], upper)) {
            continue;
        }
        InfoAxisLimit limit;
        limit.index = index;
        limit.lower = lower;
        limit.upper = upper;
        axes.push_back(limit);
    }
    error.clear();
    return !axes.empty();
}

namespace {

struct Side {
    std::filesystem::path folder;
    CojtGraph graph;
    bool hasGraph = false;
    MotionParameters motion;
    bool hasMotion = false;
    MachineData machine;
    bool hasMachine = false;
    std::vector<InfoAxisLimit> info;
    std::string detail;
};

bool loadSide(const std::filesystem::path& folder, Side& side, std::string& error) {
    side.folder = folder;
    const auto graphPath = folder / "kin_graph.xml";
    if (std::filesystem::exists(graphPath)) {
        const std::string text = readTextFile(graphPath, error);
        if (!error.empty() || !parseKinGraphXml(text, side.graph, error)) {
            return false;
        }
        side.hasGraph = true;
    }
    const auto motionPath = folder / "motionparameters.e";
    if (std::filesystem::exists(motionPath)) {
        const std::string text = readTextFile(motionPath, error);
        if (!error.empty() || !parseMotionParameters(text, side.motion, error)) {
            return false;
        }
        side.hasMotion = true;
    }
    const auto machinePath = findMachineDat(folder);
    if (!machinePath.empty()) {
        const std::string text = readTextFile(machinePath, error);
        if (!error.empty() || !parseMachineDat(text, side.machine, error)) {
            return false;
        }
        side.hasMachine = side.machine.ok;
    }
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(folder, ec)) {
        const std::string filename = lowerCopy(entry.path().filename().string());
        const bool infoFile = filename == ".info" || lowerCopy(entry.path().extension().string()) == ".info";
        if (ec || !entry.is_regular_file() || !infoFile) {
            continue;
        }
        const std::string text = readTextFile(entry.path(), error);
        std::vector<InfoAxisLimit> parsed;
        if (!error.empty() || !parseComponentInfo(text, parsed, error)) {
            error.clear();
            continue;
        }
        side.info = std::move(parsed);
        break;
    }
    const auto tune = folder / "TuneData.xml";
    if (std::filesystem::exists(tune)) {
        const std::string text = readTextFile(tune, error);
        error.clear();
        const auto at = lowerCopy(text).find("<weight>");
        if (at != std::string::npos) {
            side.detail = text;
        }
    }
    error.clear();
    return true;
}

double tuneWeight(const std::string& text) {
    const auto at = lowerCopy(text).find("<weight>");
    if (at == std::string::npos) {
        return 0;
    }
    const auto start = text.find('>', at);
    if (start == std::string::npos) {
        return 0;
    }
    double value = 0;
    parseDouble(text.substr(start + 1), value);
    return value;
}

int usedAxes(const MachineData& machine) {
    int count = 0;
    for (const MachineAxis& axis : machine.axes) {
        if (axis.used) {
            ++count;
        }
    }
    return count;
}

void appendSource(std::ostringstream& source, const Side& side) {
    if (side.hasGraph) {
        source << "kin_graph 连杆 " << side.graph.links.size() << "，关节边 " << side.graph.edges.size() << "。";
        if (side.graph.edges.empty() && !side.graph.links.empty()) {
            source << "这份图没有关节边，轴向不从图里来。";
        }
    }
    if (side.hasMachine) {
        source << " $machine.dat";
        if (!side.machine.trafoName.empty()) {
            source << " " << side.machine.trafoName;
        }
        if (!side.machine.kinClass.empty()) {
            source << " " << side.machine.kinClass;
        }
        source << "。";
        if (side.machine.tiroro.present) {
            source << " d1=$TIRORO.Z " << side.machine.tiroro.z << "。";
        }
        if (side.machine.hasLengthA) {
            source << " shoulder=$LENGTH_A " << side.machine.lengthA << "。";
        }
        if (side.machine.hasLengthB) {
            source << " a2=$LENGTH_B " << side.machine.lengthB << "。";
        }
        if (side.machine.tx3p3.present) {
            source << " $TX3P3 X " << side.machine.tx3p3.x << " Y " << side.machine.tx3p3.y << " Z " << side.machine.tx3p3.z
                   << "。TX3P3.Z 没有并进 a3。";
        }
        if (side.machine.tflwp.present) {
            source << " d6=$TFLWP.Z " << side.machine.tflwp.z << "。";
        }
        source << " 转动轴速度 = |$VEL_AXIS_MA| * 6 / |$RAT_MOT_AX|，单位 deg/s。直线轴的减速比没有换算。$AXIS_JERK 只记录，不当成加速度。"
                  "$AXIS_DIR 是电机方向，没有用来翻转几何轴。关节 0 是导入时的装配姿势，不是 $AXIS_HOME。";
        for (const MachineAxis& axis : side.machine.axes) {
            if (!axis.used) {
                continue;
            }
            source << " A" << axis.index << " 限位 " << axis.lower << ".." << axis.upper << " 减速比 " << axis.gearNum << "/"
                   << axis.gearDen;
            if (axis.hasSpeed) {
                source << " 速度 " << axis.speed;
            }
            if (axis.jerk != 0) {
                source << " jerk " << axis.jerk;
            }
            if (axis.couples.size() > 1) {
                source << " 耦合";
                for (const auto& couple : axis.couples) {
                    source << " A" << couple.first << ":" << couple.second;
                }
                source << " 多于一个主动轴，没有写进单主动轴字段。";
            }
            source << "。";
        }
    }
    if (side.hasMotion) {
        source << " motionparameters.e";
        if (!side.motion.configFamily.empty()) {
            source << " " << side.motion.configFamily;
        }
        source << "。";
        if (side.motion.hasCartSpeed) {
            source << " 笛卡尔速度 " << side.motion.cartMaxLinSpeed << "。";
        }
        if (side.motion.hasCartAcc) {
            source << " 笛卡尔加速度 " << side.motion.cartMaxLinAcc << "。";
        }
        for (const MotionAxisRate& rate : side.motion.joints) {
            if (rate.hasSpeed) {
                source << " " << rate.name << " " << rate.speedDeg << " deg/s";
                if (rate.hasAccTime && rate.accTime > 1e-9) {
                    source << " 加速度 " << rate.speedDeg / rate.accTime;
                }
                if (rate.settle > 0 || rate.delay > 0) {
                    source << " settle " << rate.settle << " delay " << rate.delay;
                }
                source << "。";
            }
        }
        for (const MotionZone& zone : side.motion.zones) {
            source << " zone " << zone.name;
            if (!zone.jointParam.empty()) {
                source << " joint " << zone.jointParam.front();
            }
            if (zone.hasCart) {
                source << " cart " << zone.cartParam;
            }
            source << "。";
        }
        source << " joint_max_speed 小于 20 的按 rad/s 换成度。settle 和 delay 没有加进梯形规划。";
    }
    if (!side.info.empty()) {
        source << " .info 限位";
        for (const InfoAxisLimit& limit : side.info) {
            source << " A" << limit.index << " " << limit.lower << ".." << limit.upper;
        }
        source << "。";
    }
}

void fillWrist(KinematicResource& resource, const MachineData& machine) {
    if (machine.tiroro.present) {
        resource.wrist.d1 = machine.tiroro.z;
    }
    if (machine.hasLengthA) {
        resource.wrist.shoulder = machine.lengthA;
    }
    if (machine.hasLengthB) {
        resource.wrist.a2 = machine.lengthB;
    }
    if (machine.tx3p3.present) {
        resource.wrist.a3 = machine.tx3p3.x;
    }
    if (machine.tflwp.present) {
        resource.wrist.d6 = machine.tflwp.z;
    }
}

Vec3 dhAxis(const MachineData& machine, int index) {
    if (index == 1) {
        return {0, 0, 1};
    }
    if (index == 2 || index == 3) {
        return {0, 1, 0};
    }
    const Quat frame = kukaEuler(machine.tx3p3.a, machine.tx3p3.b, machine.tx3p3.c);
    if (index == 4) {
        return frame.rotate({0, 0, 1});
    }
    const double alpha = index == 5 ? (machine.hasDh4 ? machine.dh4Alpha : 90.0) : (machine.hasDh5 ? machine.dh5Alpha : -90.0);
    const Quat rx = Quat::fromAxisAngle({1, 0, 0}, deg2rad(alpha));
    return rx.rotate({0, 0, 1});
}

Transform dhRest(const MachineData& machine, int childIndex) {
    Transform local;
    if (childIndex == 2) {
        local.translation = {machine.hasLengthA ? machine.lengthA : 0, 0, machine.tiroro.present ? machine.tiroro.z : 0};
    } else if (childIndex == 3) {
        local.translation = {0, 0, machine.hasLengthB ? machine.lengthB : 0};
    } else if (childIndex == 4 && machine.tx3p3.present) {
        local.translation = {machine.tx3p3.x, machine.tx3p3.y, machine.tx3p3.z};
        local.rotation = kukaEuler(machine.tx3p3.a, machine.tx3p3.b, machine.tx3p3.c);
    }
    return local;
}

void discoverKLinks(const Scene& scene, std::vector<CojtGraphLink>& links) {
    if (!links.empty() || !scene.root()) {
        return;
    }
    struct Hit {
        int index = 0;
        int depth = 1000000;
        std::string name;
    };
    std::vector<Hit> hits;
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
        const std::string stem = stemOf(node->name);
        if (stem.size() < 2 || stem[0] != 'k' || !std::isdigit(static_cast<unsigned char>(stem[1]))) {
            continue;
        }
        char* end = nullptr;
        const long index = std::strtol(stem.c_str() + 1, &end, 10);
        if (end == stem.c_str() + 1 || index <= 0) {
            continue;
        }
        const int depth = depthOf(node);
        bool found = false;
        for (Hit& hit : hits) {
            if (hit.index == static_cast<int>(index)) {
                found = true;
                if (depth < hit.depth) {
                    hit.depth = depth;
                    hit.name = stem;
                }
            }
        }
        if (!found) {
            hits.push_back({static_cast<int>(index), depth, stem});
        }
    }
    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.index < b.index; });
    if (hits.size() < 2) {
        return;
    }
    for (const Hit& hit : hits) {
        CojtGraphLink link;
        link.name = hit.name;
        links.push_back(link);
    }
}

void attachCojtSidecar(const std::string& jtOrFolder, Scene& scene, std::string& note) {
    const std::filesystem::path folder = componentFolder(jtOrFolder);
    if (!folderIsCojt(folder)) {
        note.clear();
        return;
    }
    Side side;
    std::string error;
    if (!loadSide(folder, side, error)) {
        note = "cojt " + error;
        return;
    }
    const std::string name = componentName(folder);
    const bool track = looksLikeTrack(folder.filename().string(), side.graph, side.motion, side.machine, side.info);
    const int machineCount = side.hasMachine ? usedAxes(side.machine) : 0;
    int jointCount = 0;
    if (track) {
        jointCount = 1;
    } else if (machineCount > 0) {
        jointCount = machineCount;
    } else if (!side.graph.edges.empty()) {
        jointCount = static_cast<int>(side.graph.edges.size());
    } else if (!side.motion.joints.empty()) {
        jointCount = static_cast<int>(side.motion.joints.size());
    } else if (!side.info.empty()) {
        jointCount = static_cast<int>(side.info.size());
    }
    if (!track && side.graph.links.empty()) {
        std::vector<CojtGraphLink> discovered;
        discoverKLinks(scene, discovered);
        if (discovered.size() >= 2 && (jointCount == 0 || static_cast<int>(discovered.size()) >= jointCount + 1)) {
            side.graph.links = std::move(discovered);
        }
    }
    const int linkCount = track ? 2 : (jointCount > 0 ? jointCount + 1 : static_cast<int>(side.graph.links.size()));
    if (linkCount <= 0 && jointCount <= 0 && !side.hasMotion && !side.hasGraph) {
        note = "cojt " + name + ": links=0 edges=0 joints=0 matched=0 axis-frames=0";
        return;
    }

    scene.beginEdit();
    scene.resources.erase(std::remove_if(scene.resources.begin(), scene.resources.end(),
                                         [&](const KinematicResource& item) { return item.name == name; }),
                          scene.resources.end());
    KinematicResource resource;
    resource.name = name;
    resource.ik = "numerical";
    resource.payload = tuneWeight(side.detail);
    if (side.hasMachine) {
        fillWrist(resource, side.machine);
    } else {
        resource.wrist.d1 = 0;
        resource.wrist.shoulder = 0;
        resource.wrist.a2 = 0;
        resource.wrist.a3 = 0;
        resource.wrist.d6 = 0;
    }
    std::ostringstream source;
    appendSource(source, side);
    const SceneIndex indexed = indexScene(scene);
    std::vector<std::string> usedIds;
    std::vector<BuiltLink> links;
    auto takeName = [&](int index) {
        if (index >= 0 && index < static_cast<int>(side.graph.links.size()) && !side.graph.links[static_cast<std::size_t>(index)].name.empty()) {
            return side.graph.links[static_cast<std::size_t>(index)].name;
        }
        if (track) {
            return index == 0 ? std::string("base") : std::string("carriage");
        }
        if (index == 0) {
            return std::string("base");
        }
        return "A" + std::to_string(index);
    };
    int matched = 0;
    for (int i = 0; i < linkCount; ++i) {
        BuiltLink built;
        built.name = takeName(i);
        const std::string guid = i < static_cast<int>(side.graph.links.size()) ? side.graph.links[static_cast<std::size_t>(i)].guid : std::string();
        built.node = findStem(indexed, built.name, guid, usedIds);
        if (built.node) {
            built.matched = true;
            ++matched;
            usedIds.push_back(built.node->id);
        }
        links.push_back(built);
    }
    for (int i = linkCount; i < static_cast<int>(side.graph.links.size()); ++i) {
        BuiltLink extra;
        extra.name = side.graph.links[static_cast<std::size_t>(i)].name;
        extra.node = findStem(indexed, extra.name, side.graph.links[static_cast<std::size_t>(i)].guid, usedIds);
        if (extra.node) {
            extra.matched = true;
            ++matched;
            usedIds.push_back(extra.node->id);
        }
        links.push_back(extra);
    }
    for (int i = 0; i < static_cast<int>(links.size()); ++i) {
        BuiltLink& built = links[static_cast<std::size_t>(i)];
        if (!built.node) {
            built.node = scene.makeNode(NodeKind::Assembly, built.name);
            std::shared_ptr<SceneNode> parent = scene.root();
            if (i > 0 && i <= jointCount && links[static_cast<std::size_t>(i - 1)].node) {
                parent = links[static_cast<std::size_t>(i - 1)].node;
            } else if (i > jointCount && jointCount > 0 && links[static_cast<std::size_t>(jointCount)].node) {
                parent = links[static_cast<std::size_t>(jointCount)].node;
            }
            if (side.hasMachine && i >= 1 && i <= jointCount) {
                built.node->local = dhRest(side.machine, i);
            }
            scene.addChild(parent, built.node);
        }
        KinLink link;
        link.name = built.name;
        link.nodeId = built.node->id;
        resource.links.push_back(link);
    }

    int axisFrames = 0;
    int ignoredFrames = 0;
    bool placeholderAxis = false;
    std::vector<MachineAxis> used;
    if (side.hasMachine) {
        for (const MachineAxis& axis : side.machine.axes) {
            if (axis.used) {
                used.push_back(axis);
            }
        }
    }
    const bool dhChain = side.hasMachine && (side.machine.hasLengthA || side.machine.hasLengthB || side.machine.tx3p3.present);
    auto addOne = [&](int ordinal, const std::string& jointName, int parentIndex, int childIndex, JointMotion motion, Vec3 axis,
                      bool measuredAxis) {
        if (parentIndex < 0 || childIndex < 0 || parentIndex >= static_cast<int>(links.size()) ||
            childIndex >= static_cast<int>(links.size())) {
            return;
        }
        KinJoint joint;
        joint.name = jointName;
        joint.parentLink = links[static_cast<std::size_t>(parentIndex)].name;
        joint.childLink = links[static_cast<std::size_t>(childIndex)].name;
        joint.motion = motion;
        joint.maxSpeed = 0;
        joint.maxAccel = 0;
        joint.accelSource = "velocity";
        joint.limited = true;
        setRest(joint, links[static_cast<std::size_t>(parentIndex)].node, links[static_cast<std::size_t>(childIndex)].node, scene);
        Vec3 anchor = joint.restLocal.translation;
        if (!measuredAxis) {
            if (const auto frame = findStem(indexed, "ax" + std::to_string(ordinal), {}, usedIds)) {
                if (links[static_cast<std::size_t>(parentIndex)].node) {
                    bool accepted = false;
                    axis = axisFromFrame(scene, frame, links[static_cast<std::size_t>(parentIndex)].node->worldMatrix(), anchor,
                                         accepted);
                    if (accepted) {
                        measuredAxis = true;
                        ++axisFrames;
                    } else {
                        ++ignoredFrames;
                    }
                }
            }
        }
        if (!measuredAxis && dhChain && motion == JointMotion::Revolute) {
            axis = dhAxis(side.machine, ordinal);
            anchor = joint.restLocal.translation;
        } else if (!measuredAxis) {
            placeholderAxis = true;
        }
        if (length(axis) < 1e-9) {
            axis = {0, 0, 1};
            placeholderAxis = true;
        }
        joint.axis = normalize(axis);
        joint.anchor = anchor;
        if (ordinal - 1 < static_cast<int>(used.size())) {
            const MachineAxis& data = used[static_cast<std::size_t>(ordinal - 1)];
            if (!data.rotational) {
                joint.motion = JointMotion::Prismatic;
            }
            joint.lower = data.lower;
            joint.upper = data.upper;
            joint.limited = !data.endless && std::abs(data.upper - data.lower) > 1e-9;
            if (data.hasSpeed) {
                joint.maxSpeed = data.speed;
            }
            if (data.couples.size() == 1) {
                joint.coupledTo = "A" + std::to_string(data.couples.front().first);
                joint.coupleRatio = data.couples.front().second;
            }
        }
        if (ordinal - 1 < static_cast<int>(side.info.size()) && (used.empty() || !joint.limited)) {
            const InfoAxisLimit& limit = side.info[static_cast<std::size_t>(ordinal - 1)];
            joint.lower = limit.lower;
            joint.upper = limit.upper;
            joint.limited = true;
        }
        if (ordinal - 1 < static_cast<int>(side.motion.joints.size())) {
            const MotionAxisRate& rate = side.motion.joints[static_cast<std::size_t>(ordinal - 1)];
            if (rate.hasSpeed && joint.maxSpeed <= 0) {
                joint.maxSpeed = rate.speedDeg;
            }
            if (rate.hasAccTime && rate.accTime > 1e-9 && joint.maxSpeed > 0) {
                joint.maxAccel = joint.maxSpeed / rate.accTime;
                joint.accelSource = "motionparameters";
            }
        }
        if (!side.graph.edges.empty() && ordinal - 1 < static_cast<int>(side.graph.edges.size())) {
            const CojtGraphEdge& edge = side.graph.edges[static_cast<std::size_t>(ordinal - 1)];
            if (edge.hasLimits && used.empty()) {
                joint.lower = edge.lower;
                joint.upper = edge.upper;
                joint.limited = true;
            }
            if (edge.hasSpeed && joint.maxSpeed <= 0) {
                joint.maxSpeed = edge.speed;
            }
            if (!edge.coupledTo.empty() && joint.coupledTo.empty()) {
                joint.coupledTo = edge.coupledTo;
                joint.coupleRatio = edge.coupleRatio;
            }
            if (edge.prismatic) {
                joint.motion = JointMotion::Prismatic;
            }
        }
        resource.joints.push_back(joint);
    };

    if (track) {
        const Aabb box = meshBounds(scene);
        Vec3 axis{1, 0, 0};
        double span = 0;
        std::string travel = "component name";
        if (box.valid) {
            const Vec3 size = box.size();
            if (size.y >= size.x && size.y >= size.z) {
                axis = {0, 1, 0};
                span = size.y;
            } else if (size.z >= size.x && size.z >= size.y) {
                axis = {0, 0, 1};
                span = size.z;
            } else {
                span = size.x;
            }
            if (span > 100.0) {
                travel = "mesh bounds";
            } else {
                span = 0;
            }
        }
        if (span <= 0) {
            span = nominalMillimeters(folder.filename().string());
            travel = span > 0 ? "component name" : "missing";
        }
        KinJoint joint;
        joint.name = "E1";
        joint.parentLink = links[0].name;
        joint.childLink = links[1].name;
        joint.motion = JointMotion::Prismatic;
        joint.axis = axis;
        joint.externalName = "E1";
        joint.externalKind = "track";
        joint.lower = 0;
        joint.upper = span;
        joint.limited = span > 0;
        joint.maxSpeed = 0;
        joint.maxAccel = 0;
        joint.accelSource = "velocity";
        if (links[1].node && links[0].node) {
            const Vec3 low = box.valid ? (box.center() - axis * (span * 0.5)) : Vec3{};
            const Mat4 parent = links[0].node->worldMatrix();
            const Vec3 local = transformPoint(inverseAffine(parent), low);
            links[1].node->local.translation = local;
            links[1].node->local.rotation = Quat::identity();
            joint.restLocal = links[1].node->local;
            joint.anchor = local;
        }
        resource.joints.push_back(joint);
        source << " 地轨 E1 行程 " << span << " mm，来源 " << travel << "。组件没有关节速度。";
        if (travel != "mesh bounds") {
            source << " JT 三角形没有放进场景时，行程取组件名里的米，不是软限位。";
        }
    } else if (machineCount > 0 && side.graph.edges.size() == static_cast<std::size_t>(machineCount)) {
        for (int i = 0; i < machineCount; ++i) {
            const CojtGraphEdge& edge = side.graph.edges[static_cast<std::size_t>(i)];
            const int from = edge.from >= 0 ? edge.from : i;
            const int to = edge.to >= 0 ? edge.to : i + 1;
            addOne(i + 1, edge.name.empty() ? "A" + std::to_string(i + 1) : edge.name, from, to,
                   edge.prismatic ? JointMotion::Prismatic : JointMotion::Revolute, edge.hasAxis ? edge.axis : Vec3{0, 0, 1},
                   edge.hasAxis);
        }
    } else if (machineCount > 0 || !side.motion.joints.empty() || !side.info.empty()) {
        const int count = jointCount;
        for (int i = 0; i < count; ++i) {
            const bool prismatic = i < static_cast<int>(used.size()) && !used[static_cast<std::size_t>(i)].rotational;
            addOne(i + 1, "A" + std::to_string(i + 1), i, i + 1, prismatic ? JointMotion::Prismatic : JointMotion::Revolute,
                   {0, 0, 1}, false);
        }
    } else if (!side.graph.edges.empty()) {
        for (int i = 0; i < static_cast<int>(side.graph.edges.size()); ++i) {
            const CojtGraphEdge& edge = side.graph.edges[static_cast<std::size_t>(i)];
            addOne(i + 1, edge.name, edge.from, edge.to, edge.prismatic ? JointMotion::Prismatic : JointMotion::Revolute,
                   edge.hasAxis ? edge.axis : Vec3{0, 0, 1}, edge.hasAxis);
        }
    }
    if (placeholderAxis) {
        source << " 没有轴帧、也没有 DH 的关节用了 +Z，这不是测得的轴向。";
    }
    if (ignoredFrames > 0) {
        source << " " << ignoredFrames
               << " 个 ax 节点没有细长网格，Z 也和父连杆重合，没有当成关节轴。";
    }
    if (dhChain && axisFrames == 0 && !track) {
        source << " 轴向按 KUKA 数学零位：A1 绕 Z，A2/A3 绕 Y，A4 沿 $TX3P3 的 Z，A5/A6 按 $DH alpha 绕 X 转过的 Z。";
    }
    for (int i = linkCount; i < static_cast<int>(links.size()); ++i) {
        if (links[static_cast<std::size_t>(i)].name.empty()) {
            continue;
        }
        KinLink* link = findLink(resource, links[static_cast<std::size_t>(i)].name);
        const int mountIndex = std::min(linkCount - 1, jointCount);
        if (link == nullptr || mountIndex < 0 || !links[static_cast<std::size_t>(mountIndex)].node || !links[static_cast<std::size_t>(i)].node) {
            continue;
        }
        if (isAncestor(links[static_cast<std::size_t>(i)].node, links[static_cast<std::size_t>(mountIndex)].node)) {
            continue;
        }
        scene.reparent(links[static_cast<std::size_t>(i)].node->id, links[static_cast<std::size_t>(mountIndex)].node->id, true);
        link->mountedOn = links[static_cast<std::size_t>(mountIndex)].name;
        link->mountRest = links[static_cast<std::size_t>(i)].node->local;
    }
    if (track && links[0].node) {
        std::vector<std::shared_ptr<SceneNode>> top;
        if (scene.root()) {
            top = scene.root()->children;
        }
        for (const auto& child : top) {
            if (!child || child == links[0].node || child == links[1].node) {
                continue;
            }
            if (!isAncestor(child, links[0].node)) {
                scene.reparent(child->id, links[0].node->id, true);
            }
        }
    }
    if (!resource.links.empty()) {
        NamedFrame tcp;
        tcp.name = "tcp";
        tcp.link = resource.links[std::min(resource.links.size() - 1, static_cast<std::size_t>(std::max(jointCount, 0)))].name;
        if (side.hasMachine && side.machine.tflwp.present && axisFrames == 0 && !track) {
            tcp.origin = {0, 0, side.machine.tflwp.z};
        }
        resource.frames.push_back(tcp);
    }
    resource.source = source.str();
    scene.resources.push_back(std::move(resource));
    scene.endEdit();

    std::ostringstream summary;
    summary << "cojt " << name << ": links=" << linkCount << " edges=" << side.graph.edges.size() << " joints=" << jointCount
            << " matched=" << matched << " axis-frames=" << axisFrames;
    if (track) {
        summary << " external=E1";
    }
    note = summary.str();
}

}  // namespace

AssetImportResult importCojt(const std::string& path, Scene& scene) {
    const std::filesystem::path file(path);
    std::error_code ec;
    const bool directory = std::filesystem::is_directory(file, ec);
    const std::filesystem::path folder = directory ? file : file.parent_path();
    const std::string ext = lowerCopy(file.extension().string());
    if (!directory && ext == ".jt") {
        JtImporter importer;
        return importer.importFile(file.string(), scene);
    }
    const std::filesystem::path jt = findJt(folder);
    if (!jt.empty()) {
        JtImporter importer;
        return importer.importFile(jt.string(), scene);
    }
    AssetImportResult result;
    result.format = "cojt";
    std::string note;
    attachCojtSidecar((folder / "part.jt").string(), scene, note);
    result.ok = !note.empty() && note.find("cannot open") == std::string::npos && !scene.resources.empty();
    result.message = note.empty() ? "cojt folder has no kinematics" : note;
    result.nodeCount = scene.nodeCount();
    return result;
}

void attachCojtFromJt(const std::string& jtPath, Scene& scene, std::string& note) { attachCojtSidecar(jtPath, scene, note); }

int attachCojtVisual(Scene& scene, const std::string& folder, const std::string& parentId, Vec3 foot, std::string& note) {
    Scene side;
    const AssetImportResult imported = importCojt(folder, side);
    if (!imported.ok || imported.triangleCount <= 0 || !side.root()) {
        note = imported.message.empty() ? "cojt import failed" : imported.message;
        return 0;
    }
    std::unordered_map<std::string, std::string> meshMap;
    std::vector<std::string> roots;
    scene.beginEdit();
    std::function<void(const std::shared_ptr<SceneNode>&, const std::shared_ptr<SceneNode>&)> copy =
        [&](const std::shared_ptr<SceneNode>& from, const std::shared_ptr<SceneNode>& parent) {
            if (!from || !parent) {
                return;
            }
            for (const auto& child : from->children) {
                if (!child) {
                    continue;
                }
                // Assembly so the visual mesh is drawn but left out of the
                // convex decomposition the soft PLC runs on every scan.
                auto node = scene.makeNode(NodeKind::Assembly, "cad_" + child->name);
                node->local = child->local;
                node->color = child->color;
                node->visible = child->visible;
                node->faceAxes = child->faceAxes;
                if (!child->meshId.empty()) {
                    const auto existing = meshMap.find(child->meshId);
                    if (existing == meshMap.end()) {
                        meshMap.emplace(child->meshId, scene.addMesh(side.mesh(child->meshId)));
                    }
                    node->meshId = meshMap[child->meshId];
                }
                scene.addChild(parent, node);
                if (parent == scene.root()) {
                    roots.push_back(node->id);
                }
                copy(child, node);
            }
        };
    copy(side.root(), scene.root());

    std::function<void(const SceneNode&, Aabb&)> expand = [&](const SceneNode& node, Aabb& box) {
        if (!node.meshId.empty()) {
            if (const auto meshData = scene.mesh(node.meshId); meshData && meshData->bounds.valid) {
                const Mat4 world = node.worldMatrix();
                const Vec3 lo = meshData->bounds.min;
                const Vec3 hi = meshData->bounds.max;
                for (int corner = 0; corner < 8; ++corner) {
                    const Vec3 local{(corner & 1) ? hi.x : lo.x, (corner & 2) ? hi.y : lo.y, (corner & 4) ? hi.z : lo.z};
                    box.expand(transformPoint(world, local));
                }
            }
        }
        for (const auto& child : node.children) {
            if (child) {
                expand(*child, box);
            }
        }
    };
    Aabb box;
    for (const std::string& id : roots) {
        if (const auto node = scene.find(id)) {
            expand(*node, box);
        }
    }
    if (!box.valid) {
        scene.endEdit();
        note = "cojt produced no mesh bounds";
        return 0;
    }
    const Vec3 delta = foot - Vec3{box.center().x, box.center().y, box.min.z};
    for (const std::string& id : roots) {
        if (const auto node = scene.find(id)) {
            node->local.translation = node->local.translation + delta;
        }
    }
    if (!parentId.empty()) {
        for (const std::string& id : roots) {
            scene.reparent(id, parentId, true);
        }
    }
    scene.endEdit();
    note = imported.message;
    return imported.triangleCount;
}

}  // namespace sim
