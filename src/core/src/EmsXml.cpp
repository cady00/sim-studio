#include "sim/core/LineModel.hpp"

#include "sim/core/Scene.hpp"
#include "sim/core/Xml.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace sim {
namespace {

std::string xmlEscape(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (unsigned char c : text) {
        if (c == '&') {
            out += "&amp;";
        } else if (c == '<') {
            out += "&lt;";
        } else if (c == '>') {
            out += "&gt;";
        } else if (c == '"') {
            out += "&quot;";
        } else {
            out.push_back(static_cast<char>(c));
        }
    }
    return out;
}

std::string localName(std::string name) {
    const auto colon = name.rfind(':');
    if (colon != std::string::npos) {
        name = name.substr(colon + 1);
    }
    for (char& c : name) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return name;
}

bool sameText(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

bool oneOf(const std::string& name, std::initializer_list<const char*> keys) {
    for (const char* key : keys) {
        if (sameText(name, key)) {
            return true;
        }
    }
    return false;
}

std::string attrOf(const XmlNode& node, std::initializer_list<const char*> keys) {
    for (const char* key : keys) {
        for (const auto& [name, value] : node.attributes) {
            if (sameText(name, key) && !value.empty()) {
                return value;
            }
        }
    }
    return {};
}

bool parseDouble(const std::string& text, double& out) {
    if (text.empty()) {
        return false;
    }
    try {
        std::size_t used = 0;
        out = std::stod(text, &used);
        return used > 0;
    } catch (const std::exception&) {
        return false;
    }
}

std::string formatDouble(double value) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(6);
    out << value;
    std::string text = out.str();
    while (text.size() > 1 && text.back() == '0') {
        text.pop_back();
    }
    if (!text.empty() && text.back() == '.') {
        text.pop_back();
    }
    if (text == "-0") {
        text = "0";
    }
    return text;
}

void setAttr(std::vector<std::pair<std::string, std::string>>& attributes, const std::string& key, const std::string& value) {
    for (auto& [name, current] : attributes) {
        if (sameText(name, key)) {
            current = value;
            return;
        }
    }
    attributes.emplace_back(key, value);
}

std::string attrValue(const std::vector<std::pair<std::string, std::string>>& attributes, const std::string& key) {
    for (const auto& [name, value] : attributes) {
        if (sameText(name, key)) {
            return value;
        }
    }
    return {};
}

enum class EmsKind { None, Product, Process, Plant, Weld, Path, Location, Point, Normal, Container };

EmsKind classify(const std::string& name) {
    if (oneOf(name, {"product", "part", "component", "item", "pmpart", "pmproduct"})) {
        return EmsKind::Product;
    }
    if (oneOf(name, {"process", "operation", "task", "activity", "pmoperation"})) {
        return EmsKind::Process;
    }
    if (oneOf(name, {"plant", "resource", "station", "robot", "equipment", "pmresource", "pmrobot", "pmstation"})) {
        return EmsKind::Plant;
    }
    if (oneOf(name, {"weld", "weldpoint", "weldlocation", "txweldlocation", "txweldlocationoperation", "mfweldpoint", "spotweld"})) {
        return EmsKind::Weld;
    }
    if (oneOf(name, {"robotpath", "pmpath", "operationpath", "path"})) {
        return EmsKind::Path;
    }
    if (oneOf(name, {"location", "compoundlocation", "placement", "transform"})) {
        return EmsKind::Location;
    }
    if (oneOf(name, {"point", "position"})) {
        return EmsKind::Point;
    }
    if (oneOf(name, {"normal"})) {
        return EmsKind::Normal;
    }
    if (oneOf(name, {"ebop", "ppr", "ems", "emsxml", "pmproject", "project", "data", "objects"})) {
        return EmsKind::Container;
    }
    return EmsKind::None;
}

struct Pose {
    bool present = false;
    double x = 0;
    double y = 0;
    double z = 0;
    double rx = 0;
    double ry = 0;
    double rz = 0;
};

void readPose(const XmlNode& node, Pose& pose) {
    const auto take = [&](const char* key, double& slot) {
        double value = 0;
        if (parseDouble(attrOf(node, {key}), value)) {
            slot = value;
            pose.present = true;
        }
    };
    take("x", pose.x);
    take("y", pose.y);
    take("z", pose.z);
    take("rx", pose.rx);
    take("ry", pose.ry);
    take("rz", pose.rz);
}

void applyPose(Scene& scene, const std::string& name, const Pose& pose) {
    if (!pose.present || name.empty() || scene.root() == nullptr) {
        return;
    }
    std::shared_ptr<SceneNode> node = scene.findByName(name);
    if (!node) {
        node = scene.makeNode(NodeKind::Part, name);
        scene.addChild(scene.root(), node);
    }
    node->local.translation = {pose.x, pose.y, pose.z};
    node->local.rotation = Quat::fromEulerXYZ(deg2rad(pose.rx), deg2rad(pose.ry), deg2rad(pose.rz));
}

Pose poseOf(const SceneNode& node) {
    Pose pose;
    pose.present = true;
    pose.x = node.local.translation.x;
    pose.y = node.local.translation.y;
    pose.z = node.local.translation.z;
    node.local.rotation.normalized().toEulerXYZ(pose.rx, pose.ry, pose.rz);
    pose.rx = rad2deg(pose.rx);
    pose.ry = rad2deg(pose.ry);
    pose.rz = rad2deg(pose.rz);
    return pose;
}

void writePose(std::vector<std::pair<std::string, std::string>>& attributes, const Pose& pose) {
    setAttr(attributes, "x", formatDouble(pose.x));
    setAttr(attributes, "y", formatDouble(pose.y));
    setAttr(attributes, "z", formatDouble(pose.z));
    setAttr(attributes, "rx", formatDouble(pose.rx));
    setAttr(attributes, "ry", formatDouble(pose.ry));
    setAttr(attributes, "rz", formatDouble(pose.rz));
}

std::vector<double> parseList(const std::string& text) {
    std::vector<double> values;
    std::string token;
    for (char c : text) {
        if (c == ' ' || c == ',' || c == ';') {
            double value = 0;
            if (parseDouble(token, value)) {
                values.push_back(value);
            }
            token.clear();
        } else {
            token.push_back(c);
        }
    }
    double value = 0;
    if (parseDouble(token, value)) {
        values.push_back(value);
    }
    return values;
}

std::string formatList(const std::vector<double>& values) {
    std::string out;
    for (double value : values) {
        if (!out.empty()) {
            out.push_back(' ');
        }
        out += formatDouble(value);
    }
    return out;
}

PathMove pathMoveFrom(const std::string& text) {
    const std::string kind = localName(text);
    if (kind == "lin" || kind == "linear" || kind == "slin") {
        return PathMove::Lin;
    }
    if (kind == "circ" || kind == "circular") {
        return PathMove::Circ;
    }
    return PathMove::Ptp;
}

EmsNode capture(const XmlNode& xml) {
    EmsNode node;
    node.name = xml.name;
    node.text = xml.text;
    node.raw = xml.raw;
    for (const auto& [key, value] : xml.attributes) {
        node.attributes.emplace_back(key, value);
    }
    return node;
}

struct Parent {
    EmsKind kind = EmsKind::None;
    std::string id;
    std::string name;
};

EmsNode convertNode(const XmlNode& xml, const Parent& parent, Scene& scene, int& products, int& processes, int& plants, int& welds,
                    int& paths, int& nextId);

void placePart(Scene& scene, const std::string& name, const Pose& pose) { applyPose(scene, name, pose); }

EmsNode convertNode(const XmlNode& xml, const Parent& parent, Scene& scene, int& products, int& processes, int& plants, int& welds,
                    int& paths, int& nextId) {
    if (xml.name == "#comment") {
        EmsNode comment = capture(xml);
        comment.recognized = false;
        return comment;
    }
    const std::string kind = localName(xml.name);
    const EmsKind cls = classify(kind);
    EmsNode node = capture(xml);
    const auto ident = [&](const XmlNode& source) {
        std::string id = attrOf(source, {"id", "ID", "name", "Name"});
        if (id.empty()) {
            id = "n" + std::to_string(nextId++);
        }
        return id;
    };
    const auto label = [&](const XmlNode& source, const std::string& id) {
        std::string name = attrOf(source, {"name", "Name", "id", "ID"});
        return name.empty() ? id : name;
    };

    Parent childParent = parent;
    Pose pose;
    readPose(xml, pose);
    bool recognized = false;

    if (cls == EmsKind::Product) {
        ProductNode item;
        item.id = ident(xml);
        item.name = label(xml, item.id);
        item.parent = attrOf(xml, {"parent", "Parent"});
        item.partName = attrOf(xml, {"part", "Part", "partName"});
        if (item.partName.empty()) {
            item.partName = item.name;
        }
        scene.ppr.products.push_back(item);
        ++products;
        recognized = true;
        node.bind = "product:" + item.id;
        childParent = {EmsKind::Product, item.id, item.partName};
    } else if (cls == EmsKind::Process) {
        ProcessNode item;
        item.id = ident(xml);
        item.name = label(xml, item.id);
        item.parent = attrOf(xml, {"parent", "Parent"});
        item.operation = attrOf(xml, {"operation", "Operation"});
        if (item.operation.empty()) {
            item.operation = item.id;
        }
        item.product = attrOf(xml, {"product", "Product", "part", "Part"});
        scene.ppr.processes.push_back(item);
        Operation operation;
        operation.id = item.operation;
        operation.name = item.name;
        operation.kind = OpKind::Task;
        operation.station = attrOf(xml, {"station", "Station", "resource", "Resource"});
        operation.parent = item.parent;
        operation.path = attrOf(xml, {"path", "Path", "robotpath"});
        double seconds = 0;
        if (parseDouble(attrOf(xml, {"duration", "Duration", "seconds", "cycleTime", "CycleTime"}), seconds)) {
            operation.seconds = seconds;
        }
        const std::string after = attrOf(xml, {"after", "After", "predecessor", "Predecessor"});
        if (!after.empty()) {
            std::string token;
            for (char c : after) {
                if (c == ',' || c == ';') {
                    if (!token.empty()) {
                        operation.after.push_back(token);
                    }
                    token.clear();
                } else if (c != ' ') {
                    token.push_back(c);
                }
            }
            if (!token.empty()) {
                operation.after.push_back(token);
            }
        }
        bool exists = false;
        for (const Operation& current : scene.operations) {
            if (current.id == operation.id) {
                exists = true;
            }
        }
        if (!exists) {
            scene.operations.push_back(std::move(operation));
        }
        ++processes;
        recognized = true;
        node.bind = "operation:" + item.operation;
        childParent = {EmsKind::Process, item.operation, item.name};
    } else if (cls == EmsKind::Plant) {
        PlantNode item;
        item.id = ident(xml);
        item.name = label(xml, item.id);
        item.parent = attrOf(xml, {"parent", "Parent"});
        item.station = attrOf(xml, {"station", "Station"});
        if (item.station.empty()) {
            item.station = item.id;
        }
        item.resource = attrOf(xml, {"resource", "Resource"});
        scene.ppr.plants.push_back(item);
        ++plants;
        recognized = true;
        node.bind = "plant:" + item.id;
        childParent = {EmsKind::Plant, item.id, item.name};
    } else if (cls == EmsKind::Weld) {
        const std::string id = ident(xml);
        const std::string name = label(xml, id);
        WeldSpot spot;
        spot.id = id;
        spot.name = name;
        spot.part = attrOf(xml, {"part", "Part"});
        spot.electrode = attrOf(xml, {"electrode", "Electrode", "gun", "Gun"});
        double value = 0;
        if (parseDouble(attrOf(xml, {"x"}), value)) {
            spot.position.x = value;
        }
        if (parseDouble(attrOf(xml, {"y"}), value)) {
            spot.position.y = value;
        }
        if (parseDouble(attrOf(xml, {"z"}), value)) {
            spot.position.z = value;
        }
        if (parseDouble(attrOf(xml, {"nx", "normalx"}), value)) {
            spot.normal.x = value;
        }
        if (parseDouble(attrOf(xml, {"ny", "normaly"}), value)) {
            spot.normal.y = value;
        }
        if (parseDouble(attrOf(xml, {"nz", "normalz"}), value)) {
            spot.normal.z = value;
        }
        bool exists = false;
        for (const WeldSpot& current : scene.welds) {
            if (current.id == id) {
                exists = true;
            }
        }
        if (!exists) {
            scene.welds.push_back(std::move(spot));
        }
        ++welds;
        recognized = true;
        node.bind = "weld:" + id;
        childParent = {EmsKind::Weld, id, name};
    } else if (cls == EmsKind::Path) {
        const std::string name = label(xml, ident(xml));
        bool exists = false;
        for (const RobotPath& current : scene.robotPaths) {
            if (current.name == name) {
                exists = true;
            }
        }
        if (!exists) {
            RobotPath path;
            path.name = name;
            path.resource = attrOf(xml, {"resource", "Resource", "robot", "Robot"});
            scene.robotPaths.push_back(std::move(path));
        }
        if (parent.kind == EmsKind::Process) {
            for (Operation& operation : scene.operations) {
                if (operation.id == parent.id && operation.path.empty()) {
                    operation.path = name;
                }
            }
        }
        ++paths;
        recognized = true;
        node.bind = "path:" + name;
        childParent = {EmsKind::Path, name, name};
    } else if ((cls == EmsKind::Location || cls == EmsKind::Point) && parent.kind == EmsKind::Path) {
        RobotPath* path = nullptr;
        for (RobotPath& current : scene.robotPaths) {
            if (current.name == parent.id) {
                path = &current;
            }
        }
        if (path != nullptr) {
            PathPoint point;
            point.name = label(xml, "p" + std::to_string(path->points.size() + 1));
            point.move = pathMoveFrom(attrOf(xml, {"motion", "Motion", "move", "Move", "type", "Type"}));
            point.joints = parseList(attrOf(xml, {"joints", "Joints", "jointvalues", "JointValues"}));
            double value = 0;
            if (parseDouble(attrOf(xml, {"x"}), value)) {
                point.position.x = value;
                point.cartesian = point.joints.empty();
            }
            if (parseDouble(attrOf(xml, {"y"}), value)) {
                point.position.y = value;
                point.cartesian = point.joints.empty();
            }
            if (parseDouble(attrOf(xml, {"z"}), value)) {
                point.position.z = value;
                point.cartesian = point.joints.empty();
            }
            if (parseDouble(attrOf(xml, {"speed", "Speed"}), value)) {
                point.speed = value;
            }
            const int index = static_cast<int>(path->points.size());
            path->points.push_back(std::move(point));
            recognized = true;
            node.bind = "point:" + parent.id + ":" + std::to_string(index);
        }
    } else if ((cls == EmsKind::Point || cls == EmsKind::Normal || cls == EmsKind::Location) && parent.kind == EmsKind::Weld) {
        for (WeldSpot& spot : scene.welds) {
            if (spot.id != parent.id && spot.name != parent.id) {
                continue;
            }
            double value = 0;
            if (cls != EmsKind::Normal) {
                if (parseDouble(attrOf(xml, {"x"}), value)) {
                    spot.position.x = value;
                }
                if (parseDouble(attrOf(xml, {"y"}), value)) {
                    spot.position.y = value;
                }
                if (parseDouble(attrOf(xml, {"z"}), value)) {
                    spot.position.z = value;
                }
            }
            if (cls != EmsKind::Point) {
                if (parseDouble(attrOf(xml, {"x", "nx"}), value) && cls == EmsKind::Normal) {
                    spot.normal.x = value;
                }
                if (parseDouble(attrOf(xml, {"y", "ny"}), value) && cls == EmsKind::Normal) {
                    spot.normal.y = value;
                }
                if (parseDouble(attrOf(xml, {"z", "nz"}), value) && cls == EmsKind::Normal) {
                    spot.normal.z = value;
                }
            }
            recognized = true;
            node.bind = (cls == EmsKind::Normal ? "weldnormal:" : "weldpos:") + spot.id;
        }
    } else if ((cls == EmsKind::Location || cls == EmsKind::Point) &&
               (parent.kind == EmsKind::Product || parent.kind == EmsKind::Plant)) {
        recognized = true;
        node.bind = "xform:" + parent.name;
        childParent = parent;
    } else if (cls == EmsKind::Container) {
        recognized = true;
    }

    node.recognized = recognized;
    for (const XmlNode& child : xml.children) {
        node.children.push_back(convertNode(child, childParent, scene, products, processes, plants, welds, paths, nextId));
    }
    if (cls == EmsKind::Product || cls == EmsKind::Plant) {
        for (const EmsNode& child : node.children) {
            if (child.bind.rfind("xform:", 0) == 0) {
                Pose childPose;
                double value = 0;
                if (parseDouble(attrValue(child.attributes, "x"), value)) {
                    childPose.x = value;
                    childPose.present = true;
                }
                if (parseDouble(attrValue(child.attributes, "y"), value)) {
                    childPose.y = value;
                    childPose.present = true;
                }
                if (parseDouble(attrValue(child.attributes, "z"), value)) {
                    childPose.z = value;
                    childPose.present = true;
                }
                if (parseDouble(attrValue(child.attributes, "rx"), value)) {
                    childPose.rx = value;
                    childPose.present = true;
                }
                if (parseDouble(attrValue(child.attributes, "ry"), value)) {
                    childPose.ry = value;
                    childPose.present = true;
                }
                if (parseDouble(attrValue(child.attributes, "rz"), value)) {
                    childPose.rz = value;
                    childPose.present = true;
                }
                if (childPose.present) {
                    pose = childPose;
                }
            }
        }
        placePart(scene, childParent.name, pose);
    }
    return node;
}

void refreshNode(EmsNode& node, const Scene& scene) {
    if (node.bind.rfind("operation:", 0) == 0) {
        const std::string id = node.bind.substr(10);
        for (const Operation& operation : scene.operations) {
            if (operation.id != id) {
                continue;
            }
            setAttr(node.attributes, "name", operation.name);
            setAttr(node.attributes, "duration", formatDouble(operation.seconds));
            if (!operation.station.empty()) {
                setAttr(node.attributes, "station", operation.station);
            }
            if (!operation.path.empty()) {
                setAttr(node.attributes, "path", operation.path);
            }
            if (!operation.after.empty()) {
                setAttr(node.attributes, "after", operation.after.front());
            }
        }
    } else if (node.bind.rfind("weld:", 0) == 0) {
        const std::string id = node.bind.substr(5);
        for (const WeldSpot& spot : scene.welds) {
            if (spot.id != id && spot.name != id) {
                continue;
            }
            setAttr(node.attributes, "x", formatDouble(spot.position.x));
            setAttr(node.attributes, "y", formatDouble(spot.position.y));
            setAttr(node.attributes, "z", formatDouble(spot.position.z));
            setAttr(node.attributes, "nx", formatDouble(spot.normal.x));
            setAttr(node.attributes, "ny", formatDouble(spot.normal.y));
            setAttr(node.attributes, "nz", formatDouble(spot.normal.z));
        }
    } else if (node.bind.rfind("weldpos:", 0) == 0 || node.bind.rfind("weldnormal:", 0) == 0) {
        const bool normal = node.bind.rfind("weldnormal:", 0) == 0;
        const std::string id = node.bind.substr(normal ? 11 : 8);
        for (const WeldSpot& spot : scene.welds) {
            if (spot.id != id && spot.name != id) {
                continue;
            }
            const Vec3 value = normal ? spot.normal : spot.position;
            setAttr(node.attributes, "x", formatDouble(value.x));
            setAttr(node.attributes, "y", formatDouble(value.y));
            setAttr(node.attributes, "z", formatDouble(value.z));
        }
    } else if (node.bind.rfind("point:", 0) == 0) {
        const std::string rest = node.bind.substr(6);
        const auto split = rest.rfind(':');
        if (split != std::string::npos) {
            const std::string pathName = rest.substr(0, split);
            int index = 0;
            try {
                index = std::stoi(rest.substr(split + 1));
            } catch (const std::exception&) {
                index = -1;
            }
            for (const RobotPath& path : scene.robotPaths) {
                if (path.name != pathName || index < 0 || index >= static_cast<int>(path.points.size())) {
                    continue;
                }
                const PathPoint& point = path.points[static_cast<std::size_t>(index)];
                setAttr(node.attributes, "name", point.name);
                setAttr(node.attributes, "motion", pathMoveName(point.move));
                setAttr(node.attributes, "speed", formatDouble(point.speed));
                if (!point.joints.empty()) {
                    setAttr(node.attributes, "joints", formatList(point.joints));
                }
                if (point.cartesian || point.joints.empty()) {
                    setAttr(node.attributes, "x", formatDouble(point.position.x));
                    setAttr(node.attributes, "y", formatDouble(point.position.y));
                    setAttr(node.attributes, "z", formatDouble(point.position.z));
                }
            }
        }
    } else if (node.bind.rfind("xform:", 0) == 0 || node.bind.rfind("product:", 0) == 0 || node.bind.rfind("plant:", 0) == 0) {
        std::string name = node.bind.rfind("xform:", 0) == 0 ? node.bind.substr(6) : attrValue(node.attributes, "name");
        if (name.empty()) {
            name = attrValue(node.attributes, "part");
        }
        const auto found = scene.findByName(name);
        if (found && (node.bind.rfind("xform:", 0) == 0 || attrValue(node.attributes, "x").size() > 0 || found->local.translation.x != 0 ||
                      found->local.translation.y != 0 || found->local.translation.z != 0)) {
            if (node.bind.rfind("xform:", 0) == 0 || poseOf(*found).present) {
                writePose(node.attributes, poseOf(*found));
            }
        }
    }
}

void collectBinds(const EmsNode& node, std::unordered_set<std::string>& binds) {
    if (!node.bind.empty()) {
        binds.insert(node.bind);
    }
    for (const EmsNode& child : node.children) {
        collectBinds(child, binds);
    }
}

void writeAttributes(std::ostream& out, const std::vector<std::pair<std::string, std::string>>& attributes) {
    for (const auto& [key, value] : attributes) {
        out << ' ' << key << "=\"" << xmlEscape(value) << '"';
    }
}

void writeNode(std::ostream& out, EmsNode node, const Scene& scene, bool isRoot) {
    if (!node.recognized) {
        if (!node.raw.empty()) {
            out << node.raw;
        }
        return;
    }
    refreshNode(node, scene);
    out << '<' << node.name;
    writeAttributes(out, node.attributes);
    if (node.children.empty() && node.text.empty() && !isRoot) {
        out << "/>";
        return;
    }
    out << '>';
    if (!node.text.empty() && node.children.empty()) {
        out << xmlEscape(node.text);
    }
    for (const EmsNode& child : node.children) {
        writeNode(out, child, scene, false);
    }
    if (isRoot) {
        std::unordered_set<std::string> binds;
        collectBinds(node, binds);
        for (const Operation& operation : scene.operations) {
            if (binds.count("operation:" + operation.id) != 0) {
                continue;
            }
            out << "<Operation id=\"" << xmlEscape(operation.id) << "\" name=\"" << xmlEscape(operation.name) << "\" duration=\""
                << formatDouble(operation.seconds) << "\" station=\"" << xmlEscape(operation.station) << "\" path=\""
                << xmlEscape(operation.path) << "\"/>";
        }
        for (const WeldSpot& spot : scene.welds) {
            if (binds.count("weld:" + spot.id) != 0) {
                continue;
            }
            out << "<WeldPoint id=\"" << xmlEscape(spot.id) << "\" name=\"" << xmlEscape(spot.name) << "\" x=\""
                << formatDouble(spot.position.x) << "\" y=\"" << formatDouble(spot.position.y) << "\" z=\""
                << formatDouble(spot.position.z) << "\" nx=\"" << formatDouble(spot.normal.x) << "\" ny=\""
                << formatDouble(spot.normal.y) << "\" nz=\"" << formatDouble(spot.normal.z) << "\" part=\""
                << xmlEscape(spot.part) << "\"/>";
        }
        for (const RobotPath& path : scene.robotPaths) {
            if (binds.count("path:" + path.name) != 0) {
                continue;
            }
            out << "<RobotPath name=\"" << xmlEscape(path.name) << "\" resource=\"" << xmlEscape(path.resource) << "\">";
            for (const PathPoint& point : path.points) {
                out << "<Location name=\"" << xmlEscape(point.name) << "\" motion=\"" << pathMoveName(point.move) << "\" speed=\""
                    << formatDouble(point.speed) << "\"";
                if (!point.joints.empty()) {
                    out << " joints=\"" << formatList(point.joints) << "\"";
                }
                if (point.cartesian || point.joints.empty()) {
                    out << " x=\"" << formatDouble(point.position.x) << "\" y=\"" << formatDouble(point.position.y) << "\" z=\""
                        << formatDouble(point.position.z) << "\"";
                }
                out << "/>";
            }
            out << "</RobotPath>";
        }
    }
    out << "</" << node.name << '>';
}

void writeSynthesized(std::ostream& out, const Scene& scene) {
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
    out << "<eBOP name=\"" << xmlEscape(scene.ppr.title.empty() ? scene.name : scene.ppr.title) << "\">";
    for (const ProductNode& product : scene.ppr.products) {
        out << "<Part id=\"" << xmlEscape(product.id) << "\" name=\"" << xmlEscape(product.name) << "\" part=\""
            << xmlEscape(product.partName) << "\"";
        const auto node = scene.findByName(product.partName.empty() ? product.name : product.partName);
        if (node) {
            const Pose pose = poseOf(*node);
            out << " x=\"" << formatDouble(pose.x) << "\" y=\"" << formatDouble(pose.y) << "\" z=\"" << formatDouble(pose.z)
                << "\" rx=\"" << formatDouble(pose.rx) << "\" ry=\"" << formatDouble(pose.ry) << "\" rz=\"" << formatDouble(pose.rz)
                << "\"";
        }
        out << "/>";
    }
    for (const PlantNode& plant : scene.ppr.plants) {
        out << "<Resource id=\"" << xmlEscape(plant.id) << "\" name=\"" << xmlEscape(plant.name) << "\" station=\""
            << xmlEscape(plant.station) << "\" resource=\"" << xmlEscape(plant.resource) << "\"";
        const auto node = scene.findByName(plant.name);
        if (node) {
            const Pose pose = poseOf(*node);
            out << " x=\"" << formatDouble(pose.x) << "\" y=\"" << formatDouble(pose.y) << "\" z=\"" << formatDouble(pose.z)
                << "\" rx=\"" << formatDouble(pose.rx) << "\" ry=\"" << formatDouble(pose.ry) << "\" rz=\"" << formatDouble(pose.rz)
                << "\"";
        }
        out << "/>";
    }
    for (const Operation& operation : scene.operations) {
        out << "<Operation id=\"" << xmlEscape(operation.id) << "\" name=\"" << xmlEscape(operation.name) << "\" duration=\""
            << formatDouble(operation.seconds) << "\" station=\"" << xmlEscape(operation.station) << "\" path=\""
            << xmlEscape(operation.path) << "\"";
        if (!operation.after.empty()) {
            out << " after=\"" << xmlEscape(operation.after.front()) << "\"";
        }
        out << "/>";
    }
    for (const WeldSpot& spot : scene.welds) {
        out << "<WeldPoint id=\"" << xmlEscape(spot.id) << "\" name=\"" << xmlEscape(spot.name) << "\" x=\""
            << formatDouble(spot.position.x) << "\" y=\"" << formatDouble(spot.position.y) << "\" z=\"" << formatDouble(spot.position.z)
            << "\" nx=\"" << formatDouble(spot.normal.x) << "\" ny=\"" << formatDouble(spot.normal.y) << "\" nz=\""
            << formatDouble(spot.normal.z) << "\" part=\"" << xmlEscape(spot.part) << "\"/>";
    }
    for (const RobotPath& path : scene.robotPaths) {
        out << "<RobotPath name=\"" << xmlEscape(path.name) << "\" resource=\"" << xmlEscape(path.resource) << "\">";
        for (const PathPoint& point : path.points) {
            out << "<Location name=\"" << xmlEscape(point.name) << "\" motion=\"" << pathMoveName(point.move) << "\" speed=\""
                << formatDouble(point.speed) << "\"";
            if (!point.joints.empty()) {
                out << " joints=\"" << formatList(point.joints) << "\"";
            }
            if (point.cartesian || point.joints.empty()) {
                out << " x=\"" << formatDouble(point.position.x) << "\" y=\"" << formatDouble(point.position.y) << "\" z=\""
                    << formatDouble(point.position.z) << "\"";
            }
            out << "/>";
        }
        out << "</RobotPath>";
    }
    out << "</eBOP>";
}

}  // namespace

nlohmann::json emsNodeToJson(const EmsNode& node) {
    nlohmann::json attributes = nlohmann::json::array();
    for (const auto& [key, value] : node.attributes) {
        attributes.push_back({{"key", key}, {"value", value}});
    }
    nlohmann::json children = nlohmann::json::array();
    for (const EmsNode& child : node.children) {
        children.push_back(emsNodeToJson(child));
    }
    nlohmann::json json = {{"name", node.name},
                           {"attributes", std::move(attributes)},
                           {"text", node.text},
                           {"recognized", node.recognized},
                           {"bind", node.bind},
                           {"children", std::move(children)}};
    if (!node.recognized) {
        json["raw"] = node.raw;
    }
    return json;
}

void emsNodeFromJson(const nlohmann::json& json, EmsNode& node) {
    node = {};
    if (!json.is_object()) {
        return;
    }
    node.name = json.value("name", std::string{});
    node.text = json.value("text", std::string{});
    node.raw = json.value("raw", std::string{});
    node.recognized = json.value("recognized", false);
    node.bind = json.value("bind", std::string{});
    if (json.contains("attributes") && json.at("attributes").is_array()) {
        for (const auto& item : json.at("attributes")) {
            node.attributes.emplace_back(item.value("key", std::string{}), item.value("value", std::string{}));
        }
    }
    if (json.contains("children") && json.at("children").is_array()) {
        for (const auto& child : json.at("children")) {
            EmsNode parsed;
            emsNodeFromJson(child, parsed);
            if (!parsed.name.empty() || !parsed.raw.empty()) {
                node.children.push_back(std::move(parsed));
            }
        }
    }
}

EmsImportResult importEmsXml(const std::string& path, Scene& scene) {
    EmsImportResult result;
    std::ifstream in(path);
    if (!in) {
        result.message = "cannot open " + path;
        return result;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    XmlNode root;
    std::string error;
    if (!parseXml(buffer.str(), root, error)) {
        result.message = error.empty() ? "xml was not read" : error;
        return result;
    }
    int products = 0;
    int processes = 0;
    int plants = 0;
    int welds = 0;
    int paths = 0;
    int nextId = 1;
    EmsNode tree = convertNode(root, {}, scene, products, processes, plants, welds, paths, nextId);
    if (products + processes + plants + welds + paths == 0) {
        result.message = "xml has no product, process, plant, weld, or path elements (root " + root.name + ")";
        return result;
    }
    if (scene.ppr.title.empty()) {
        const std::string named = attrOf(root, {"name", "Name"});
        scene.ppr.title = named.empty() ? (root.name.empty() ? std::string("PPR") : root.name) : named;
    }
    scene.ppr.ems = std::move(tree);
    scene.ppr.hasEms = true;
    result.ok = true;
    result.message = "read " + std::to_string(products) + " products, " + std::to_string(processes) + " processes, " +
                     std::to_string(plants) + " plants, " + std::to_string(welds) + " welds, " + std::to_string(paths) + " paths";
    return result;
}

EmsImportResult exportEmsXml(const Scene& scene, const std::string& path) {
    EmsImportResult result;
    std::ofstream out(path);
    if (!out) {
        result.message = "cannot write " + path;
        return result;
    }
    if (scene.ppr.hasEms && !scene.ppr.ems.name.empty()) {
        writeNode(out, scene.ppr.ems, scene, true);
    } else {
        writeSynthesized(out, scene);
    }
    if (!out) {
        result.message = "cannot write " + path;
        return result;
    }
    result.ok = true;
    result.message = "wrote " + path;
    return result;
}

}  // namespace sim
