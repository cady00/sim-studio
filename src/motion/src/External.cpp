#include "sim/motion/Motion.hpp"

#include "sim/core/Mesh.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace sim {
namespace {

std::shared_ptr<SceneNode> addBox(Scene& scene, const std::shared_ptr<SceneNode>& parent, const std::string& name,
                                  Vec3 halfExtents, Vec3 color, Vec3 localTranslation) {
    auto node = scene.makeNode(NodeKind::Part, name);
    node->meshId = scene.addMesh(makeBoxMesh(halfExtents));
    node->color = color;
    node->local.translation = localTranslation;
    scene.addChild(parent, node);
    return node;
}

int externalSlot(const std::string& name) {
    if (name.size() == 2 && name[0] == 'E' && name[1] >= '1' && name[1] <= '6') {
        return name[1] - '1';
    }
    return -1;
}

bool nameTaken(const KinematicResource& robot, const std::string& name) {
    for (const KinLink& link : robot.links) {
        if (link.name == name) {
            return true;
        }
    }
    return findJoint(robot, name) != nullptr;
}

std::string prefixed(const KinematicResource& robot, const std::string& carrier, const std::string& name) {
    if (!nameTaken(robot, name)) {
        return name;
    }
    return carrier + "_" + name;
}

}  // namespace

std::vector<int> externalJointIndices(const KinematicResource& resource) {
    std::vector<int> indices;
    for (int i = 0; i < static_cast<int>(resource.joints.size()); ++i) {
        if (!resource.joints[static_cast<std::size_t>(i)].externalName.empty()) {
            indices.push_back(i);
        }
    }
    return indices;
}

std::vector<OlpAxisName> externalExportMap(const KinematicResource& resource) {
    static const char* rapidNames[] = {"eax_a", "eax_b", "eax_c", "eax_d", "eax_e", "eax_f"};
    std::vector<OlpAxisName> rows;
    int fallback = 0;
    for (const KinJoint& joint : resource.joints) {
        if (joint.externalName.empty()) {
            continue;
        }
        int slot = externalSlot(joint.externalName);
        if (slot < 0) {
            slot = fallback;
        }
        if (slot < 0 || slot > 5) {
            continue;
        }
        ++fallback;
        OlpAxisName row;
        row.joint = joint.name;
        row.e = joint.externalName;
        row.rapid = rapidNames[slot];
        row.fanuc = "GP2:J" + std::to_string(slot + 1);
        rows.push_back(std::move(row));
    }
    return rows;
}

bool attachExternalAxes(Scene& scene, KinematicResource& robot, const std::string& carrierName,
                        const std::string& mountLink, const std::string& kind, std::string& error) {
    if (kind != "track" && kind != "positioner") {
        error = "external kind must be track or positioner";
        return false;
    }
    KinematicResource* carrier = findResource(scene.resources, carrierName);
    if (carrier == nullptr) {
        error = "no resource named " + carrierName;
        return false;
    }
    if (findLink(*carrier, mountLink) == nullptr) {
        error = "no link named " + mountLink;
        return false;
    }
    std::vector<std::string> originalLinks;
    originalLinks.reserve(robot.links.size());
    for (const KinLink& link : robot.links) {
        originalLinks.push_back(link.name);
    }
    std::unordered_map<std::string, std::string> linkRename;
    for (const KinLink& link : carrier->links) {
        KinLink copy = link;
        copy.name = prefixed(robot, carrierName, link.name);
        linkRename[link.name] = copy.name;
        if (!copy.mountedOn.empty()) {
            const auto renamed = linkRename.find(copy.mountedOn);
            if (renamed != linkRename.end()) {
                copy.mountedOn = renamed->second;
            }
        }
        robot.links.push_back(std::move(copy));
    }
    int nextExternal = 1;
    for (const KinJoint& joint : robot.joints) {
        const int slot = externalSlot(joint.externalName);
        if (slot >= 0) {
            nextExternal = std::max(nextExternal, slot + 2);
        }
    }
    for (const KinJoint& joint : carrier->joints) {
        if (nextExternal > 6) {
            error = "only E1 through E6 can be attached";
            return false;
        }
        KinJoint copy = joint;
        copy.name = prefixed(robot, carrierName, joint.name);
        const auto parent = linkRename.find(copy.parentLink);
        const auto child = linkRename.find(copy.childLink);
        if (parent != linkRename.end()) {
            copy.parentLink = parent->second;
        }
        if (child != linkRename.end()) {
            copy.childLink = child->second;
        }
        copy.externalName = "E" + std::to_string(nextExternal);
        copy.externalKind = kind;
        ++nextExternal;
        robot.joints.push_back(std::move(copy));
    }
    for (const NamedFrame& frame : carrier->frames) {
        bool exists = false;
        for (const NamedFrame& existing : robot.frames) {
            if (existing.name == frame.name) {
                exists = true;
                break;
            }
        }
        if (exists) {
            continue;
        }
        NamedFrame copy = frame;
        const auto renamed = linkRename.find(copy.link);
        if (renamed != linkRename.end()) {
            copy.link = renamed->second;
        }
        robot.frames.push_back(std::move(copy));
    }
    std::unordered_set<std::string> children;
    for (const KinJoint& joint : robot.joints) {
        children.insert(joint.childLink);
    }
    std::string robotRoot;
    for (const std::string& name : originalLinks) {
        if (children.count(name) == 0) {
            robotRoot = name;
            break;
        }
    }
    if (robotRoot.empty()) {
        error = "robot has no root link";
        return false;
    }
    KinLink* root = findLink(robot, robotRoot);
    const auto mountName = linkRename.find(mountLink);
    KinLink* mount = mountName == linkRename.end() ? nullptr : findLink(robot, mountName->second);
    if (root == nullptr || mount == nullptr) {
        error = "could not mount the robot";
        return false;
    }
    if (!scene.reparent(root->nodeId, mount->nodeId, true)) {
        error = "could not mount the robot";
        return false;
    }
    if (const auto node = scene.find(root->nodeId)) {
        root->mountedOn = mount->name;
        root->mountRest = node->local;
    }
    error.clear();
    return true;
}

ReachReport analyzeReach(const KinematicResource& resource, const CartesianPose& target, int samples) {
    ReachReport report;
    const std::vector<int> external = externalJointIndices(resource);
    int track = -1;
    int slot = 0;
    for (std::size_t i = 0; i < external.size(); ++i) {
        const KinJoint& joint = resource.joints[static_cast<std::size_t>(external[i])];
        if (joint.externalKind == "track") {
            track = external[i];
            slot = static_cast<int>(i);
            break;
        }
    }
    if (track < 0) {
        return report;
    }
    samples = std::max(samples, 2);
    const KinJoint& joint = resource.joints[static_cast<std::size_t>(track)];
    report.joint = joint.externalName.empty() ? joint.name : joint.externalName;
    report.lower = std::min(joint.lower, joint.upper);
    report.upper = std::max(joint.lower, joint.upper);
    report.samples = samples;
    const std::vector<double> seed = jointVector(resource);
    for (int i = 0; i < samples; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(samples - 1);
        const double value = report.lower + (report.upper - report.lower) * u;
        KinematicResource trial = resource;
        trial.redundancy.mode = "fixed";
        trial.redundancy.fixed.assign(external.size(), 0);
        for (std::size_t k = 0; k < external.size(); ++k) {
            const int index = external[k];
            trial.redundancy.fixed[k] = index < static_cast<int>(seed.size()) ? seed[static_cast<std::size_t>(index)] : 0;
        }
        trial.redundancy.fixed[static_cast<std::size_t>(slot)] = value;
        const IkResult solved = inverseModel(trial, target, seed);
        if (solved.ok) {
            ++report.reachable;
        }
    }
    return report;
}

std::vector<std::shared_ptr<SceneNode>> hideAllMeshedParts(Scene& scene) {
    std::vector<std::shared_ptr<SceneNode>> hidden;
    if (!scene.root()) {
        return hidden;
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
        if (node->kind == NodeKind::Part && !node->meshId.empty() && node->visible) {
            node->visible = false;
            hidden.push_back(node);
        }
    }
    return hidden;
}

bool addKr420TrackDemonstration(Scene& scene, std::string& error) {
    if (findResource(scene.resources, "kr420") == nullptr) {
        error = "no kr420 resource";
        return false;
    }
    // ensureResource may reallocate the resource vector, so look the robot up afterwards.
    KinematicResource& carrier = ensureResource(scene, "kl");
    KinematicResource* robot = findResource(scene.resources, "kr420");
    if (robot == nullptr || robot->links.empty()) {
        error = "no kr420 resource";
        return false;
    }
    if (carrier.links.empty()) {
        const std::string railId = addLink(scene, carrier, "rail", "", {}).nodeId;
        const std::string carriageId = addLink(scene, carrier, "carriage", "rail", {}).nodeId;
        KinJoint& slide = addJoint(carrier, scene, "slide", "rail", "carriage", JointMotion::Prismatic, {1, 0, 0}, {});
        slide.lower = 0;
        slide.upper = 10000;
        // Placeholders. Public KL datasheets were not used for these two numbers.
        slide.maxSpeed = 1500;
        slide.maxAccel = 1500;
        slide.position = 0;
        if (const auto rail = scene.find(railId)) {
            addBox(scene, rail, "Rail", {5000, 140, 40}, {0.35, 0.38, 0.42}, {5000, 0, 40});
        }
        if (const auto carriageNode = scene.find(carriageId)) {
            addBox(scene, carriageNode, "Carriage", {220, 180, 50}, {0.75, 0.55, 0.22}, {0, 0, 130});
        }
    }
    if (!attachExternalAxes(scene, *robot, "kl", "carriage", "track", error)) {
        return false;
    }
    const CartesianPose home = forwardModel(*robot);
    RobotPath path;
    path.name = "long";
    path.resource = robot->name;
    path.external = "sync";
    PathPoint far;
    far.name = "far";
    far.move = PathMove::Lin;
    far.cartesian = true;
    far.position = home.position + Vec3{8000, 0, 0};
    far.orientation = home.rotation;
    far.speed = 5000;
    path.points.push_back(far);
    bool replaced = false;
    for (RobotPath& existing : scene.robotPaths) {
        if (existing.name == path.name) {
            existing = path;
            replaced = true;
            break;
        }
    }
    if (!replaced) {
        scene.robotPaths.push_back(std::move(path));
    }
    error.clear();
    return true;
}

}  // namespace sim
