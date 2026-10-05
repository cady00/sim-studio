#include "sim/motion/Motion.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

namespace sim {
namespace {

const KinLink* linkByName(const KinematicResource& resource, const std::string& name) {
    for (const KinLink& link : resource.links) {
        if (link.name == name) {
            return &link;
        }
    }
    return nullptr;
}

}  // namespace

void applyResource(Scene& scene, const KinematicResource& resource) {
    for (const KinJoint& joint : resource.joints) {
        const KinLink* child = linkByName(resource, joint.childLink);
        if (child == nullptr) {
            continue;
        }
        auto node = scene.find(child->nodeId);
        if (!node) {
            continue;
        }
        const double position = jointCommand(resource, joint);
        const Vec3 axis = normalize(joint.axis);
        if (joint.motion == JointMotion::Prismatic) {
            node->local.rotation = joint.restLocal.rotation;
            node->local.translation = joint.restLocal.translation + axis * position;
            node->local.scale = joint.restLocal.scale;
        } else {
            const Quat delta = Quat::fromAxisAngle(axis, deg2rad(position));
            node->local.rotation = (delta * joint.restLocal.rotation).normalized();
            const Vec3 offset = joint.restLocal.translation - joint.anchor;
            node->local.translation = joint.anchor + delta.rotate(offset);
            node->local.scale = joint.restLocal.scale;
        }
    }
}

std::vector<double> jointVector(const KinematicResource& resource) {
    std::vector<double> values;
    values.reserve(resource.joints.size());
    for (const KinJoint& joint : resource.joints) {
        values.push_back(jointCommand(resource, joint));
    }
    return values;
}

void setJointVector(KinematicResource& resource, const std::vector<double>& values) {
    const std::size_t count = std::min(values.size(), resource.joints.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (!resource.joints[i].coupledTo.empty()) {
            continue;
        }
        resource.joints[i].position = clampJoint(resource.joints[i], values[i]);
    }
}

KinematicResource& ensureResource(Scene& scene, const std::string& name) {
    if (KinematicResource* existing = findResource(scene.resources, name)) {
        return *existing;
    }
    KinematicResource created;
    created.name = name;
    scene.resources.push_back(std::move(created));
    return scene.resources.back();
}

KinLink& addLink(Scene& scene, KinematicResource& resource, const std::string& name, const std::string& parentLink,
                 const Transform& local) {
    auto node = scene.makeNode(NodeKind::Assembly, name);
    node->local = local;
    std::shared_ptr<SceneNode> parent = scene.root();
    if (!parentLink.empty()) {
        if (KinLink* link = findLink(resource, parentLink)) {
            if (auto found = scene.find(link->nodeId)) {
                parent = found;
            }
        }
    }
    scene.addChild(parent, node);
    KinLink link;
    link.name = name;
    link.nodeId = node->id;
    resource.links.push_back(std::move(link));
    return resource.links.back();
}

bool adoptParts(Scene& scene, KinematicResource& resource, const std::string& linkName,
                const std::vector<std::string>& partIds, std::string& error) {
    KinLink* link = findLink(resource, linkName);
    if (link == nullptr) {
        error = "no link named " + linkName;
        return false;
    }
    for (const std::string& id : partIds) {
        if (!scene.reparent(id, link->nodeId, true)) {
            error = "could not group " + id;
            return false;
        }
    }
    error.clear();
    return true;
}

KinJoint& addJoint(KinematicResource& resource, Scene& scene, const std::string& name, const std::string& parentLink,
                   const std::string& childLink, JointMotion motion, Vec3 axis, Vec3 anchor) {
    KinJoint joint;
    joint.name = name;
    joint.parentLink = parentLink;
    joint.childLink = childLink;
    joint.motion = motion;
    joint.axis = normalize(axis);
    if (length(joint.axis) < 1e-12) {
        joint.axis = {0, 0, 1};
    }
    joint.anchor = anchor;
    if (motion == JointMotion::Prismatic) {
        joint.lower = 0;
        joint.upper = 500;
        joint.maxSpeed = 200;
        joint.maxAccel = 1500;
    }
    if (const KinLink* child = linkByName(resource, childLink)) {
        if (auto node = scene.find(child->nodeId)) {
            joint.restLocal = node->local;
        }
    }
    resource.joints.push_back(std::move(joint));
    return resource.joints.back();
}

void setAxisFromPoints(KinJoint& joint, Vec3 start, Vec3 end) {
    const Vec3 axis = end - start;
    if (length(axis) < 1e-9) {
        return;
    }
    joint.axis = normalize(axis);
    joint.anchor = start;
}

void capturePose(KinematicResource& resource, const std::string& poseName) {
    KinPose pose;
    pose.name = poseName;
    for (const KinJoint& joint : resource.joints) {
        if (!joint.coupledTo.empty()) {
            continue;
        }
        pose.joints.emplace_back(joint.name, joint.position);
    }
    for (KinPose& existing : resource.poses) {
        if (existing.name == poseName) {
            existing = std::move(pose);
            return;
        }
    }
    resource.poses.push_back(std::move(pose));
}

bool jogJoint(KinematicResource& resource, const std::string& jointName, double delta, std::string& error) {
    KinJoint* joint = findJoint(resource, jointName);
    if (joint == nullptr) {
        error = "no joint named " + jointName;
        return false;
    }
    if (!joint->coupledTo.empty()) {
        error = jointName + " is coupled to " + joint->coupledTo;
        return false;
    }
    joint->position = clampJoint(*joint, joint->position + delta);
    error.clear();
    return true;
}

bool beginJointMove(const KinematicResource& resource, const std::vector<double>& target, ProfileKind profile,
                    ActiveMove& move, std::string& error) {
    move = {};
    move.profile = profile;
    for (std::size_t i = 0; i < resource.joints.size() && i < target.size(); ++i) {
        const KinJoint& joint = resource.joints[i];
        if (!joint.coupledTo.empty()) {
            continue;
        }
        const double goal = clampJoint(joint, target[i]);
        move.index.push_back(static_cast<int>(i));
        move.q0.push_back(joint.position);
        move.q1.push_back(goal);
        const JointAccelLimit limit = jointAccelLimit(resource, joint);
        move.vmax.push_back(std::max(joint.maxSpeed, 1e-6));
        move.amax.push_back(std::max(limit.value, 1e-6));
        move.jointDuration.push_back(profileDuration(profile, joint.position, goal, joint.maxSpeed, limit.value));
        move.duration = std::max(move.duration, move.jointDuration.back());
    }
    move.active = true;
    error.clear();
    return true;
}

bool beginPoseMove(const KinematicResource& resource, const std::string& poseName, ProfileKind profile, ActiveMove& move,
                   std::string& error) {
    const KinPose* pose = findPose(resource, poseName);
    if (pose == nullptr) {
        error = "no pose named " + poseName;
        return false;
    }
    std::vector<double> target = jointVector(resource);
    for (const auto& [name, value] : pose->joints) {
        for (std::size_t i = 0; i < resource.joints.size(); ++i) {
            if (resource.joints[i].name == name) {
                if (i < target.size()) {
                    target[i] = value;
                }
            }
        }
    }
    return beginJointMove(resource, target, profile, move, error);
}

bool advanceMove(KinematicResource& resource, ActiveMove& move, double dt) {
    if (!move.active) {
        return false;
    }
    move.time += std::max(dt, 0.0);
    bool allDone = true;
    for (std::size_t i = 0; i < move.index.size(); ++i) {
        const double duration = move.jointDuration[i];
        const double local = move.duration <= 1e-12 ? duration : move.time * (duration / move.duration);
        bool done = false;
        const double value = profileSample(move.profile, move.q0[i], move.q1[i], move.vmax[i], move.amax[i], local, &done);
        KinJoint& joint = resource.joints[static_cast<std::size_t>(move.index[i])];
        joint.position = value;
        allDone = allDone && done;
    }
    if (move.time >= move.duration - 1e-9) {
        allDone = true;
    }
    if (allDone) {
        for (std::size_t i = 0; i < move.index.size(); ++i) {
            resource.joints[static_cast<std::size_t>(move.index[i])].position = move.q1[i];
        }
        move.active = false;
    }
    return move.active;
}

bool attachTool(Scene& scene, KinematicResource& resource, const std::string& nodeId, std::string& error) {
    if (resource.links.empty()) {
        error = "resource has no flange link";
        return false;
    }
    const KinLink& flange = resource.links.back();
    if (!scene.reparent(nodeId, flange.nodeId, true)) {
        error = "could not attach " + nodeId;
        return false;
    }
    error.clear();
    return true;
}

bool detachTool(Scene& scene, const std::string& nodeId, std::string& error) {
    if (!scene.root() || !scene.reparent(nodeId, scene.root()->id, true)) {
        error = "could not detach " + nodeId;
        return false;
    }
    error.clear();
    return true;
}

bool saveLibrary(const Scene& scene, const std::string& resourceName, const std::string& path, std::string& error) {
    const KinematicResource* resource = findResource(scene.resources, resourceName);
    if (resource == nullptr) {
        error = "no resource named " + resourceName;
        return false;
    }
    nlohmann::json json;
    json["format"] = "simstudio.library";
    json["version"] = 1;
    json["resource"] = kinematicsToJson(*resource);
    json["logic"] = nlohmann::json::array();
    for (const LogicBlock& block : scene.logicBlocks) {
        if (block.resource == resourceName || block.resource.empty()) {
            json["logic"].push_back(logicBlockToJson(block));
        }
    }
    json["paths"] = nlohmann::json::array();
    for (const RobotPath& robotPath : scene.robotPaths) {
        if (robotPath.resource == resourceName) {
            json["paths"].push_back(robotPathToJson(robotPath));
        }
    }
    std::ofstream out(path);
    if (!out) {
        error = "could not write " + path;
        return false;
    }
    out << json.dump(2);
    error.clear();
    return true;
}

bool loadLibrary(Scene& scene, const std::string& path, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "could not read " + path;
        return false;
    }
    nlohmann::json json;
    try {
        in >> json;
    } catch (const std::exception& ex) {
        error = ex.what();
        return false;
    }
    if (json.value("format", "") != "simstudio.library") {
        error = "not an Ondot Sim library";
        return false;
    }
    KinematicResource resource;
    if (!kinematicsFromJson(json.at("resource"), resource, error)) {
        return false;
    }
    if (findResource(scene.resources, resource.name) != nullptr) {
        resource.name += "_lib";
    }
    scene.resources.push_back(std::move(resource));
    if (json.contains("logic")) {
        for (const auto& item : json.at("logic")) {
            LogicBlock block;
            if (!logicBlockFromJson(item, block, error)) {
                return false;
            }
            block.resource = scene.resources.back().name;
            scene.logicBlocks.push_back(std::move(block));
        }
    }
    if (json.contains("paths")) {
        for (const auto& item : json.at("paths")) {
            RobotPath robotPath;
            if (!robotPathFromJson(item, robotPath, error)) {
                return false;
            }
            robotPath.resource = scene.resources.back().name;
            scene.robotPaths.push_back(std::move(robotPath));
        }
    }
    error.clear();
    return true;
}

}  // namespace sim
