#include "sim/core/Kinematics.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>

namespace sim {
namespace {

nlohmann::json vec3ToJson(Vec3 v) { return {v.x, v.y, v.z}; }

Vec3 vec3FromJson(const nlohmann::json& json) {
    return {json.at(0).get<double>(), json.at(1).get<double>(), json.at(2).get<double>()};
}

nlohmann::json quatToJson(Quat q) { return {q.w, q.x, q.y, q.z}; }

Quat quatFromJson(const nlohmann::json& json) {
    return Quat{json.at(0).get<double>(), json.at(1).get<double>(), json.at(2).get<double>(), json.at(3).get<double>()}
        .normalized();
}

nlohmann::json transformToJson(const Transform& t) {
    return {{"translation", vec3ToJson(t.translation)},
            {"rotation", quatToJson(t.rotation)},
            {"scale", vec3ToJson(t.scale)}};
}

Transform transformFromJson(const nlohmann::json& json) {
    Transform t;
    t.translation = vec3FromJson(json.at("translation"));
    t.rotation = quatFromJson(json.at("rotation"));
    if (json.contains("scale")) {
        t.scale = vec3FromJson(json.at("scale"));
    }
    return t;
}

}  // namespace

const char* jointMotionName(JointMotion motion) {
    return motion == JointMotion::Prismatic ? "prismatic" : "revolute";
}

bool jointMotionFromName(const std::string& name, JointMotion& out) {
    if (name == "revolute") {
        out = JointMotion::Revolute;
        return true;
    }
    if (name == "prismatic") {
        out = JointMotion::Prismatic;
        return true;
    }
    return false;
}

const char* pathMoveName(PathMove move) {
    if (move == PathMove::Lin) {
        return "lin";
    }
    if (move == PathMove::Circ) {
        return "circ";
    }
    return "ptp";
}

bool pathMoveFromName(const std::string& name, PathMove& out) {
    if (name == "ptp") {
        out = PathMove::Ptp;
        return true;
    }
    if (name == "lin") {
        out = PathMove::Lin;
        return true;
    }
    if (name == "circ") {
        out = PathMove::Circ;
        return true;
    }
    return false;
}

double clampJoint(const KinJoint& joint, double position) {
    if (!joint.limited) {
        return position;
    }
    return std::clamp(position, std::min(joint.lower, joint.upper), std::max(joint.lower, joint.upper));
}

KinematicResource* findResource(std::vector<KinematicResource>& resources, const std::string& name) {
    for (KinematicResource& resource : resources) {
        if (resource.name == name) {
            return &resource;
        }
    }
    return nullptr;
}

const KinematicResource* findResource(const std::vector<KinematicResource>& resources, const std::string& name) {
    for (const KinematicResource& resource : resources) {
        if (resource.name == name) {
            return &resource;
        }
    }
    return nullptr;
}

KinJoint* findJoint(KinematicResource& resource, const std::string& name) {
    for (KinJoint& joint : resource.joints) {
        if (joint.name == name) {
            return &joint;
        }
    }
    return nullptr;
}

const KinJoint* findJoint(const KinematicResource& resource, const std::string& name) {
    for (const KinJoint& joint : resource.joints) {
        if (joint.name == name) {
            return &joint;
        }
    }
    return nullptr;
}

KinLink* findLink(KinematicResource& resource, const std::string& name) {
    for (KinLink& link : resource.links) {
        if (link.name == name) {
            return &link;
        }
    }
    return nullptr;
}

const KinPose* findPose(const KinematicResource& resource, const std::string& name) {
    for (const KinPose& pose : resource.poses) {
        if (pose.name == name) {
            return &pose;
        }
    }
    return nullptr;
}

nlohmann::json kinematicsToJson(const KinematicResource& resource) {
    nlohmann::json json;
    json["name"] = resource.name;
    json["ik"] = resource.ik;
    json["wrist"] = {{"d1", resource.wrist.d1},
                     {"shoulder", resource.wrist.shoulder},
                     {"a2", resource.wrist.a2},
                     {"a3", resource.wrist.a3},
                     {"d6", resource.wrist.d6}, {"tcp", vec3ToJson(resource.wrist.tcp)},
                     {"tcpRotation", quatToJson(resource.wrist.tcpRotation)}};
    json["scara"] = {{"l1", resource.scara.l1}, {"l2", resource.scara.l2}, {"baseZ", resource.scara.baseZ},
                     {"tcp", vec3ToJson(resource.scara.tcp)}};
    json["links"] = nlohmann::json::array();
    for (const KinLink& link : resource.links) {
        nlohmann::json item = {{"name", link.name},
                               {"node", link.nodeId},
                               {"mountedOn", link.mountedOn},
                               {"mount", transformToJson(link.mountRest)}};
        if (link.hasInertia) {
            item["mass"] = link.mass;
            item["com"] = vec3ToJson(link.com);
            item["inertia"] = {link.ixx, link.iyy, link.izz, link.ixy, link.ixz, link.iyz};
        }
        json["links"].push_back(std::move(item));
    }
    json["joints"] = nlohmann::json::array();
    for (const KinJoint& joint : resource.joints) {
        json["joints"].push_back({{"name", joint.name},
                                  {"parent", joint.parentLink},
                                  {"child", joint.childLink},
                                  {"motion", jointMotionName(joint.motion)},
                                  {"anchor", vec3ToJson(joint.anchor)},
                                  {"axis", vec3ToJson(joint.axis)},
                                  {"limited", joint.limited},
                                  {"lower", joint.lower},
                                  {"upper", joint.upper},
                                  {"maxSpeed", joint.maxSpeed},
                                  {"maxAccel", joint.maxAccel},
                                  {"coupledTo", joint.coupledTo},
                                  {"coupleRatio", joint.coupleRatio},
                                  {"position", joint.position},
                                  {"rest", transformToJson(joint.restLocal)},
                                  {"external", joint.externalName},
                                  {"externalKind", joint.externalKind}});
    }
    json["poses"] = nlohmann::json::array();
    for (const KinPose& pose : resource.poses) {
        nlohmann::json joints = nlohmann::json::array();
        for (const auto& [name, value] : pose.joints) {
            joints.push_back({{"name", name}, {"value", value}});
        }
        json["poses"].push_back({{"name", pose.name}, {"joints", std::move(joints)}});
    }
    json["frames"] = nlohmann::json::array();
    for (const NamedFrame& frame : resource.frames) {
        json["frames"].push_back({{"name", frame.name},
                                  {"origin", vec3ToJson(frame.origin)},
                                  {"rotation", quatToJson(frame.rotation)},
                                  {"link", frame.link}});
    }
    json["redundancy"] = {{"mode", resource.redundancy.mode},
                          {"travel", resource.redundancy.travel},
                          {"midrange", resource.redundancy.midrange},
                          {"manipulability", resource.redundancy.manipulability},
                          {"fixed", resource.redundancy.fixed}};
    return json;
}

bool kinematicsFromJson(const nlohmann::json& json, KinematicResource& resource, std::string& error) {
    if (!json.is_object() || !json.contains("name")) {
        error = "kinematic resource is missing a name";
        return false;
    }
    resource = {};
    resource.name = json.at("name").get<std::string>();
    resource.ik = json.value("ik", std::string("none"));
    if (json.contains("wrist")) {
        const auto& wrist = json.at("wrist");
        resource.wrist.d1 = wrist.value("d1", resource.wrist.d1);
        resource.wrist.shoulder = wrist.value("shoulder", resource.wrist.shoulder);
        resource.wrist.a2 = wrist.value("a2", resource.wrist.a2);
        resource.wrist.a3 = wrist.value("a3", resource.wrist.a3);
        resource.wrist.d6 = wrist.value("d6", resource.wrist.d6);
        if (wrist.contains("tcp")) {
            resource.wrist.tcp = vec3FromJson(wrist.at("tcp"));
        }
        if (wrist.contains("tcpRotation")) {
            resource.wrist.tcpRotation = quatFromJson(wrist.at("tcpRotation"));
        }
    }
    if (json.contains("scara")) {
        const auto& scara = json.at("scara");
        resource.scara.l1 = scara.value("l1", resource.scara.l1);
        resource.scara.l2 = scara.value("l2", resource.scara.l2);
        resource.scara.baseZ = scara.value("baseZ", resource.scara.baseZ);
        if (scara.contains("tcp")) {
            resource.scara.tcp = vec3FromJson(scara.at("tcp"));
        }
    }
    if (json.contains("links")) {
        for (const auto& item : json.at("links")) {
            KinLink link;
            link.name = item.value("name", std::string{});
            link.nodeId = item.value("node", std::string{});
            link.mountedOn = item.value("mountedOn", std::string{});
            if (item.contains("mount")) {
                link.mountRest = transformFromJson(item.at("mount"));
            }
            if (item.contains("mass")) {
                link.hasInertia = true;
                link.mass = item.value("mass", 0.0);
                if (item.contains("com")) {
                    link.com = vec3FromJson(item.at("com"));
                }
                if (item.contains("inertia") && item.at("inertia").is_array() && item.at("inertia").size() >= 6) {
                    link.ixx = item.at("inertia")[0].get<double>();
                    link.iyy = item.at("inertia")[1].get<double>();
                    link.izz = item.at("inertia")[2].get<double>();
                    link.ixy = item.at("inertia")[3].get<double>();
                    link.ixz = item.at("inertia")[4].get<double>();
                    link.iyz = item.at("inertia")[5].get<double>();
                }
            }
            if (!link.name.empty()) {
                resource.links.push_back(std::move(link));
            }
        }
    }
    if (json.contains("joints")) {
        for (const auto& item : json.at("joints")) {
            KinJoint joint;
            joint.name = item.value("name", std::string{});
            joint.parentLink = item.value("parent", std::string{});
            joint.childLink = item.value("child", std::string{});
            JointMotion motion = JointMotion::Revolute;
            if (item.contains("motion") && !jointMotionFromName(item.at("motion").get<std::string>(), motion)) {
                error = "unknown joint motion";
                return false;
            }
            joint.motion = motion;
            if (item.contains("anchor")) {
                joint.anchor = vec3FromJson(item.at("anchor"));
            }
            if (item.contains("axis")) {
                joint.axis = vec3FromJson(item.at("axis"));
            }
            joint.limited = item.value("limited", true);
            joint.lower = item.value("lower", 0.0);
            joint.upper = item.value("upper", 0.0);
            joint.maxSpeed = item.value("maxSpeed", 90.0);
            joint.maxAccel = item.value("maxAccel", 360.0);
            joint.coupledTo = item.value("coupledTo", std::string{});
            joint.coupleRatio = item.value("coupleRatio", 1.0);
            joint.position = item.value("position", 0.0);
            if (item.contains("rest")) {
                joint.restLocal = transformFromJson(item.at("rest"));
            }
            joint.externalName = item.value("external", std::string{});
            joint.externalKind = item.value("externalKind", std::string{});
            if (!joint.name.empty()) {
                resource.joints.push_back(std::move(joint));
            }
        }
    }
    if (json.contains("poses")) {
        for (const auto& item : json.at("poses")) {
            KinPose pose;
            pose.name = item.value("name", std::string{});
            if (item.contains("joints")) {
                for (const auto& joint : item.at("joints")) {
                    pose.joints.emplace_back(joint.value("name", std::string{}), joint.value("value", 0.0));
                }
            }
            if (!pose.name.empty()) {
                resource.poses.push_back(std::move(pose));
            }
        }
    }
    if (json.contains("frames")) {
        for (const auto& item : json.at("frames")) {
            NamedFrame frame;
            frame.name = item.value("name", std::string{});
            if (item.contains("origin")) {
                frame.origin = vec3FromJson(item.at("origin"));
            }
            if (item.contains("rotation")) {
                frame.rotation = quatFromJson(item.at("rotation"));
            }
            frame.link = item.value("link", std::string{});
            if (!frame.name.empty()) {
                resource.frames.push_back(std::move(frame));
            }
        }
    }
    if (json.contains("redundancy")) {
        const auto& redundancy = json.at("redundancy");
        resource.redundancy.mode = redundancy.value("mode", std::string("fixed"));
        resource.redundancy.travel = redundancy.value("travel", resource.redundancy.travel);
        resource.redundancy.midrange = redundancy.value("midrange", resource.redundancy.midrange);
        resource.redundancy.manipulability = redundancy.value("manipulability", resource.redundancy.manipulability);
        if (redundancy.contains("fixed")) {
            for (const auto& value : redundancy.at("fixed")) {
                resource.redundancy.fixed.push_back(value.get<double>());
            }
        }
    }
    error.clear();
    return true;
}

nlohmann::json logicBlockToJson(const LogicBlock& block) {
    nlohmann::json json;
    json["name"] = block.name;
    json["resource"] = block.resource;
    json["runSignal"] = block.runSignal;
    json["inputs"] = block.inputs;
    json["outputs"] = block.outputs;
    json["python"] = block.python;
    json["parameters"] = nlohmann::json::array();
    for (const LogicParam& param : block.parameters) {
        json["parameters"].push_back({{"name", param.name},
                                      {"type", param.type},
                                      {"flag", param.flag},
                                      {"integer", param.integer},
                                      {"real", param.real}});
    }
    json["steps"] = nlohmann::json::array();
    for (const LogicStep& step : block.steps) {
        nlohmann::json when = nlohmann::json::array();
        for (const LogicCondition& condition : step.when) {
            when.push_back({{"signal", condition.signal},
                            {"op", condition.op},
                            {"asBool", condition.asBool},
                            {"flag", condition.flag},
                            {"number", condition.number}});
        }
        nlohmann::json actions = nlohmann::json::array();
        for (const LogicAction& action : step.actions) {
            actions.push_back({{"kind", action.kind},
                               {"resource", action.resource},
                               {"pose", action.pose},
                               {"joint", action.joint},
                               {"signal", action.signal},
                               {"profile", action.profile},
                               {"speed", action.speed},
                               {"seconds", action.seconds}});
        }
        json["steps"].push_back({{"when", std::move(when)}, {"actions", std::move(actions)}});
    }
    return json;
}

bool logicBlockFromJson(const nlohmann::json& json, LogicBlock& block, std::string& error) {
    if (!json.is_object() || !json.contains("name")) {
        error = "logic block is missing a name";
        return false;
    }
    block = {};
    block.name = json.at("name").get<std::string>();
    block.resource = json.value("resource", std::string{});
    block.runSignal = json.value("runSignal", std::string{});
    block.python = json.value("python", std::string{});
    if (json.contains("inputs")) {
        for (const auto& item : json.at("inputs")) {
            block.inputs.push_back(item.get<std::string>());
        }
    }
    if (json.contains("outputs")) {
        for (const auto& item : json.at("outputs")) {
            block.outputs.push_back(item.get<std::string>());
        }
    }
    if (json.contains("parameters")) {
        for (const auto& item : json.at("parameters")) {
            LogicParam param;
            param.name = item.value("name", std::string{});
            param.type = item.value("type", std::string("real"));
            param.flag = item.value("flag", false);
            param.integer = item.value("integer", 0);
            param.real = item.value("real", 0.0);
            if (!param.name.empty()) {
                block.parameters.push_back(std::move(param));
            }
        }
    }
    if (json.contains("steps")) {
        for (const auto& item : json.at("steps")) {
            LogicStep step;
            if (item.contains("when")) {
                for (const auto& cond : item.at("when")) {
                    LogicCondition condition;
                    condition.signal = cond.value("signal", std::string{});
                    condition.op = cond.value("op", std::string("=="));
                    condition.asBool = cond.value("asBool", true);
                    condition.flag = cond.value("flag", false);
                    condition.number = cond.value("number", 0.0);
                    step.when.push_back(std::move(condition));
                }
            }
            if (item.contains("actions")) {
                for (const auto& act : item.at("actions")) {
                    LogicAction action;
                    action.kind = act.value("kind", std::string("wait"));
                    action.resource = act.value("resource", std::string{});
                    action.pose = act.value("pose", std::string{});
                    action.joint = act.value("joint", std::string{});
                    action.signal = act.value("signal", std::string{});
                    action.profile = act.value("profile", std::string("trapezoid"));
                    action.speed = act.value("speed", 0.0);
                    action.seconds = act.value("seconds", 0.0);
                    step.actions.push_back(std::move(action));
                }
            }
            block.steps.push_back(std::move(step));
        }
    }
    error.clear();
    return true;
}

nlohmann::json robotPathToJson(const RobotPath& path) {
    nlohmann::json json;
    json["name"] = path.name;
    json["resource"] = path.resource;
    json["external"] = path.external;
    json["workpiece"] = path.workpiece;
    json["points"] = nlohmann::json::array();
    for (const PathPoint& point : path.points) {
        json["points"].push_back({{"name", point.name},
                                  {"move", pathMoveName(point.move)},
                                  {"cartesian", point.cartesian},
                                  {"joints", point.joints},
                                  {"position", vec3ToJson(point.position)},
                                  {"orientation", quatToJson(point.orientation)},
                                  {"via", vec3ToJson(point.via)},
                                  {"speed", point.speed},
                                  {"zone", point.zone},
                                  {"external", point.external},
                                  {"externalValues", point.externalValues}});
    }
    return json;
}

bool robotPathFromJson(const nlohmann::json& json, RobotPath& path, std::string& error) {
    if (!json.is_object() || !json.contains("name")) {
        error = "robot path is missing a name";
        return false;
    }
    path = {};
    path.name = json.at("name").get<std::string>();
    path.resource = json.value("resource", std::string{});
    path.external = json.value("external", std::string("locked"));
    path.workpiece = json.value("workpiece", std::string{});
    if (json.contains("points")) {
        for (const auto& item : json.at("points")) {
            PathPoint point;
            point.name = item.value("name", std::string{});
            PathMove move = PathMove::Ptp;
            if (item.contains("move") && !pathMoveFromName(item.at("move").get<std::string>(), move)) {
                error = "unknown path move";
                return false;
            }
            point.move = move;
            point.cartesian = item.value("cartesian", false);
            if (item.contains("joints")) {
                for (const auto& value : item.at("joints")) {
                    point.joints.push_back(value.get<double>());
                }
            }
            if (item.contains("position")) {
                point.position = vec3FromJson(item.at("position"));
            }
            if (item.contains("orientation")) {
                point.orientation = quatFromJson(item.at("orientation"));
            }
            if (item.contains("via")) {
                point.via = vec3FromJson(item.at("via"));
            }
            point.speed = item.value("speed", 100.0);
            point.zone = item.value("zone", 0.0);
            point.external = item.value("external", std::string{});
            if (item.contains("externalValues")) {
                for (const auto& value : item.at("externalValues")) {
                    point.externalValues.push_back(value.get<double>());
                }
            }
            path.points.push_back(std::move(point));
        }
    }
    error.clear();
    return true;
}

}  // namespace sim
