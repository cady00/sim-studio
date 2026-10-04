#include "sim/core/Kinematics.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <sstream>

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
    json["repeat"] = block.repeat;
    json["pins"] = nlohmann::json::array();
    for (const auto& pin : block.pins) {
        json["pins"].push_back({{"key", pin.first}, {"x", pin.second.first}, {"y", pin.second.second}});
    }
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
    block.repeat = json.value("repeat", false);
    if (json.contains("pins")) {
        for (const auto& item : json.at("pins")) {
            const std::string key = item.value("key", std::string{});
            if (!key.empty()) {
                block.pins.emplace_back(key, std::make_pair(item.value("x", 0.0), item.value("y", 0.0)));
            }
        }
    }
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

namespace {

std::string trimCopy(const std::string& value) {
    std::size_t begin = 0;
    while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
        ++begin;
    }
    std::size_t end = value.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        --end;
    }
    return value.substr(begin, end - begin);
}

std::vector<std::string> splitWords(const std::string& line) {
    std::vector<std::string> words;
    std::istringstream in(line);
    std::string word;
    while (in >> word) {
        words.push_back(word);
    }
    return words;
}

}  // namespace

std::string logicToText(const LogicBlock& block) {
    std::ostringstream out;
    out << "name " << block.name << "\n";
    out << "in";
    for (const std::string& name : block.inputs) {
        out << " " << name;
    }
    out << "\nout";
    for (const std::string& name : block.outputs) {
        out << " " << name;
    }
    out << "\nrun " << block.runSignal << "\n";
    for (const LogicParam& param : block.parameters) {
        out << "param " << param.name << " " << param.type << " " << param.real << "\n";
    }
    out << "python:\n" << block.python;
    if (!block.python.empty() && block.python.back() != '\n') {
        out << "\n";
    }
    out << "---\n";
    for (const LogicStep& step : block.steps) {
        for (const LogicCondition& condition : step.when) {
            out << "when " << condition.signal << " " << condition.op << " " << condition.number << "\n";
        }
        for (const LogicAction& action : step.actions) {
            out << action.kind;
            if (action.kind == "wait" || action.kind == "jog") {
                out << " " << action.seconds;
                if (action.kind == "jog") {
                    out << " " << action.joint;
                }
            } else if (action.kind == "set_value") {
                out << " " << action.signal << " " << action.speed;
            } else if (action.kind == "move_pose") {
                out << " " << action.pose;
            } else if (!action.signal.empty()) {
                out << " " << action.signal;
            }
            out << "\n";
        }
        out << "step\n";
    }
    return out.str();
}

bool logicFromText(const std::string& text, LogicBlock& block, std::string& error) {
    LogicBlock parsed;
    parsed.pins = block.pins;
    parsed.repeat = block.repeat;
    parsed.resource = block.resource;
    std::string python;
    bool inPython = false;
    LogicStep step;
    bool wroteStep = false;
    std::istringstream lines(text);
    std::string line;
    try {
    while (std::getline(lines, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (inPython) {
            if (trimCopy(line) == "---") {
                inPython = false;
                continue;
            }
            python += line;
            python.push_back('\n');
            continue;
        }
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty()) {
            continue;
        }
        if (trimmed == "python:") {
            inPython = false;
            inPython = true;
            continue;
        }
        const std::vector<std::string> words = splitWords(trimmed);
        if (words[0] == "name" && words.size() >= 2) {
            parsed.name = words[1];
        } else if (words[0] == "in") {
            parsed.inputs.assign(words.begin() + 1, words.end());
        } else if (words[0] == "out") {
            parsed.outputs.assign(words.begin() + 1, words.end());
        } else if (words[0] == "run") {
            parsed.runSignal = words.size() >= 2 ? words[1] : std::string{};
        } else if (words[0] == "param" && words.size() >= 2) {
            LogicParam param;
            param.name = words[1];
            param.type = words.size() >= 3 ? words[2] : "real";
            param.real = words.size() >= 4 ? std::stod(words[3]) : 0;
            parsed.parameters.push_back(std::move(param));
        } else if (words[0] == "when" && words.size() >= 2) {
            LogicCondition condition;
            condition.signal = words[1];
            condition.op = words.size() >= 3 ? words[2] : "==";
            condition.asBool = false;
            condition.number = words.size() >= 4 ? std::stod(words[3]) : 0;
            step.when.push_back(std::move(condition));
        } else if (words[0] == "step") {
            parsed.steps.push_back(std::move(step));
            step = {};
            wroteStep = true;
        } else {
            LogicAction action;
            action.kind = words[0];
            if (action.kind == "wait") {
                action.seconds = words.size() >= 2 ? std::stod(words[1]) : 0;
            } else if (action.kind == "jog") {
                action.seconds = words.size() >= 2 ? std::stod(words[1]) : 0;
                action.joint = words.size() >= 3 ? words[2] : std::string{};
            } else if (action.kind == "set_value") {
                action.signal = words.size() >= 2 ? words[1] : std::string{};
                action.speed = words.size() >= 3 ? std::stod(words[2]) : 0;
            } else if (action.kind == "move_pose") {
                action.pose = words.size() >= 2 ? words[1] : std::string{};
            } else if (words.size() >= 2) {
                action.signal = words[1];
            }
            step.actions.push_back(std::move(action));
        }
    }
    } catch (const std::exception&) {
        error = "logic text has a bad number";
        return false;
    }
    if (!step.actions.empty() || !step.when.empty()) {
        parsed.steps.push_back(std::move(step));
        wroteStep = true;
    }
    if (parsed.name.empty()) {
        error = "logic text is missing a name";
        return false;
    }
    if (!wroteStep) {
        error = "logic text has no step";
        return false;
    }
    parsed.python = python;
    block = std::move(parsed);
    error.clear();
    return true;
}

bool wireLogicBlocks(LogicBlock& from, LogicBlock& to, std::string& signal) {
    if (&from == &to) {
        return false;
    }
    if (!from.outputs.empty()) {
        signal = from.outputs.front();
    } else {
        signal = from.name + "_out";
        from.outputs.push_back(signal);
    }
    if (std::find(to.inputs.begin(), to.inputs.end(), signal) == to.inputs.end()) {
        to.inputs.push_back(signal);
    }
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
                                  {"externalValues", point.externalValues},
                                  {"shape", point.shape}});
    }
    json["frames"] = nlohmann::json::array();
    for (const OlpFrame& frame : path.frames) {
        json["frames"].push_back({{"kind", frame.kind},
                                  {"origin", vec3ToJson(frame.origin)},
                                  {"a", frame.a},
                                  {"b", frame.b},
                                  {"c", frame.c}});
    }
    json["directives"] = nlohmann::json::array();
    for (const OlpDirective& directive : path.directives) {
        json["directives"].push_back({{"kind", directive.kind}, {"text", directive.text}});
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
            point.shape = item.value("shape", std::string{});
            if (item.contains("externalValues")) {
                for (const auto& value : item.at("externalValues")) {
                    point.externalValues.push_back(value.get<double>());
                }
            }
            path.points.push_back(std::move(point));
        }
    }
    if (json.contains("frames") && json.at("frames").is_array()) {
        for (const auto& item : json.at("frames")) {
            OlpFrame frame;
            frame.kind = item.value("kind", std::string{});
            if (item.contains("origin")) {
                frame.origin = vec3FromJson(item.at("origin"));
            }
            frame.a = item.value("a", 0.0);
            frame.b = item.value("b", 0.0);
            frame.c = item.value("c", 0.0);
            if (!frame.kind.empty()) {
                path.frames.push_back(std::move(frame));
            }
        }
    }
    if (json.contains("directives") && json.at("directives").is_array()) {
        for (const auto& item : json.at("directives")) {
            OlpDirective directive;
            directive.kind = item.value("kind", std::string{});
            directive.text = item.value("text", std::string{});
            if (!directive.text.empty()) {
                path.directives.push_back(std::move(directive));
            }
        }
    }
    error.clear();
    return true;
}

}  // namespace sim
