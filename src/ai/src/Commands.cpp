#include "sim/ai/Commands.hpp"

#include "sim/collision/Query.hpp"

#include "sim/motion/Motion.hpp"

#include <vector>

namespace sim {
namespace {

CommandResult okValue(nlohmann::json value) {
    CommandResult result;
    result.ok = true;
    result.value = std::move(value);
    return result;
}

CommandResult fail(std::string error) {
    CommandResult result;
    result.error = std::move(error);
    return result;
}

std::shared_ptr<SceneNode> named(Scene& scene, const nlohmann::json& command, CommandResult& error) {
    const std::string name = command.value("name", "");
    auto node = scene.findByName(name);
    if (!node) {
        error = fail("no object named " + name);
    }
    return node;
}

}  // namespace

CommandExecutor::CommandExecutor(Scene& scene, Simulation* simulation, IPhysicsEngine* physics)
    : scene_(scene), simulation_(simulation), physics_(physics) {}

nlohmann::json CommandExecutor::catalog() {
    const auto op = [](const char* name, const char* summary, nlohmann::json params, bool executable) {
        return nlohmann::json{{"op", name}, {"summary", summary}, {"params", std::move(params)}, {"executable", executable}};
    };
    return nlohmann::json::array({
        op("get_bool", "Read a boolean signal. Missing signals read as false.", {"name"}, true),
        op("set_bool", "Write a boolean signal, creating it if needed.", {"name", "value"}, true),
        op("get_int", "Read an integer signal.", {"name"}, true),
        op("set_int", "Write an integer signal.", {"name", "value"}, true),
        op("get_real", "Read a real signal.", {"name"}, true),
        op("set_real", "Write a real signal.", {"name", "value"}, true),
        op("set_joint_target", "Same as sim.set_joint_target. Position units are mm or degrees.", {"name", "value"}, true),
        op("set_joint_speed", "Same as sim.set_joint_speed.", {"name", "value"}, true),
        op("get_transform", "Local pose as tx,ty,tz,qw,qx,qy,qz in millimeters.", {"name"}, true),
        op("set_transform", "Set local translation in millimeters.", {"name", "x", "y", "z"}, true),
        op("spawn", "Clone a named object. Returns the new name.", {"name"}, true),
        op("delete", "Remove a named object and its rigid body.", {"name"}, true),
        op("list_nodes", "Names and mechanism kinds in the scene.", nlohmann::json::array(), true),
        op("sim_play", "Start or resume the fixed-step simulation.", nlohmann::json::array(), true),
        op("sim_pause", "Pause. Time does not advance.", nlohmann::json::array(), true),
        op("sim_stop", "Stop, restore rest poses, delete transient bodies.", nlohmann::json::array(), true),
        op("sim_state", "state, time, and speed.", nlohmann::json::array(), true),
        op("kin_list", "Kinematic resources, joints, poses, logic blocks, and paths.", nlohmann::json::array(), true),
        op("kin_set_joint", "Set one independent joint and apply it to the scene.", {"resource", "joint", "value"}, true),
        op("kin_fk", "Forward kinematics of a scara or spherical-wrist resource.", {"resource"}, true),
        op("kin_ik", "Inverse kinematics. Returns joint values and writes them when apply is true.",
           {"resource", "x", "y", "z", "qw", "qx", "qy", "qz"}, true),
        op("kin_move_pose", "Move to a named pose with a trapezoid or scurve profile.", {"resource", "pose", "profile"}, true),
        op("logic_start", "Start one logic block. Blocks still exchange only signals.", {"name"}, true),
        op("logic_step", "Advance logic blocks by fixed steps. dt defaults to the simulation step.", {"dt", "steps"}, true),
        op("robot_plan", "Plan a PTP/LIN/CIRC path and return cycle time, warnings, and collision pairs.", {"path"}, true),
        op("attach_tool", "Parent a named node under the resource flange, keeping its world pose.", {"resource", "name"}, true),
        op("detach_tool", "Parent a named node back under the world, keeping its world pose.", {"name"}, true),
        op("collision_query", "Overlap and near-miss pairs. Clearance defaults to the scene value, millimeters.", {"clearance"}, true),
        op("min_distance", "Signed distance, witness points, and gradient between two named parts.", {"name", "other"}, true),
        op("sweep_check", "Continuous sweep of a part between local translations, or of a named robot path.",
           {"name", "path", "x0", "y0", "z0", "x1", "y1", "z1"}, true),
        op("nl_to_logic", "Future: natural language to a logic block or script.", {"text"}, false),
        op("layout_suggest", "Future: suggest a line layout. Does not edit the scene.", {"text"}, false),
        op("cycle_optimize", "Future: suggest cycle-time changes.", {"text"}, false),
        op("explain_anomaly", "Future: explain a signal or cycle anomaly.", {"text"}, false),
    });
}

CommandResult CommandExecutor::execute(const nlohmann::json& command) const {
    if (!command.is_object() || !command.contains("op")) {
        return fail("command needs an op field");
    }
    const std::string op = command.value("op", "");
    if (op == "get_bool") {
        const Signal* signal = scene_.findSignal(command.value("name", ""));
        return okValue(signal != nullptr && signal->flag);
    }
    if (op == "set_bool") {
        Signal& signal = scene_.upsertSignal(command.value("name", ""), SignalType::Bool);
        signal.flag = command.value("value", false);
        return okValue(signal.flag);
    }
    if (op == "get_int") {
        const Signal* signal = scene_.findSignal(command.value("name", ""));
        return okValue(signal == nullptr ? 0 : signal->integer);
    }
    if (op == "set_int") {
        Signal& signal = scene_.upsertSignal(command.value("name", ""), SignalType::Int);
        signal.integer = command.value("value", 0);
        return okValue(signal.integer);
    }
    if (op == "get_real") {
        const Signal* signal = scene_.findSignal(command.value("name", ""));
        return okValue(signal == nullptr ? 0.0 : signal->real);
    }
    if (op == "set_real") {
        Signal& signal = scene_.upsertSignal(command.value("name", ""), SignalType::Real);
        signal.real = command.value("value", 0.0);
        return okValue(signal.real);
    }
    if (op == "set_joint_target" || op == "set_joint_speed") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        node->mech.target = command.value("value", 0.0);
        node->mech.drive = op == "set_joint_speed" ? DriveMode::Speed : node->mech.drive == DriveMode::None ? DriveMode::Position : node->mech.drive;
        if (op == "set_joint_speed") {
            node->mech.drive = DriveMode::Speed;
        }
        return okValue(node->mech.target);
    }
    if (op == "get_transform") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        const Transform& local = node->local;
        return okValue(nlohmann::json::array({local.translation.x, local.translation.y, local.translation.z, local.rotation.w,
                                              local.rotation.x, local.rotation.y, local.rotation.z}));
    }
    if (op == "set_transform") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        node->local.translation = {command.value("x", 0.0), command.value("y", 0.0), command.value("z", 0.0)};
        return okValue(true);
    }
    if (op == "spawn") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        auto clone = scene_.spawnClone(node, node->local);
        if (!clone) {
            return fail("could not spawn " + command.value("name", ""));
        }
        if (physics_ != nullptr) {
            physics_->adoptSpawned(clone->id);
        }
        return okValue(clone->name);
    }
    if (op == "delete") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        const std::string id = node->id;
        if (physics_ != nullptr) {
            physics_->releaseBody(id);
        }
        scene_.removeNode(id);
        return okValue(true);
    }
    if (op == "list_nodes") {
        nlohmann::json names = nlohmann::json::array();
        std::vector<std::shared_ptr<SceneNode>> stack{scene_.root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (node->id == "root") {
                continue;
            }
            names.push_back({{"name", node->name}, {"kind", mechKindName(node->mech.kind)}});
        }
        return okValue(std::move(names));
    }
    if (op == "sim_state") {
        if (simulation_ == nullptr) {
            return fail("no simulation");
        }
        const char* state = "stopped";
        if (simulation_->state() == SimState::Playing) {
            state = "playing";
        } else if (simulation_->state() == SimState::Paused) {
            state = "paused";
        }
        return okValue({{"state", state}, {"time", simulation_->time()}, {"speed", simulation_->speed()}});
    }
    if (op == "sim_play" || op == "sim_pause" || op == "sim_stop") {
        if (simulation_ == nullptr) {
            return fail("no simulation");
        }
        if (op == "sim_play") {
            simulation_->play();
        } else if (op == "sim_pause") {
            simulation_->pause();
        } else {
            simulation_->stop();
        }
        if (!simulation_->lastError().empty()) {
            return fail(simulation_->lastError());
        }
        return execute(nlohmann::json{{"op", "sim_state"}});
    }
    if (op == "kin_list") {
        nlohmann::json resources = nlohmann::json::array();
        for (const KinematicResource& resource : scene_.resources) {
            nlohmann::json joints = nlohmann::json::array();
            for (const KinJoint& joint : resource.joints) {
                joints.push_back({{"name", joint.name}, {"position", jointCommand(resource, joint)}});
            }
            resources.push_back({{"name", resource.name}, {"ik", resource.ik}, {"joints", std::move(joints)}});
        }
        nlohmann::json blocks = nlohmann::json::array();
        for (const LogicBlock& block : scene_.logicBlocks) {
            blocks.push_back(block.name);
        }
        nlohmann::json paths = nlohmann::json::array();
        for (const RobotPath& path : scene_.robotPaths) {
            paths.push_back(path.name);
        }
        return okValue({{"resources", std::move(resources)}, {"logic", std::move(blocks)}, {"paths", std::move(paths)}});
    }
    if (op == "kin_set_joint") {
        KinematicResource* resource = findResource(scene_.resources, command.value("resource", ""));
        if (resource == nullptr) {
            return fail("no resource named " + command.value("resource", ""));
        }
        KinJoint* joint = findJoint(*resource, command.value("joint", ""));
        if (joint == nullptr) {
            return fail("no joint named " + command.value("joint", ""));
        }
        if (!joint->coupledTo.empty()) {
            return fail(joint->name + " is coupled to " + joint->coupledTo);
        }
        joint->position = clampJoint(*joint, command.value("value", 0.0));
        applyResource(scene_, *resource);
        return okValue(jointCommand(*resource, *joint));
    }
    if (op == "kin_fk") {
        const KinematicResource* resource = findResource(scene_.resources, command.value("resource", ""));
        if (resource == nullptr) {
            return fail("no resource named " + command.value("resource", ""));
        }
        const CartesianPose pose = forwardModel(*resource);
        return okValue({{"x", pose.position.x},
                        {"y", pose.position.y},
                        {"z", pose.position.z},
                        {"qw", pose.rotation.w},
                        {"qx", pose.rotation.x},
                        {"qy", pose.rotation.y},
                        {"qz", pose.rotation.z}});
    }
    if (op == "kin_ik") {
        KinematicResource* resource = findResource(scene_.resources, command.value("resource", ""));
        if (resource == nullptr) {
            return fail("no resource named " + command.value("resource", ""));
        }
        CartesianPose target;
        target.position = {command.value("x", 0.0), command.value("y", 0.0), command.value("z", 0.0)};
        target.rotation = Quat{command.value("qw", 1.0), command.value("qx", 0.0), command.value("qy", 0.0), command.value("qz", 0.0)}.normalized();
        const IkResult solved = inverseModel(*resource, target, jointVector(*resource));
        if (command.value("apply", true) && solved.ok) {
            setJointVector(*resource, solved.joints);
            applyResource(scene_, *resource);
        }
        return solved.ok ? okValue({{"joints", solved.joints}, {"warning", solved.warning}, {"approximated", solved.approximated}})
                         : fail(solved.warning.empty() ? "inverse kinematics failed" : solved.warning);
    }
    if (op == "kin_move_pose") {
        KinematicResource* resource = findResource(scene_.resources, command.value("resource", ""));
        if (resource == nullptr) {
            return fail("no resource named " + command.value("resource", ""));
        }
        ActiveMove move;
        std::string error;
        const ProfileKind profile = command.value("profile", std::string("trapezoid")) == "scurve" ? ProfileKind::SCurve : ProfileKind::Trapezoid;
        if (!beginPoseMove(*resource, command.value("pose", ""), profile, move, error)) {
            return fail(error);
        }
        for (int guard = 0; guard < 20000 && move.active; ++guard) {
            advanceMove(*resource, move, Simulation::kFixedDt);
        }
        applyResource(scene_, *resource);
        return okValue(jointVector(*resource));
    }
    if (op == "logic_start") {
        std::string error;
        if (!logic_.start(scene_, command.value("name", ""), error)) {
            return fail(error);
        }
        return okValue(true);
    }
    if (op == "logic_step") {
        const double dt = command.value("dt", Simulation::kFixedDt);
        const int steps = std::max(1, command.value("steps", 1));
        for (int i = 0; i < steps; ++i) {
            logic_.step(scene_, dt);
        }
        if (!logic_.lastError().empty()) {
            return fail(logic_.lastError());
        }
        return okValue(true);
    }
    if (op == "robot_plan") {
        const std::string pathName = command.value("path", "");
        const RobotPath* path = nullptr;
        for (const RobotPath& candidate : scene_.robotPaths) {
            if (candidate.name == pathName) {
                path = &candidate;
                break;
            }
        }
        if (path == nullptr) {
            return fail("no path named " + pathName);
        }
        KinematicResource* resource = findResource(scene_.resources, path->resource);
        if (resource == nullptr) {
            return fail("no resource named " + path->resource);
        }
        const MotionPlan plan = planRobotPath(scene_, *resource, *path);
        nlohmann::json hits = nlohmann::json::array();
        for (const auto& [a, b] : plan.collisions) {
            hits.push_back({{"a", a}, {"b", b}});
        }
        return okValue({{"ok", plan.ok}, {"seconds", plan.seconds}, {"warnings", plan.warnings}, {"collisions", std::move(hits)}});
    }
    if (op == "attach_tool" || op == "detach_tool") {
        std::string error;
        if (op == "detach_tool") {
            CommandResult missing;
            auto node = named(scene_, command, missing);
            if (!node) {
                return missing;
            }
            if (!detachTool(scene_, node->id, error)) {
                return fail(error);
            }
            return okValue(true);
        }
        KinematicResource* resource = findResource(scene_.resources, command.value("resource", ""));
        if (resource == nullptr) {
            return fail("no resource named " + command.value("resource", ""));
        }
        CommandResult missing;
        auto node = named(scene_, command, missing);
        if (!node) {
            return missing;
        }
        if (!attachTool(scene_, *resource, node->id, error)) {
            return fail(error);
        }
        return okValue(true);
    }
    if (op == "collision_query" || op == "min_distance" || op == "sweep_check") {
        auto query = createCollisionQuery();
        query->rebuild(scene_);
        if (op == "min_distance") {
            const DistanceResult distance = query->minDistance(scene_, command.value("name", ""), command.value("other", ""));
            if (!distance.ok) {
                return fail(distance.error.empty() ? "distance failed" : distance.error);
            }
            return okValue({{"distance", distance.distance},
                            {"pointA", {distance.pointA.x, distance.pointA.y, distance.pointA.z}},
                            {"pointB", {distance.pointB.x, distance.pointB.y, distance.pointB.z}},
                            {"normal", {distance.normal.x, distance.normal.y, distance.normal.z}},
                            {"gradient", {distance.gradient.x, distance.gradient.y, distance.gradient.z}},
                            {"gradientFromSdf", distance.gradientFromSdf}});
        }
        if (op == "sweep_check" && command.contains("path")) {
            const std::string pathName = command.value("path", "");
            const RobotPath* path = nullptr;
            for (const RobotPath& candidate : scene_.robotPaths) {
                if (candidate.name == pathName) {
                    path = &candidate;
                }
            }
            if (path == nullptr) {
                return fail("no path named " + pathName);
            }
            KinematicResource* resource = findResource(scene_.resources, path->resource);
            if (resource == nullptr) {
                return fail("no resource named " + path->resource);
            }
            const MotionPlan plan = planRobotPath(scene_, *resource, *path);
            return okValue({{"hit", plan.sweepHit},
                            {"time", plan.sweepTime},
                            {"a", plan.sweepA},
                            {"b", plan.sweepB}});
        }
        if (op == "sweep_check") {
            const SweepHit hit = query->sweep(scene_, command.value("name", ""),
                                              {command.value("x0", 0.0), command.value("y0", 0.0), command.value("z0", 0.0)},
                                              {command.value("x1", 0.0), command.value("y1", 0.0), command.value("z1", 0.0)});
            if (!hit.error.empty()) {
                return fail(hit.error);
            }
            return okValue({{"hit", hit.hit},
                            {"time", hit.time},
                            {"a", hit.a},
                            {"b", hit.b},
                            {"point", {hit.point.x, hit.point.y, hit.point.z}}});
        }
        const double clearance = command.contains("clearance") ? command.value("clearance", scene_.proximity.clearanceMm)
                                                               : scene_.proximity.clearanceMm;
        nlohmann::json hits = nlohmann::json::array();
        for (const ContactHit& hit : query->query(scene_, clearance)) {
            hits.push_back({{"a", hit.a},
                            {"b", hit.b},
                            {"kind", hit.kind == HitKind::Overlap ? "overlap" : "clearance"},
                            {"distance", hit.distance},
                            {"penetration", hit.penetration},
                            {"volume", hit.volume},
                            {"volumeEstimated", hit.volumeEstimated}});
        }
        query->commitCache(scene_);
        return okValue(hits);
    }
    if (op == "nl_to_logic" || op == "layout_suggest" || op == "cycle_optimize" || op == "explain_anomaly") {
        return fail(op + " is a documented hook and is not implemented");
    }
    return fail("unknown op " + op);
}

}  // namespace sim
