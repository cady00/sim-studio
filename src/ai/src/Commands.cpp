#include "sim/ai/Commands.hpp"

#include "sim/ai/Provider.hpp"
#include "sim/collision/Query.hpp"

#include "sim/io/AssetImport.hpp"
#include "sim/motion/Motion.hpp"

#include <algorithm>
#include <cctype>

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
        op("kin_ik_numeric", "Bounded numeric inverse kinematics. solver is quasi-newton or sqp.",
           {"resource", "x", "y", "z", "solver", "apply"}, true),
        op("physics_set_motion", "Set anchored, driven, free, or surface on a named body.", {"name", "motion"}, true),
        op("physics_pair", "Write one friction pair. Order does not matter.", {"a", "b", "friction", "restitution"}, true),
        op("motor_profile", "Stopping distance, stopping time, and time to reach speed.", {"speed", "decel"}, true),
        op("motor_set", "Create or update a named motor. Speeds match the owner, mm/s or deg/s.",
           {"name", "owner", "target", "running"}, true),
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
        op("line_list", "Stations, grid, title, and whether the line is example data.", nlohmann::json::array(), true),
        op("line_add_station", "Add or replace a station anchor in plan millimeters.",
           {"id", "name", "x", "y", "width", "depth", "kind", "resource", "node"}, true),
        op("line_move_station", "Move a station anchor. Coordinates are plan millimeters.", {"id", "x", "y"}, true),
        op("line_snap", "Snap every station anchor to the layout grid.", nlohmann::json::array(), true),
        op("line_align", "Align station left edges, then snap.", nlohmann::json::array(), true),
        op("line_distribute", "Distribute station anchors along X, then snap.", nlohmann::json::array(), true),
        op("line_import_dxf", "Import a DXF underlay: LINE, LWPOLYLINE, CIRCLE, ARC, SPLINE, INSERT, layers, ASCII and binary.", {"path", "mmPerUnit"}, true),
        op("line_set_underlay", "Store an image underlay in the project.", {"path", "opacity"}, true),
        op("line_measure", "Recompute task seconds from the path planner, logic events, or a conveyor step.", {"force"}, true),
        op("line_ops", "Operation tree: sequence, parallel, predecessors, path, and logic names.", nlohmann::json::array(), true),
        op("line_add_op", "Add or replace an operation. after is a list of finish-start ids.",
           {"id", "name", "station", "parent", "kind", "seconds", "index", "after", "path", "logic"}, true),
        op("line_schedule", "Critical path, makespan, and bottleneck station. Does not replan a robot path.", nlohmann::json::array(), true),
        op("line_export_csv", "Write the cycle report as CSV.", {"path"}, true),
        op("line_export_xlsx", "Write the cycle report as a spreadsheet.", {"path"}, true),
        op("ppr_list", "Product, process, and plant nodes.", nlohmann::json::array(), true),
        op("ems_import", "Read eBOP-style XML into operations, resources, parts, welds, paths, and placements.", {"path"}, true),
        op("ems_export", "Write operations, resources, parts, welds, paths, and placements to eBOP-style XML.", {"path"}, true),
        op("asset_import", "Import a file through the asset registry. Format is chosen by extension, then by content.", {"path"}, true),
        op("asset_export", "Export the scene as gltf, glb, stl, or usda.", {"path", "format"}, true),
        op("logic_upsert", "Create or replace a logic block. steps is a list of actions.", {"name", "steps", "repeat"}, true),
        op("line_set_cyclic", "Turn the operation list into a repeating production cycle.", {"value"}, true),
        op("nl_to_logic", "Offline rules turn a sentence into logic commands. An optional localhost model can replace the commands.", {"text"}, true),
        op("layout_suggest", "Offline rules add stations. An optional localhost model can replace the commands.", {"text"}, true),
        op("cycle_optimize", "Offline rules shorten a task that is more than twice the median and mark the line cyclic.", {"text"}, true),
        op("explain_anomaly", "Offline rules describe stuck signals and mark anomaly_explained.", {"text"}, true),
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
        scene_.notify(ChangeKind::Structure);
        return okValue(signal.flag);
    }
    if (op == "get_int") {
        const Signal* signal = scene_.findSignal(command.value("name", ""));
        return okValue(signal == nullptr ? 0 : signal->integer);
    }
    if (op == "set_int") {
        Signal& signal = scene_.upsertSignal(command.value("name", ""), SignalType::Int);
        signal.integer = command.value("value", 0);
        scene_.notify(ChangeKind::Structure);
        return okValue(signal.integer);
    }
    if (op == "get_real") {
        const Signal* signal = scene_.findSignal(command.value("name", ""));
        return okValue(signal == nullptr ? 0.0 : signal->real);
    }
    if (op == "set_real") {
        Signal& signal = scene_.upsertSignal(command.value("name", ""), SignalType::Real);
        signal.real = command.value("value", 0.0);
        scene_.notify(ChangeKind::Structure);
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
        scene_.notify(ChangeKind::Transform);
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
        scene_.notify(ChangeKind::Transform);
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
        scene_.notify(ChangeKind::Transform);
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
            scene_.notify(ChangeKind::Transform);
        }
        return solved.ok ? okValue({{"joints", solved.joints}, {"warning", solved.warning}, {"approximated", solved.approximated}})
                         : fail(solved.warning.empty() ? "inverse kinematics failed" : solved.warning);
    }
    if (op == "kin_ik_numeric") {
        KinematicResource* resource = findResource(scene_.resources, command.value("resource", ""));
        if (resource == nullptr) {
            return fail("no resource named " + command.value("resource", ""));
        }
        const CartesianPose targetPose{{command.value("x", 0.0), command.value("y", 0.0), command.value("z", 0.0)},
                                       Quat::identity()};
        std::vector<double> seed = jointVector(*resource);
        std::vector<double> lower;
        std::vector<double> upper;
        lower.reserve(resource->joints.size());
        upper.reserve(resource->joints.size());
        for (const KinJoint& joint : resource->joints) {
            lower.push_back(joint.limited ? std::min(joint.lower, joint.upper) : -1e6);
            upper.push_back(joint.limited ? std::max(joint.lower, joint.upper) : 1e6);
        }
        const std::string solverName = command.value("solver", std::string("quasi-newton"));
        const IkSolverKind kind = solverName == "sqp" ? IkSolverKind::SequentialQuadratic : IkSolverKind::QuasiNewton;
        IkSolverSettings settings;
        settings.iterations = command.value("iterations", 40);
        settings.epsilon = command.value("epsilon", 1e-2);
        settings.penalty = command.value("penalty", 8.0);
        const IkNumberResult solved = solveBoundedIk(
            [&](const std::vector<double>& values, std::vector<double>& residual) {
                KinematicResource trial = *resource;
                setJointVector(trial, values);
                const CartesianPose pose = forwardModel(trial);
                residual = {pose.position.x - targetPose.position.x, pose.position.y - targetPose.position.y,
                            pose.position.z - targetPose.position.z};
            },
            3, seed, lower, upper, kind, settings);
        if (command.value("apply", true) && solved.ok) {
            setJointVector(*resource, solved.values);
            applyResource(scene_, *resource);
            scene_.notify(ChangeKind::Transform);
        }
        return solved.ok ? okValue({{"joints", solved.values}, {"residual", solved.residual}, {"solver", solved.solver},
                                    {"iterations", solved.iterations}})
                         : fail("numeric inverse kinematics residual " + std::to_string(solved.residual));
    }
    if (op == "physics_set_motion") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        BodyMotion motion = BodyMotion::Free;
        if (!bodyMotionFromName(command.value("motion", std::string("free")), motion)) {
            return fail("unknown body motion");
        }
        node->mech.motion = motion;
        syncMotionFlags(node->mech);
        scene_.notify(ChangeKind::Structure);
        return okValue(bodyMotionName(node->mech.motion));
    }
    if (op == "physics_pair") {
        const std::string a = command.value("a", "");
        const std::string b = command.value("b", "");
        if (a.empty() || b.empty()) {
            return fail("pair needs a and b");
        }
        const double friction = command.value("friction", scene_.physicsDoc.defaultFriction);
        const double restitution = command.value("restitution", scene_.physicsDoc.defaultRestitution);
        for (GradePair& pair : scene_.physicsDoc.pairs) {
            if ((pair.a == a && pair.b == b) || (pair.a == b && pair.b == a)) {
                pair.friction = friction;
                pair.restitution = restitution;
                scene_.notify(ChangeKind::Structure);
                return okValue(pair.friction);
            }
        }
        scene_.physicsDoc.pairs.push_back(GradePair{a, b, friction, restitution});
        scene_.notify(ChangeKind::Structure);
        return okValue(friction);
    }
    if (op == "motor_profile") {
        const double speed = command.value("speed", 0.0);
        const double decel = command.value("decel", 0.0);
        return okValue({{"distance", stoppingDistance(speed, decel)},
                        {"time", stoppingTime(speed, decel)},
                        {"timeToSpeed", timeToSpeed(0, speed, command.value("accel", decel))}});
    }
    if (op == "motor_set") {
        const std::string name = command.value("name", "");
        if (name.empty()) {
            return fail("motor needs a name");
        }
        MotorAspect* motor = nullptr;
        for (MotorAspect& candidate : scene_.motors) {
            if (candidate.name == name) {
                motor = &candidate;
                break;
            }
        }
        if (motor == nullptr) {
            scene_.motors.push_back(MotorAspect{});
            motor = &scene_.motors.back();
            motor->name = name;
        }
        if (command.contains("owner")) {
            motor->owner = command.value("owner", "");
        }
        if (command.contains("target")) {
            motor->targetSpeed = command.value("target", 0.0);
        }
        if (command.contains("running")) {
            motor->running = command.value("running", false);
        }
        scene_.notify(ChangeKind::Structure);
        return okValue({{"name", motor->name}, {"target", motor->targetSpeed}, {"running", motor->running}});
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
        scene_.notify(ChangeKind::Transform);
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
            scene_.notify(ChangeKind::Structure);
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
        scene_.notify(ChangeKind::Structure);
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
    if (op == "line_list") {
        nlohmann::json stations = nlohmann::json::array();
        for (const Station& station : scene_.layout.stations) {
            stations.push_back({{"id", station.id},
                                {"name", station.name},
                                {"x", station.x},
                                {"y", station.y},
                                {"kind", stationKindName(station.kind)},
                                {"resource", station.resource},
                                {"node", station.nodeName}});
        }
        return okValue({{"example", scene_.layout.example},
                        {"title", scene_.layout.title},
                        {"gridMm", scene_.layout.gridMm},
                        {"underlay", scene_.layout.underlay.kind},
                        {"stations", std::move(stations)}});
    }
    if (op == "line_add_station") {
        const std::string id = command.value("id", "");
        if (id.empty()) {
            return fail("station needs an id");
        }
        Station station;
        if (Station* existing = findStation(scene_.layout, id)) {
            station = *existing;
        }
        station.id = id;
        if (command.contains("name")) {
            station.name = command.value("name", station.name);
        }
        station.x = command.value("x", station.x);
        station.y = command.value("y", station.y);
        station.width = command.value("width", station.width);
        station.depth = command.value("depth", station.depth);
        if (command.contains("kind")) {
            StationKind kind = station.kind;
            if (!stationKindFromName(command.value("kind", ""), kind)) {
                return fail("unknown station kind");
            }
            station.kind = kind;
        }
        if (command.contains("resource")) {
            station.resource = command.value("resource", "");
        }
        if (command.contains("node")) {
            station.nodeName = command.value("node", "");
        }
        if (Station* existing = findStation(scene_.layout, id)) {
            *existing = station;
        } else {
            scene_.layout.stations.push_back(station);
        }
        scene_.notify(ChangeKind::Structure);
        return okValue(id);
    }
    if (op == "line_move_station") {
        Station* station = findStation(scene_.layout, command.value("id", ""));
        if (station == nullptr) {
            return fail("no station named " + command.value("id", ""));
        }
        station->x = command.value("x", station->x);
        station->y = command.value("y", station->y);
        scene_.notify(ChangeKind::Structure);
        return okValue({{"x", station->x}, {"y", station->y}});
    }
    if (op == "line_snap") {
        snapStations(scene_.layout);
        scene_.notify(ChangeKind::Structure);
        return okValue(static_cast<int>(scene_.layout.stations.size()));
    }
    if (op == "line_align") {
        alignStationsMinX(scene_.layout);
        scene_.notify(ChangeKind::Structure);
        return okValue(true);
    }
    if (op == "line_distribute") {
        distributeStationsX(scene_.layout);
        scene_.notify(ChangeKind::Structure);
        return okValue(true);
    }
    if (op == "line_import_dxf") {
        std::string error;
        if (!loadDxfUnderlay(scene_.layout, command.value("path", ""), command.value("mmPerUnit", 1.0), error)) {
            return fail(error);
        }
        scene_.notify(ChangeKind::Structure);
        return okValue(static_cast<int>(scene_.layout.underlay.lines.size()));
    }
    if (op == "line_set_underlay") {
        setImageUnderlay(scene_.layout, command.value("path", ""), command.value("opacity", 0.35));
        scene_.notify(ChangeKind::Structure);
        return okValue(scene_.layout.underlay.path);
    }
    if (op == "line_ops") {
        return okValue(operationsToJson(scene_.operations));
    }
    if (op == "line_add_op") {
        const std::string id = command.value("id", "");
        if (id.empty()) {
            return fail("operation needs an id");
        }
        Operation operation;
        for (const Operation& existing : scene_.operations) {
            if (existing.id == id) {
                operation = existing;
            }
        }
        operation.id = id;
        if (command.contains("name")) {
            operation.name = command.value("name", operation.name);
        }
        if (command.contains("station")) {
            operation.station = command.value("station", "");
        }
        if (command.contains("parent")) {
            operation.parent = command.value("parent", "");
        }
        if (command.contains("kind")) {
            OpKind kind = operation.kind;
            if (!opKindFromName(command.value("kind", ""), kind)) {
                return fail("unknown operation kind");
            }
            operation.kind = kind;
        }
        operation.seconds = command.value("seconds", operation.seconds);
        operation.index = command.value("index", operation.index);
        if (command.contains("path")) {
            operation.path = command.value("path", "");
        }
        if (command.contains("logic")) {
            operation.logic = command.value("logic", "");
        }
        if (command.contains("after")) {
            operation.after.clear();
            const nlohmann::json& after = command.at("after");
            if (after.is_array()) {
                for (const auto& item : after) {
                    if (item.is_string()) {
                        operation.after.push_back(item.get<std::string>());
                    }
                }
            } else if (after.is_string() && !after.get<std::string>().empty()) {
                operation.after.push_back(after.get<std::string>());
            }
        }
        bool replaced = false;
        for (Operation& existing : scene_.operations) {
            if (existing.id == id) {
                existing = operation;
                replaced = true;
            }
        }
        if (!replaced) {
            scene_.operations.push_back(operation);
        }
        scene_.notify(ChangeKind::Structure);
        return okValue(id);
    }
    if (op == "line_schedule") {
        const ScheduleReport report = scheduleOperations(scene_.operations, scene_.layout);
        if (!report.error.empty()) {
            return fail(report.error);
        }
        return okValue(scheduleToJson(report));
    }
    if (op == "line_export_csv") {
        const ScheduleReport report = scheduleOperations(scene_.operations, scene_.layout);
        std::string error = report.error;
        if (error.empty() && !exportScheduleCsv(report, command.value("path", ""), error)) {
            return fail(error.empty() ? "csv export failed" : error);
        }
        if (!error.empty()) {
            return fail(error);
        }
        return okValue(command.value("path", ""));
    }
    if (op == "line_export_xlsx") {
        const ScheduleReport report = scheduleOperations(scene_.operations, scene_.layout);
        std::string error = report.error;
        if (error.empty() && !exportScheduleXlsx(report, command.value("path", ""), error)) {
            return fail(error.empty() ? "xlsx export failed" : error);
        }
        if (!error.empty()) {
            return fail(error);
        }
        return okValue(command.value("path", ""));
    }
    if (op == "ppr_list") {
        return okValue(pprToJson(scene_.ppr));
    }
    if (op == "line_measure") {
        const MeasureResult measured = measureOperationTimes(scene_, command.value("force", false));
        scene_.notify(ChangeKind::Structure);
        return okValue({{"updated", measured.updated},
                        {"planned", measured.planned},
                        {"logic", measured.logic},
                        {"conveyor", measured.conveyor},
                        {"note", measured.note}});
    }
    if (op == "ems_import") {
        const EmsImportResult imported = importEmsXml(command.value("path", ""), scene_);
        if (!imported.ok) {
            return fail(imported.message);
        }
        scene_.notify(ChangeKind::Structure);
        return okValue(imported.message);
    }
    if (op == "ems_export") {
        const EmsImportResult written = exportEmsXml(scene_, command.value("path", ""));
        if (!written.ok) {
            return fail(written.message);
        }
        return okValue(written.message);
    }
    if (op == "asset_import") {
        const auto registry = createBuiltinRegistry();
        const AssetImportResult imported = registry->importPath(command.value("path", ""), scene_);
        if (!imported.ok) {
            return fail(imported.message);
        }
        scene_.notify(ChangeKind::Structure);
        return okValue({{"format", imported.format},
                        {"nodes", imported.nodeCount},
                        {"triangles", imported.triangleCount},
                        {"message", imported.message}});
    }
    if (op == "asset_export") {
        const std::string path = command.value("path", "");
        std::string format = command.value("format", "");
        if (format.empty()) {
            const auto dot = path.find_last_of('.');
            format = dot == std::string::npos ? "" : path.substr(dot + 1);
        }
        for (char& c : format) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        std::string error;
        bool ok = false;
        if (format == "stl") {
            ok = exportSceneStl(scene_, path, error);
        } else if (format == "gltf" || format == "glb") {
            ok = exportSceneGltf(scene_, path, error);
        } else if (format == "csv") {
            const ScheduleReport report = scheduleOperations(scene_.operations, scene_.layout);
            error = report.error;
            ok = error.empty() && exportScheduleCsv(report, path, error);
        } else if (format == "xlsx") {
            const ScheduleReport report = scheduleOperations(scene_.operations, scene_.layout);
            error = report.error;
            ok = error.empty() && exportScheduleXlsx(report, path, error);
        } else if (format == "usda" || format == "usd") {
            const UsdIoResult written = exportSceneUsda(scene_, path);
            ok = written.ok;
            error = written.message;
        } else {
            return fail("export format is gltf, glb, stl, usda, csv, or xlsx");
        }
        if (!ok) {
            return fail(error.empty() ? "export failed" : error);
        }
        return okValue(path);
    }
    if (op == "logic_upsert") {
        const std::string name = command.value("name", "");
        if (name.empty()) {
            return fail("logic block needs a name");
        }
        LogicBlock block;
        block.name = name;
        block.repeat = command.value("repeat", false);
        block.resource = command.value("resource", "");
        if (command.contains("steps") && command.at("steps").is_array()) {
            LogicStep step;
            for (const auto& item : command.at("steps")) {
                if (!item.is_object()) {
                    continue;
                }
                LogicAction action;
                action.kind = item.value("kind", "wait");
                action.signal = item.value("signal", "");
                action.seconds = item.value("seconds", 0.0);
                step.actions.push_back(std::move(action));
            }
            if (!step.actions.empty()) {
                block.steps.push_back(std::move(step));
            }
        }
        bool replaced = false;
        for (LogicBlock& existing : scene_.logicBlocks) {
            if (existing.name == name) {
                existing = block;
                replaced = true;
                break;
            }
        }
        if (!replaced) {
            scene_.logicBlocks.push_back(std::move(block));
        }
        scene_.notify(ChangeKind::Structure);
        return okValue(name);
    }
    if (op == "line_set_cyclic") {
        scene_.layout.cyclic = command.value("value", true);
        scene_.notify(ChangeKind::Structure);
        return okValue(scene_.layout.cyclic);
    }
    if (op == "nl_to_logic" || op == "layout_suggest" || op == "cycle_optimize" || op == "explain_anomaly") {
        const std::string text = command.value("text", "");
        std::string folded = text;
        for (char& character : folded) {
            character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
        }
        nlohmann::json commands = nlohmann::json::array();
        std::string note;
        if (op == "nl_to_logic") {
            std::string cue = "barrier";
            std::string act = "push";
            const auto when = folded.find("when ");
            const auto then = folded.find(" then ");
            if (when != std::string::npos && then != std::string::npos && then > when) {
                cue = folded.substr(when + 5, then - (when + 5));
                act = folded.substr(then + 6);
            }
            const auto dang = text.find("当");
            const auto ze = text.find("则");
            if (dang != std::string::npos && ze != std::string::npos && ze > dang) {
                cue = text.substr(dang + std::string("当").size(), ze - (dang + std::string("当").size()));
                act = text.substr(ze + std::string("则").size());
            }
            auto trim = [](std::string value) {
                while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) {
                    value.erase(value.begin());
                }
                while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
                    value.pop_back();
                }
                return value;
            };
            cue = trim(cue);
            act = trim(act);
            if (cue.empty()) {
                cue = "go";
            }
            if (act.empty()) {
                act = "done";
            }
            if (text.find("气缸") != std::string::npos || folded.find("cylinder") != std::string::npos) {
                act = "cylinder";
            }
            commands.push_back({{"op", "logic_upsert"},
                                {"name", "nl"},
                                {"repeat", true},
                                {"steps",
                                 nlohmann::json::array({{{"kind", "wait_signal"}, {"signal", cue}},
                                                        {{"kind", "set_output"}, {"signal", act}}})}});
            note = "rule: wait " + cue + " then set " + act;
        } else if (op == "layout_suggest") {
            if (text.find("焊接") != std::string::npos || folded.find("weld") != std::string::npos) {
                commands.push_back({{"op", "line_add_station"}, {"id", "st_weld"}, {"name", "焊接"}, {"kind", "robot"}, {"x", 2000}});
                note = "rule: weld station";
            } else {
                commands.push_back({{"op", "line_add_station"}, {"id", "st_load"}, {"name", "上料"}, {"kind", "manual"}, {"x", 0}});
                commands.push_back({{"op", "line_add_station"}, {"id", "st_weld"}, {"name", "焊接"}, {"kind", "robot"}, {"x", 2500}});
                commands.push_back({{"op", "line_add_station"}, {"id", "st_inspect"}, {"name", "检查"}, {"kind", "manual"}, {"x", 5000}});
                note = "rule: load, weld, inspect";
            }
        } else if (op == "cycle_optimize") {
            std::vector<double> spans;
            for (const Operation& operation : scene_.operations) {
                if (operation.kind == OpKind::Task && operation.seconds > 0) {
                    spans.push_back(operation.seconds);
                }
            }
            std::sort(spans.begin(), spans.end());
            const double median = spans.empty() ? 0 : spans[(spans.size() - 1) / 2];
            for (const Operation& operation : scene_.operations) {
                if (operation.kind == OpKind::Task && median > 0 && operation.seconds > median * 2.0) {
                    commands.push_back({{"op", "line_add_op"},
                                        {"id", operation.id},
                                        {"name", operation.name},
                                        {"station", operation.station},
                                        {"parent", operation.parent},
                                        {"kind", "task"},
                                        {"seconds", median}});
                }
            }
            commands.push_back({{"op", "line_set_cyclic"}, {"value", true}});
            note = spans.empty() ? "rule: line set cyclic" : "rule: cap tasks above twice the median";
        } else {
            std::string stuck;
            for (const Signal& signal : scene_.signals) {
                if (signal.type == SignalType::Bool && signal.flag) {
                    if (!stuck.empty()) {
                        stuck += ", ";
                    }
                    stuck += signal.name;
                }
            }
            note = stuck.empty() ? "没有保持为真的布尔信号" : "这些布尔信号保持为真: " + stuck;
            commands.push_back({{"op", "set_bool"}, {"name", "anomaly_explained"}, {"value", true}});
        }
        if (command.value("use_model", false)) {
            ProviderConfig config;
            config.enabled = true;
            config.kind = "openai-compatible";
            config.endpoint = command.value("endpoint", "http://127.0.0.1:8080/v1/chat/completions");
            config.model = command.value("model", "local");
            auto provider = createAiProvider(config);
            std::string modelError;
            const nlohmann::json reply = provider->complete(
                {{"messages", nlohmann::json::array({{{"role", "user"}, {"content", text}}})}}, modelError);
            if (reply.contains("commands") && reply.at("commands").is_array()) {
                commands = reply.at("commands");
                note = "localhost model";
            } else if (!modelError.empty()) {
                note += "; " + modelError;
            }
        }
        nlohmann::json applied = nlohmann::json::array();
        for (const auto& generated : commands) {
            if (!generated.is_object()) {
                continue;
            }
            const std::string generatedOp = generated.value("op", "");
            if (generatedOp == "nl_to_logic" || generatedOp == "layout_suggest" || generatedOp == "cycle_optimize" ||
                generatedOp == "explain_anomaly") {
                return fail("model command cannot call " + generatedOp);
            }
            const CommandResult step = execute(generated);
            if (!step.ok) {
                return fail(step.error.empty() ? "generated command failed" : step.error);
            }
            applied.push_back({{"op", generatedOp}, {"value", step.value}});
        }
        return okValue({{"note", note}, {"commands", commands}, {"applied", applied}});
    }
    return fail("unknown op " + op);
}

}  // namespace sim
