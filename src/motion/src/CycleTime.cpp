#include "sim/motion/Motion.hpp"

#include "sim/core/Mesh.hpp"
#include "sim/physics/PhysicsEngine.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace sim {
namespace {

const RobotPath* findPath(const Scene& scene, const std::string& name) {
    for (const RobotPath& path : scene.robotPaths) {
        if (path.name == name) {
            return &path;
        }
    }
    return nullptr;
}

const LogicBlock* findBlock(const Scene& scene, const std::string& name) {
    for (const LogicBlock& block : scene.logicBlocks) {
        if (block.name == name) {
            return &block;
        }
    }
    return nullptr;
}

bool measureConveyor(double speed, Vec3 direction, double travel, double& seconds, std::string& note) {
    if (speed <= 1e-6 || travel <= 1e-6) {
        note = "conveyor speed is zero";
        return false;
    }
    Scene scene;
    scene.physicsDoc.defaultFriction = scene.physicsDoc.referenceFriction;
    Mech belt;
    belt.kind = MechKind::Conveyor;
    belt.motion = BodyMotion::Anchored;
    belt.halfExtents = {std::max(800.0, travel), 200, 20};
    belt.direction = length(direction) > 1e-8 ? normalize(direction) : Vec3{1, 0, 0};
    belt.speed = speed;
    belt.grip = 1;
    auto beltNode = scene.makeNode(NodeKind::Part, "belt");
    beltNode->local.translation = {0, 0, 20};
    beltNode->mech = belt;
    beltNode->meshId = scene.addMesh(makeBoxMesh(belt.halfExtents));
    scene.addChild(scene.root(), beltNode);

    Mech rider;
    rider.kind = MechKind::Rigid;
    rider.motion = BodyMotion::Surface;
    rider.halfExtents = {30, 30, 30};
    rider.kinematic = false;
    auto box = scene.makeNode(NodeKind::Part, "load");
    box->local.translation = {-travel * 0.5, 0, 50};
    box->mech = rider;
    box->meshId = scene.addMesh(makeBoxMesh(rider.halfExtents));
    scene.addChild(scene.root(), box);

    auto engine = createPhysicsEngine(PhysicsBackend::Builtin);
    std::string error;
    if (!engine->build(scene, error)) {
        note = error.empty() ? "conveyor world was not built" : error;
        return false;
    }
    const double x0 = box->local.translation.x;
    const double dt = Simulation::kFixedDt;
    double elapsed = 0;
    const double cap = std::max(1.0, travel / speed * 4.0);
    while (elapsed < cap) {
        engine->step(dt);
        elapsed += dt;
        if (box->local.translation.x - x0 >= travel - 1.0) {
            seconds = elapsed;
            std::ostringstream text;
            text.setf(std::ios::fixed);
            text.precision(1);
            text << "belt " << travel << " mm at " << speed << " mm/s";
            note = text.str();
            return true;
        }
    }
    note = "conveyor load did not travel";
    return false;
}

}  // namespace

std::string pathFingerprint(const RobotPath& path, const KinematicResource* resource) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(4);
    out << path.name << '|' << path.resource << '|' << path.external << '|' << path.workpiece << '|';
    for (const PathPoint& point : path.points) {
        out << point.name << ',' << pathMoveName(point.move) << ',' << (point.cartesian ? 1 : 0) << ',' << point.position.x
            << ',' << point.position.y << ',' << point.position.z << ',' << point.speed << ',' << point.zone << ','
            << point.external << ',';
        for (double value : point.joints) {
            out << value << ';';
        }
        out << ',';
        for (double value : point.externalValues) {
            out << value << ';';
        }
        out << '|';
    }
    if (resource != nullptr) {
        for (const KinJoint& joint : resource->joints) {
            out << joint.name << ',' << joint.maxSpeed << ',' << joint.maxAccel << ',' << joint.lower << ',' << joint.upper
                << ',' << joint.externalName << ',' << joint.position << ';';
        }
    }
    return out.str();
}

MeasureResult measureOperationTimes(Scene& scene, bool force) {
    MeasureResult result;
    for (Operation& operation : scene.operations) {
        if (operation.kind != OpKind::Task) {
            continue;
        }
        if (!operation.path.empty()) {
            const RobotPath* path = findPath(scene, operation.path);
            KinematicResource* resource = path == nullptr ? nullptr : findResource(scene.resources, path->resource);
            if (path == nullptr || resource == nullptr) {
                result.note += operation.id + " path resource missing; ";
                continue;
            }
            const std::string key = pathFingerprint(*path, resource);
            if (!force && operation.timeSource == "pinned") {
                if (operation.timeKey == "pinned") {
                    operation.timeKey = key;
                }
                if (operation.timeKey == key) {
                    continue;
                }
            } else if (!force && operation.timeSource == "plan" && operation.timeKey == key) {
                continue;
            }
            const MotionPlan plan = planRobotPath(scene, *resource, *path);
            if (!plan.ok || plan.seconds <= 0) {
                result.note += operation.id + " plan was not accepted; ";
                continue;
            }
            operation.seconds = plan.seconds;
            operation.timeSource = "plan";
            operation.timeKey = key;
            operation.timeNote.clear();
            for (const std::string& warning : plan.warnings) {
                if (!operation.timeNote.empty()) {
                    operation.timeNote += "; ";
                }
                operation.timeNote += warning;
            }
            ++result.planned;
            ++result.updated;
            continue;
        }
        if (!operation.logic.empty()) {
            if (!force && operation.timeSource == "pinned") {
                continue;
            }
            if (findBlock(scene, operation.logic) == nullptr) {
                result.note += operation.id + " logic missing; ";
                continue;
            }
            LogicMachine machine;
            std::string error;
            if (!machine.start(scene, operation.logic, error)) {
                result.note += error + "; ";
                continue;
            }
            const double dt = 0.01;
            double elapsed = 0;
            while (!machine.finished(operation.logic) && elapsed < 120.0) {
                machine.step(scene, dt);
                elapsed += dt;
            }
            if (!machine.finished(operation.logic)) {
                result.note += operation.id + " logic did not finish; ";
                continue;
            }
            operation.seconds = std::max(0.01, elapsed);
            operation.timeSource = "logic";
            operation.timeKey = operation.logic;
            operation.timeNote = machine.lastError().empty() ? "event" : machine.lastError();
            ++result.logic;
            ++result.updated;
            continue;
        }
        if (!force && operation.timeSource == "pinned") {
            continue;
        }
        const Station* station = findStation(scene.layout, operation.station);
        const auto node = station == nullptr || station->nodeName.empty() ? nullptr : scene.findByName(station->nodeName);
        if (node && node->mech.kind == MechKind::Conveyor && node->mech.speed > 1e-6) {
            const double travel = std::min(std::max(station->width * 0.25, 200.0), 2000.0);
            double seconds = 0;
            std::string note;
            if (!measureConveyor(node->mech.speed, node->mech.direction, travel, seconds, note)) {
                operation.timeNote = note;
                result.note += operation.id + " " + note + "; ";
                continue;
            }
            operation.seconds = seconds;
            operation.timeSource = "conveyor";
            operation.timeKey = std::to_string(node->mech.speed);
            operation.timeNote = note;
            ++result.conveyor;
            ++result.updated;
        }
    }
    return result;
}

}  // namespace sim
