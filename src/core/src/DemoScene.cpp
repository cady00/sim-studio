#include "sim/core/DemoScene.hpp"

#include "sim/core/Mesh.hpp"

namespace sim {
namespace {

const char* kPusher = R"(def on_start():
    sim.set_bool("barrier", False)
    sim.set_joint_target("Pusher", 0)

hold = {"t": 0.0}

def on_step(dt):
    # The barrier sits upstream of the pusher. Hold the stroke so the part
    # arrives before the rod retracts.
    if sim.get_bool("barrier"):
        hold["t"] = 0.9
    if hold["t"] > 0.0:
        sim.set_joint_target("Pusher", 120)
        hold["t"] -= dt
    else:
        sim.set_joint_target("Pusher", 0)
)";

const char* kSequence = R"(state = {"t": 0.0}

def on_start():
    state["t"] = 0.0

def on_step(dt):
    state["t"] += dt
    if state["t"] < 1.5:
        sim.set_joint_target("Slider", 80)
    else:
        sim.set_joint_target("Slider", 0)
        sim.set_joint_target("Hinge", 40)
)";

const char* kCounter = R"(seen = {"n": 0, "hot": False}

def on_step(dt):
    blocked = sim.get_bool("barrier")
    if blocked and not seen["hot"]:
        seen["n"] += 1
        sim.set_int("parts_seen", seen["n"])
    seen["hot"] = blocked
)";

void addBox(Scene& scene, const std::string& name, Vec3 halfExtents, Vec3 translation, Vec3 color, const Mech& mech,
            bool visible = true) {
    auto node = scene.makeNode(NodeKind::Part, name);
    node->local.translation = translation;
    node->color = color;
    node->visible = visible;
    node->mech = mech;
    node->meshId = scene.addMesh(makeBoxMesh(halfExtents));
    scene.addChild(scene.root(), node);
}

}  // namespace

const char* pusherScriptSource() { return kPusher; }
const char* sequenceScriptSource() { return kSequence; }
const char* counterScriptSource() { return kCounter; }

void buildConveyorLine(Scene& scene) {
    scene.clear();
    scene.name = "conveyor_line";

    Mech floor;
    floor.kind = MechKind::Collision;
    floor.shape = CollisionShape::Box;
    floor.halfExtents = {650, 320, 10};
    addBox(scene, "Floor", floor.halfExtents, {0, 0, -10}, {0.12, 0.14, 0.20}, floor);

    Mech belt;
    belt.kind = MechKind::Conveyor;
    belt.shape = CollisionShape::Box;
    belt.halfExtents = {480, 90, 16};
    belt.direction = {1, 0, 0};
    belt.speed = 240;
    belt.kinematic = true;
    addBox(scene, "Conveyor", belt.halfExtents, {0, 0, 16}, {0.18, 0.28, 0.48}, belt);

    Mech blank;
    blank.kind = MechKind::Rigid;
    blank.autoMass = true;
    blank.shape = CollisionShape::Box;
    blank.halfExtents = {25, 25, 25};
    addBox(scene, "Blank", blank.halfExtents, {-520, 0, 180}, {0.57, 0.58, 0.95}, blank, false);

    Mech source;
    source.kind = MechKind::Source;
    source.interval = 1.2;
    source.prototype = "Blank";
    source.halfExtents = {20, 20, 20};
    addBox(scene, "Source", {18, 18, 18}, {-360, 0, 90}, {0.35, 0.75, 0.45}, source);

    Mech barrier;
    barrier.kind = MechKind::LightBarrier;
    barrier.axis = {0, 1, 0};
    barrier.range = 280;
    barrier.signal = "barrier";
    barrier.valueSignal = "barrier_distance";
    addBox(scene, "Barrier", {6, 8, 28}, {-80, -150, 62}, {0.95, 0.78, 0.25}, barrier);

    Mech base;
    base.kind = MechKind::Collision;
    base.shape = CollisionShape::Box;
    base.halfExtents = {24, 24, 24};
    addBox(scene, "PusherBase", base.halfExtents, {40, -210, 40}, {0.25, 0.28, 0.36}, base);

    Mech pusher;
    pusher.kind = MechKind::Slider;
    pusher.other = "PusherBase";
    pusher.axis = {0, 1, 0};
    pusher.limited = true;
    pusher.lower = 0;
    pusher.upper = 140;
    pusher.drive = DriveMode::Position;
    pusher.target = 0;
    pusher.maxSpeed = 280;
    pusher.maxAccel = 2200;
    pusher.autoMass = false;
    pusher.mass = 0.6;
    pusher.shape = CollisionShape::Box;
    pusher.halfExtents = {22, 36, 18};
    addBox(scene, "Pusher", pusher.halfExtents, {40, -160, 58}, {0.86, 0.45, 0.28}, pusher);

    Mech sink;
    sink.kind = MechKind::Sink;
    sink.halfExtents = {70, 110, 50};
    addBox(scene, "Sink", sink.halfExtents, {430, 0, 50}, {0.55, 0.22, 0.28}, sink);

    Signal blocked;
    blocked.name = "barrier";
    blocked.type = SignalType::Bool;
    scene.signals.push_back(blocked);
    Signal distance;
    distance.name = "barrier_distance";
    distance.type = SignalType::Real;
    distance.real = barrier.range;
    scene.signals.push_back(distance);

    scene.scripts.push_back(ProjectScript{"pusher", kPusher});
    scene.scripts.push_back(ProjectScript{"sequence", kSequence});
    scene.scripts.push_back(ProjectScript{"counter", kCounter});
    scene.activeScript = "pusher";
    scene.markClean();
}

void buildTransferLine(Scene& scene) {
    scene.clear();
    scene.name = "transfer_line";
    scene.physicsDoc.engine = "builtin";
    scene.physicsDoc.referenceFriction = 0.5;
    scene.physicsDoc.defaultFriction = 0.2;
    scene.physicsDoc.grades = {{"rubber", 0.6, 0.05}, {"steel", 0.3, 0.02}, {"slick", 0.05, 0.01}};
    scene.physicsDoc.pairs = {
        {"rubber", "rubber", 0.8, 0.05},
        {"slick", "rubber", 0.05, 0.02},
        {"rubber", "steel", 0.4, 0.04},
        {"slick", "steel", 0.02, 0.01},
    };

    Mech roller;
    roller.kind = MechKind::Conveyor;
    roller.motion = BodyMotion::Driven;
    roller.kinematic = true;
    roller.surface = "rubber";
    roller.direction = {1, 0, 0};
    roller.speed = 300;
    roller.grip = 1;
    roller.guide = 90;
    roller.nextName = "Belt";
    roller.motor = "rollerDrive";
    roller.halfExtents = {400, 90, 20};
    addBox(scene, "Roller", roller.halfExtents, {0, 0, 20}, {0.18, 0.32, 0.62}, roller);

    Mech belt;
    belt.kind = MechKind::Conveyor;
    belt.motion = BodyMotion::Driven;
    belt.kinematic = true;
    belt.surface = "steel";
    belt.direction = {1, 0, 0};
    belt.speed = 220;
    belt.grip = 1;
    belt.guide = 90;
    belt.halfExtents = {400, 90, 20};
    addBox(scene, "Belt", belt.halfExtents, {920, 0, 20}, {0.45, 0.48, 0.52}, belt);

    Mech gate;
    gate.kind = MechKind::Collision;
    gate.motion = BodyMotion::Anchored;
    gate.kinematic = true;
    gate.handoff = true;
    gate.halfExtents = {70, 120, 40};
    addBox(scene, "Handoff", gate.halfExtents, {460, 0, 70}, {0.95, 0.62, 0.2}, gate);

    Mech rubber;
    rubber.kind = MechKind::Rigid;
    rubber.motion = BodyMotion::Surface;
    rubber.surfaceRide = true;
    rubber.kinematic = true;
    rubber.surface = "rubber";
    rubber.autoMass = true;
    rubber.density = 600;
    rubber.nextName = "Belt";
    rubber.halfExtents = {30, 30, 30};
    addBox(scene, "RubberBox", rubber.halfExtents, {-220, -36, 65}, {0.72, 0.28, 0.22}, rubber);

    Mech slick = rubber;
    slick.surface = "slick";
    slick.density = 400;
    addBox(scene, "SlickBox", slick.halfExtents, {-220, 36, 65}, {0.85, 0.82, 0.45}, slick);

    MotorAspect drive;
    drive.name = "rollerDrive";
    drive.owner = "Roller";
    drive.running = true;
    drive.targetSpeed = 300;
    drive.speed = 300;
    drive.accel = 800;
    drive.decel = 1200;
    drive.pulseDistance = 25;
    drive.highRatio = 0.4;
    drive.grayCode = true;
    drive.countSignal = "encoder_count";
    drive.pulseSignal = "encoder_pulse";
    drive.graySignal = "encoder_gray";
    drive.runningSignal = "encoder_run";
    drive.speedSignal = "encoder_speed";
    drive.distanceSignal = "encoder_tick";
    drive.startedSignal = "drive_on";
    drive.stoppedSignal = "drive_off";
    drive.changedSignal = "drive_changed";
    drive.notifyEvery = 100;
    scene.motors.push_back(drive);
    scene.markClean();
}

}  // namespace sim
