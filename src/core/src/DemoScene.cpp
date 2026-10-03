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

}  // namespace sim
