#include "sim/core/DemoScene.hpp"

#include "sim/core/Mesh.hpp"

#include <algorithm>
#include <cmath>

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

void buildMeshGapDemo(Scene& scene) {
    scene.clear();
    scene.name = "mesh_gap";
    scene.physicsDoc.engine = "builtin";
    scene.physicsDoc.defaultRestitution = 0.02;

    auto mesh = makeBoxMesh({40, 40, 15});
    for (std::size_t i = 0; i + 2 < mesh->positions.size(); i += 3) {
        mesh->positions[i] += 120.f;
    }
    const auto base = static_cast<std::uint32_t>(mesh->vertexCount());
    const Vec3 far[3] = {{-240, -8, -4}, {-210, 8, -4}, {-225, 0, 12}};
    for (const Vec3& point : far) {
        mesh->positions.push_back(static_cast<float>(point.x));
        mesh->positions.push_back(static_cast<float>(point.y));
        mesh->positions.push_back(static_cast<float>(point.z));
        mesh->normals.push_back(0);
        mesh->normals.push_back(0);
        mesh->normals.push_back(1);
    }
    mesh->indices.insert(mesh->indices.end(), {base, base + 1, base + 2});
    mesh->recomputeBounds();

    Mech plate;
    plate.kind = MechKind::Collision;
    plate.motion = BodyMotion::Anchored;
    plate.kinematic = true;
    plate.shape = CollisionShape::Mesh;
    plate.halfExtents = {280, 120, 80};
    plate.surface = "steel";
    auto plateNode = scene.makeNode(NodeKind::Part, "Plate");
    plateNode->mech = plate;
    plateNode->meshId = scene.addMesh(mesh);
    plateNode->color = {0.35, 0.42, 0.55};
    scene.addChild(scene.root(), plateNode);

    Mech supported;
    supported.kind = MechKind::Rigid;
    supported.motion = BodyMotion::Free;
    supported.surface = "rubber";
    supported.halfExtents = {18, 18, 18};
    supported.allowSleep = true;
    addBox(scene, "Supported", supported.halfExtents, {120, 0, 40}, {0.18, 0.62, 0.38}, supported);

    Mech falling = supported;
    falling.surface = "rubber";
    addBox(scene, "FallingGap", falling.halfExtents, {0, 0, 90}, {0.86, 0.42, 0.22}, falling);
    scene.markClean();
}

void buildSlipDemo(Scene& scene) {
    scene.clear();
    scene.name = "slip_line";
    scene.physicsDoc.engine = "builtin";
    scene.physicsDoc.defaultFriction = 0.2;
    scene.physicsDoc.defaultRestitution = 0;
    scene.physicsDoc.grades = {{"rubber", 0.9, 0}, {"steel", 0.3, 0}, {"slick", 0.02, 0}};
    scene.physicsDoc.pairs = {{"rubber", "steel", 1.0, 0}, {"slick", "steel", 0.02, 0}};

    Mech belt;
    belt.kind = MechKind::Conveyor;
    belt.motion = BodyMotion::Driven;
    belt.kinematic = true;
    belt.surface = "steel";
    belt.direction = {1, 0, 0};
    belt.speed = 400;
    belt.grip = 1;
    belt.halfExtents = {520, 90, 20};
    addBox(scene, "Belt", belt.halfExtents, {0, 0, 20}, {0.42, 0.45, 0.5}, belt);

    Mech grip;
    grip.kind = MechKind::Rigid;
    grip.motion = BodyMotion::Free;
    grip.surface = "rubber";
    grip.surfaceRide = false;
    grip.kinematic = false;
    grip.autoMass = false;
    grip.mass = 2;
    grip.halfExtents = {24, 24, 24};
    grip.allowSleep = false;
    addBox(scene, "GripBox", grip.halfExtents, {-180, -36, 63}, {0.72, 0.28, 0.22}, grip);

    Mech slick = grip;
    slick.surface = "slick";
    slick.mass = 2;
    addBox(scene, "SlipBox", slick.halfExtents, {-180, 36, 63}, {0.85, 0.82, 0.45}, slick);
    scene.markClean();
}

void buildExampleLine(Scene& scene) {
    scene.clear();
    scene.name = "example_weld_line";
    scene.layout.example = true;
    scene.layout.title = "示例焊装线";
    scene.layout.gridMm = 500;
    scene.ppr.example = true;
    scene.ppr.title = "示例焊装线";

    const char* outline = R"(0
SECTION
2
ENTITIES
0
LWPOLYLINE
70
1
10
-800
20
-1600
10
9800
20
-1600
10
9800
20
1600
10
-800
20
1600
0
LINE
10
-800
20
0
11
9800
21
0
0
ENDSEC
0
EOF
)";
    const DxfResult parsed = parseDxf(outline, 1);
    scene.layout.underlay.kind = "dxf";
    scene.layout.underlay.mmPerUnit = 1;
    scene.layout.underlay.opacity = 0.45;
    scene.layout.underlay.lines = parsed.lines;

    auto addStation = [&](const std::string& id, const std::string& name, double x, StationKind kind, const std::string& resource,
                          const std::string& node, Vec3 color) {
        Station station;
        station.id = id;
        station.name = name;
        station.x = x;
        station.y = 0;
        station.width = 2000;
        station.depth = 1800;
        station.kind = kind;
        station.resource = resource;
        station.nodeName = node;
        scene.layout.stations.push_back(station);
        Mech pad;
        pad.kind = MechKind::Collision;
        pad.motion = BodyMotion::Anchored;
        pad.halfExtents = {1000, 900, 20};
        addBox(scene, node, pad.halfExtents, {x, 0, 20}, color, pad);
    };
    addStation("st_load", "上料", 0, StationKind::Manual, "", "LoadPad", {0.35, 0.55, 0.78});
    addStation("st_weld", "焊接", 2500, StationKind::Robot, "kr420", "KR420", {0.17, 0.43, 0.81});
    addStation("st_inspect", "检查", 5000, StationKind::Fixture, "", "InspectPad", {0.45, 0.62, 0.55});
    addStation("st_unload", "下料", 7500, StationKind::Conveyor, "", "UnloadPad", {0.55, 0.48, 0.32});

    Mech column;
    column.kind = MechKind::Collision;
    column.motion = BodyMotion::Anchored;
    column.halfExtents = {280, 280, 420};
    addBox(scene, "KR420Column", column.halfExtents, {2500, 0, 460}, {0.24, 0.53, 0.91}, column);

    auto addOp = [&](const std::string& id, const std::string& name, const std::string& parent, OpKind kind, int index,
                     const std::string& station, double seconds, const std::string& path, const std::string& logic) {
        Operation operation;
        operation.id = id;
        operation.name = name;
        operation.parent = parent;
        operation.kind = kind;
        operation.index = index;
        operation.station = station;
        operation.seconds = seconds;
        operation.path = path;
        operation.logic = logic;
        scene.operations.push_back(std::move(operation));
    };
    addOp("line", "节拍", "", OpKind::Sequence, 0, "", 0, "", "");
    addOp("load", "上料", "line", OpKind::Task, 0, "st_load", 8, "", "load_seq");
    addOp("weld", "焊接", "line", OpKind::Task, 1, "st_weld", 12, "cycle", "weld_seq");
    addOp("branch", "检查并行", "line", OpKind::Parallel, 2, "", 0, "", "");
    addOp("inspect", "检查", "branch", OpKind::Task, 0, "st_inspect", 6, "", "inspect_seq");
    addOp("prep", "夹具准备", "branch", OpKind::Task, 1, "st_inspect", 4, "", "");
    addOp("unload", "下料", "line", OpKind::Task, 3, "st_unload", 5, "", "");
    for (Operation& operation : scene.operations) {
        if (operation.id == "unload") {
            operation.wait = 1;
        }
    }
    if (auto pad = scene.findByName("UnloadPad")) {
        pad->mech.kind = MechKind::Conveyor;
        pad->mech.motion = BodyMotion::Anchored;
        pad->mech.speed = 250;
        pad->mech.direction = {1, 0, 0};
    }
    Mech eye;
    eye.kind = MechKind::CollisionSensor;
    eye.motion = BodyMotion::Anchored;
    eye.halfExtents = {40, 80, 40};
    eye.signal = "part_seen";
    addBox(scene, "InspectEye", eye.halfExtents, {5000, 0, 140}, {0.95, 0.72, 0.28}, eye);

    ProductNode product;
    product.id = "prod_bracket";
    product.name = "示例支架";
    product.partName = "Bracket";
    scene.ppr.products.push_back(product);
    ProcessNode process;
    process.id = "proc_weld";
    process.name = "焊接工艺";
    process.operation = "weld";
    process.product = "prod_bracket";
    scene.ppr.processes.push_back(process);
    PlantNode plant;
    plant.id = "plant_weld";
    plant.name = "焊接工位";
    plant.station = "st_weld";
    plant.resource = "kr420";
    scene.ppr.plants.push_back(plant);

    RobotPath path;
    path.name = "cycle";
    path.resource = "kr420";
    PathPoint home;
    home.name = "home";
    home.move = PathMove::Ptp;
    home.speed = 50;
    home.zone = 20;
    PathPoint away;
    away.name = "away";
    away.move = PathMove::Lin;
    away.cartesian = true;
    away.position = {2500, 0, 800};
    away.speed = 40;
    away.zone = 30;
    path.points.push_back(home);
    path.points.push_back(away);
    scene.robotPaths.push_back(std::move(path));

    LogicBlock block;
    block.name = "weld_seq";
    block.resource = "kr420";
    LogicAction action;
    action.kind = "set_output";
    action.signal = "weld_done";
    LogicStep step;
    step.actions.push_back(action);
    block.steps.push_back(std::move(step));
    scene.logicBlocks.push_back(std::move(block));
    LogicBlock load;
    load.name = "load_seq";
    LogicAction wait;
    wait.kind = "wait";
    wait.seconds = 2;
    LogicStep loadStep;
    loadStep.actions.push_back(wait);
    load.steps.push_back(std::move(loadStep));
    scene.logicBlocks.push_back(std::move(load));
    LogicBlock inspect;
    inspect.name = "inspect_seq";
    LogicAction seen;
    seen.kind = "wait_signal";
    seen.signal = "part_seen";
    LogicStep inspectStep;
    inspectStep.actions.push_back(seen);
    inspect.steps.push_back(std::move(inspectStep));
    scene.logicBlocks.push_back(std::move(inspect));
    for (LogicBlock& block : scene.logicBlocks) {
        block.repeat = true;
    }
    scene.layout.cyclic = true;

    Mech blank;
    blank.kind = MechKind::Collision;
    blank.motion = BodyMotion::Anchored;
    blank.halfExtents = {40, 40, 40};
    addBox(scene, "Blank", blank.halfExtents, {0, 400, 80}, {0.72, 0.55, 0.28}, blank, false);
    Mech feeder;
    feeder.kind = MechKind::Source;
    feeder.motion = BodyMotion::Anchored;
    feeder.interval = 8;
    feeder.prototype = "Blank";
    feeder.halfExtents = {30, 30, 30};
    addBox(scene, "Feeder", feeder.halfExtents, {0, 0, 200}, {0.20, 0.45, 0.75}, feeder);
    Mech sink;
    sink.kind = MechKind::Sink;
    sink.motion = BodyMotion::Anchored;
    sink.valueSignal = "shipped";
    sink.halfExtents = {80, 80, 40};
    addBox(scene, "Ship", sink.halfExtents, {7500, 0, 40}, {0.35, 0.35, 0.38}, sink);
    scene.markClean();
}

void buildDeviceDemo(Scene& scene) {
    scene.clear();
    scene.name = "device_cell";
    scene.physicsDoc.engine = "builtin";
    scene.physicsDoc.defaultRestitution = 0.02;

    Mech floor;
    floor.kind = MechKind::Collision;
    floor.motion = BodyMotion::Anchored;
    floor.kinematic = true;
    floor.halfExtents = {520, 280, 8};
    addBox(scene, "Floor", floor.halfExtents, {40, -80, -8}, {0.12, 0.14, 0.20}, floor);

    Mech master;
    master.kind = MechKind::Hinge;
    master.motion = BodyMotion::Driven;
    master.kinematic = true;
    master.axis = {0, 0, 1};
    master.limited = true;
    master.lower = 0;
    master.upper = 120;
    master.control = "position";
    master.setpoint = "gear_cmd";
    master.maxSpeed = 360;
    master.maxAccel = 4000;
    master.halfExtents = {70, 14, 14};
    addBox(scene, "GearIn", master.halfExtents, {-280, 40, 70}, {0.18, 0.42, 0.78}, master);

    Mech gear;
    gear.kind = MechKind::Hinge;
    gear.motion = BodyMotion::Driven;
    gear.kinematic = true;
    gear.axis = {0, 0, 1};
    gear.limited = false;
    gear.couple = "gear";
    gear.master = "GearIn";
    gear.ratio = 2;
    gear.halfExtents = {70, 14, 14};
    addBox(scene, "GearOut", gear.halfExtents, {-80, 40, 70}, {0.22, 0.62, 0.38}, gear);

    Mech cam;
    cam.kind = MechKind::Hinge;
    cam.motion = BodyMotion::Driven;
    cam.kinematic = true;
    cam.axis = {0, 0, 1};
    cam.limited = false;
    cam.couple = "cam";
    cam.master = "GearIn";
    cam.camIn = {0, 90};
    cam.camOut = {0, 36};
    cam.halfExtents = {64, 12, 12};
    addBox(scene, "CamOut", cam.halfExtents, {120, 40, 70}, {0.90, 0.62, 0.18}, cam);

    Mech ecam;
    ecam.kind = MechKind::Slider;
    ecam.motion = BodyMotion::Driven;
    ecam.kinematic = true;
    ecam.axis = {1, 0, 0};
    ecam.limited = false;
    ecam.couple = "ecam";
    ecam.master = "cam_master";
    ecam.camIn = {0, 90};
    ecam.camOut = {0, 80};
    ecam.halfExtents = {28, 16, 16};
    addBox(scene, "Ecam", ecam.halfExtents, {300, 40, 48}, {0.55, 0.38, 0.78}, ecam);

    Mech positioned;
    positioned.kind = MechKind::Slider;
    positioned.motion = BodyMotion::Driven;
    positioned.kinematic = true;
    positioned.axis = {1, 0, 0};
    positioned.limited = true;
    positioned.lower = 0;
    positioned.upper = 160;
    positioned.control = "position";
    positioned.setpoint = "pos_cmd";
    positioned.maxSpeed = 400;
    positioned.maxAccel = 4000;
    positioned.halfExtents = {22, 16, 16};
    addBox(scene, "PosAxis", positioned.halfExtents, {-280, -80, 36}, {0.25, 0.70, 0.78}, positioned);

    Mech velocity;
    velocity.kind = MechKind::Slider;
    velocity.motion = BodyMotion::Driven;
    velocity.kinematic = true;
    velocity.axis = {1, 0, 0};
    velocity.limited = true;
    velocity.lower = 0;
    velocity.upper = 200;
    velocity.control = "velocity";
    velocity.setpoint = "vel_cmd";
    velocity.gain = 1;
    velocity.maxSpeed = 200;
    velocity.maxAccel = 2000;
    velocity.halfExtents = {22, 16, 16};
    addBox(scene, "VelAxis", velocity.halfExtents, {-40, -80, 36}, {0.35, 0.55, 0.85}, velocity);

    Mech post;
    post.kind = MechKind::Collision;
    post.motion = BodyMotion::Anchored;
    post.kinematic = true;
    post.halfExtents = {24, 24, 40};
    addBox(scene, "Post", post.halfExtents, {360, -40, 50}, {0.55, 0.58, 0.62}, post);

    Mech range;
    range.kind = MechKind::DistanceSensor;
    range.axis = {0, 1, 0};
    range.range = 400;
    range.signal = "gap_hit";
    range.valueSignal = "gap";
    range.halfExtents = {8, 8, 8};
    addBox(scene, "Range", range.halfExtents, {360, -180, 50}, {0.95, 0.82, 0.25}, range);

    Mech limit;
    limit.kind = MechKind::Limit;
    limit.other = "GearIn";
    limit.trip = 30;
    limit.signal = "at_limit";
    limit.halfExtents = {12, 12, 18};
    addBox(scene, "Limit", limit.halfExtents, {-280, 110, 40}, {0.75, 0.22, 0.22}, limit);

    Mech cylinder;
    cylinder.kind = MechKind::Cylinder;
    cylinder.axis = {1, 0, 0};
    cylinder.lower = 0;
    cylinder.upper = 160;
    cylinder.maxSpeed = 280;
    cylinder.throttle = 0.75;
    cylinder.cushion = 0.12;
    cylinder.signal = "extend";
    cylinder.halfExtents = {36, 18, 18};
    addBox(scene, "Cylinder", cylinder.halfExtents, {-220, -190, 36}, {0.86, 0.45, 0.18}, cylinder);

    Mech gripper;
    gripper.kind = MechKind::Gripper;
    gripper.graspSignal = "grip";
    gripper.graspRadius = 90;
    gripper.halfExtents = {16, 28, 10};
    addBox(scene, "Gripper", gripper.halfExtents, {40, -190, 110}, {0.72, 0.74, 0.78}, gripper);

    Mech part;
    part.kind = MechKind::Rigid;
    part.motion = BodyMotion::Free;
    part.autoMass = false;
    part.mass = 0.4;
    part.allowSleep = false;
    part.halfExtents = {18, 18, 18};
    addBox(scene, "Part", part.halfExtents, {40, -190, 62}, {0.92, 0.78, 0.22}, part);

    Mech blank = part;
    blank.halfExtents = {14, 14, 14};
    addBox(scene, "Blank", blank.halfExtents, {240, -190, 220}, {0.45, 0.72, 0.42}, blank, false);

    Mech source;
    source.kind = MechKind::Source;
    source.prototype = "Blank";
    source.interval = 0.45;
    source.signal = "feed";
    source.halfExtents = {16, 16, 16};
    addBox(scene, "Source", source.halfExtents, {240, -190, 90}, {0.28, 0.62, 0.36}, source);

    Mech scrap = part;
    scrap.halfExtents = {16, 16, 16};
    addBox(scene, "Scrap", scrap.halfExtents, {420, -190, 28}, {0.62, 0.30, 0.28}, scrap);

    Mech sink;
    sink.kind = MechKind::Sink;
    sink.signal = "collected";
    sink.valueSignal = "collected_n";
    sink.halfExtents = {36, 36, 28};
    addBox(scene, "Sink", sink.halfExtents, {420, -190, 28}, {0.45, 0.18, 0.24}, sink);

    LogicBlock cell;
    cell.name = "cell";
    LogicStep step;
    auto flag = [&](const char* name) {
        LogicAction action;
        action.kind = "set_output";
        action.signal = name;
        step.actions.push_back(action);
    };
    auto number = [&](const char* name, double value) {
        LogicAction action;
        action.kind = "set_value";
        action.signal = name;
        action.speed = value;
        step.actions.push_back(action);
    };
    flag("extend");
    flag("grip");
    flag("feed");
    number("gear_cmd", 70);
    number("pos_cmd", 100);
    number("vel_cmd", 80);
    number("cam_master", 45);
    cell.steps.push_back(std::move(step));
    scene.logicBlocks.push_back(std::move(cell));
    scene.markClean();
}

void buildCommissionLine(Scene& scene) {
    scene.clear();
    scene.name = "commission_line";
    scene.physicsDoc.engine = "builtin";

    Mech floor;
    floor.kind = MechKind::Collision;
    floor.motion = BodyMotion::Anchored;
    floor.kinematic = true;
    floor.halfExtents = {420, 280, 8};
    addBox(scene, "Floor", floor.halfExtents, {40, -40, -8}, {0.12, 0.14, 0.20}, floor);

    Mech belt;
    belt.kind = MechKind::Conveyor;
    belt.motion = BodyMotion::Driven;
    belt.kinematic = true;
    belt.halfExtents = {360, 70, 16};
    belt.direction = {1, 0, 0};
    belt.speed = 200;
    belt.signal = "belt_run";
    addBox(scene, "Conveyor", belt.halfExtents, {40, 0, 16}, {0.18, 0.36, 0.72}, belt);

    Mech part;
    part.kind = MechKind::Rigid;
    part.motion = BodyMotion::Surface;
    part.surfaceRide = true;
    part.kinematic = true;
    part.halfExtents = {24, 24, 18};
    addBox(scene, "Part", part.halfExtents, {80, 0, 46}, {0.93, 0.78, 0.22}, part);

    Mech barrier;
    barrier.kind = MechKind::LightBarrier;
    barrier.axis = {0, 1, 0};
    barrier.range = 220;
    barrier.signal = "photo";
    barrier.halfExtents = {8, 10, 28};
    addBox(scene, "Photoeye", barrier.halfExtents, {80, -130, 55}, {0.95, 0.78, 0.22}, barrier);

    Mech cylinder;
    cylinder.kind = MechKind::Cylinder;
    cylinder.axis = {0, 1, 0};
    cylinder.lower = 0;
    cylinder.upper = 80;
    cylinder.maxSpeed = 400;
    cylinder.throttle = 1;
    cylinder.cushion = 0.05;
    cylinder.signal = "extend";
    cylinder.halfExtents = {18, 36, 16};
    addBox(scene, "Cylinder", cylinder.halfExtents, {80, -200, 48}, {0.86, 0.42, 0.16}, cylinder);

    Mech gripper;
    gripper.kind = MechKind::Gripper;
    gripper.graspSignal = "grip";
    gripper.graspRadius = 80;
    gripper.halfExtents = {16, 22, 10};
    addBox(scene, "Gripper", gripper.halfExtents, {240, -170, 88}, {0.72, 0.76, 0.82}, gripper);

    Mech cup;
    cup.kind = MechKind::Rigid;
    cup.motion = BodyMotion::Free;
    cup.autoMass = false;
    cup.mass = 0.25;
    cup.allowSleep = false;
    cup.halfExtents = {14, 14, 12};
    addBox(scene, "Cup", cup.halfExtents, {240, -170, 58}, {0.35, 0.72, 0.48}, cup);

    auto flag = [](const char* name) {
        Signal signal;
        signal.name = name;
        signal.type = SignalType::Bool;
        return signal;
    };
    scene.signals.push_back(flag("photo"));
    scene.signals.push_back(flag("extend"));
    scene.signals.push_back(flag("belt_run"));
    scene.signals.push_back(flag("grip"));
    scene.signals.push_back(flag("seen"));
    Signal parts;
    parts.name = "parts";
    parts.type = SignalType::Int;
    scene.signals.push_back(parts);
    scene.signals.push_back(flag("plc_T0"));
    scene.signals.push_back(flag("plc_C0"));

    auto tag = [](const char* address, PlcValueKind kind) {
        PlcTag item;
        item.address = address;
        item.kind = kind;
        return item;
    };
    scene.softPlc.enabled = true;
    scene.softPlc.scanSeconds = 0.01;
    scene.softPlc.backend = "softplc";
    scene.softPlc.tags.push_back(tag("I0.0", PlcValueKind::Bool));
    scene.softPlc.tags.push_back(tag("Q0.0", PlcValueKind::Bool));
    scene.softPlc.tags.push_back(tag("Q0.1", PlcValueKind::Bool));
    PlcTag timer = tag("T0", PlcValueKind::Timer);
    timer.preset = 0.02;
    scene.softPlc.tags.push_back(timer);
    scene.softPlc.tags.push_back(tag("M0.0", PlcValueKind::Bool));
    PlcTag counter = tag("C0", PlcValueKind::Counter);
    counter.preset = 1;
    scene.softPlc.tags.push_back(counter);

    auto bind = [](const char* device, const char* address, const char* direction) {
        PlcBinding item;
        item.device = device;
        item.address = address;
        item.direction = direction;
        return item;
    };
    scene.softPlc.bindings.push_back(bind("photo", "I0.0", "in"));
    scene.softPlc.bindings.push_back(bind("extend", "Q0.0", "out"));
    scene.softPlc.bindings.push_back(bind("belt_run", "Q0.1", "out"));
    scene.softPlc.bindings.push_back(bind("seen", "M0.0", "out"));
    scene.softPlc.bindings.push_back(bind("parts", "C0", "out"));

    auto level = [](const char* address, bool normallyOpen) {
        PlcContact contact;
        contact.address = address;
        contact.normallyOpen = normallyOpen;
        return contact;
    };
    auto edge = [](const char* address, const char* which) {
        PlcContact contact;
        contact.address = address;
        contact.edge = which;
        return contact;
    };
    PlcRung arrive;
    arrive.series.push_back(edge("I0.0", "rising"));
    arrive.coil = "M0.0";
    arrive.action = "set";
    PlcRung leave;
    leave.series.push_back(edge("I0.0", "falling"));
    leave.coil = "M0.0";
    leave.action = "reset";
    PlcRung count;
    count.series.push_back(edge("I0.0", "rising"));
    count.coil = "C0";
    count.action = "ctu";
    PlcRung ton;
    ton.series.push_back(level("I0.0", true));
    ton.coil = "T0";
    ton.action = "ton";
    // Extend when the timer and the arrival latch are both on, or when the
    // counter has seen the part and the photoeye is still blocked.
    PlcRung extendRung;
    extendRung.series.push_back(level("T0", true));
    extendRung.series.push_back(level("M0.0", true));
    extendRung.parallel.push_back({level("C0", true), level("I0.0", true)});
    extendRung.coil = "Q0.0";
    extendRung.action = "out";
    PlcRung beltRung;
    beltRung.series.push_back(level("I0.0", false));
    beltRung.series.push_back(level("M0.0", false));
    beltRung.coil = "Q0.1";
    beltRung.action = "out";
    scene.softPlc.rungs.push_back(std::move(arrive));
    scene.softPlc.rungs.push_back(std::move(leave));
    scene.softPlc.rungs.push_back(std::move(count));
    scene.softPlc.rungs.push_back(std::move(ton));
    scene.softPlc.rungs.push_back(std::move(extendRung));
    scene.softPlc.rungs.push_back(std::move(beltRung));

    LogicBlock gripOn;
    gripOn.name = "grip_on";
    gripOn.runSignal = "extend";
    gripOn.inputs.push_back("extend");
    gripOn.outputs.push_back("grip");
    LogicStep step;
    LogicAction action;
    action.kind = "set_output";
    action.signal = "grip";
    step.actions.push_back(action);
    gripOn.steps.push_back(std::move(step));
    scene.logicBlocks.push_back(std::move(gripOn));
    scene.markClean();
}

void buildMcdCompatLine(Scene& scene) {
    scene.clear();
    scene.name = "mcd_compat";
    scene.physicsDoc.engine = "builtin";

    Mech floor;
    floor.kind = MechKind::Collision;
    floor.motion = BodyMotion::Anchored;
    floor.kinematic = true;
    floor.halfExtents = {360, 220, 8};
    addBox(scene, "Floor", floor.halfExtents, {40, 0, -8}, {0.12, 0.14, 0.20}, floor);

    Mech belt;
    belt.kind = MechKind::Conveyor;
    belt.motion = BodyMotion::Driven;
    belt.kinematic = true;
    belt.halfExtents = {320, 70, 16};
    belt.direction = {1, 0, 0};
    belt.speed = 180;
    belt.signal = "belt_run";
    addBox(scene, "Conveyor", belt.halfExtents, {40, 0, 16}, {0.18, 0.36, 0.72}, belt);

    Mech part;
    part.kind = MechKind::Rigid;
    part.motion = BodyMotion::Surface;
    part.surfaceRide = true;
    part.kinematic = true;
    // Bottom sits 4 mm into the belt (belt top is z = 32). A flush box is not
    // a supporting contact, so the part would drop out of the light barrier.
    part.halfExtents = {22, 22, 18};
    addBox(scene, "Product", part.halfExtents, {80, 0, 46}, {0.93, 0.78, 0.22}, part);

    Mech barrier;
    barrier.kind = MechKind::LightBarrier;
    barrier.axis = {0, 1, 0};
    barrier.range = 220;
    barrier.signal = "photo";
    barrier.halfExtents = {8, 10, 24};
    addBox(scene, "Photoeye", barrier.halfExtents, {80, -120, 58}, {0.95, 0.78, 0.22}, barrier);

    Mech cylinder;
    cylinder.kind = MechKind::Cylinder;
    cylinder.axis = {0, 0, -1};
    cylinder.lower = 0;
    cylinder.upper = 70;
    cylinder.maxSpeed = 350;
    cylinder.throttle = 1;
    cylinder.cushion = 0.04;
    cylinder.signal = "extend";
    cylinder.halfExtents = {16, 16, 28};
    addBox(scene, "Cylinder", cylinder.halfExtents, {80, 0, 168}, {0.86, 0.42, 0.16}, cylinder);

    Mech suction;
    suction.kind = MechKind::Gripper;
    suction.graspSignal = "vacuum";
    suction.graspRadius = 36;
    suction.direction = {0, 0, 0};
    suction.halfExtents = {18, 18, 8};
    addBox(scene, "Suction", suction.halfExtents, {80, 0, 120}, {0.45, 0.72, 0.86}, suction);
    if (const auto cup = scene.findByName("Suction")) {
        if (const auto rod = scene.findByName("Cylinder")) {
            scene.reparent(cup->id, rod->id, false);
            cup->local.translation = {0, 0, -40};
        }
    }

    auto flag = [](const char* name) {
        Signal signal;
        signal.name = name;
        signal.type = SignalType::Bool;
        return signal;
    };
    scene.signals.push_back(flag("photo"));
    scene.signals.push_back(flag("extend"));
    scene.signals.push_back(flag("belt_run"));
    scene.signals.push_back(flag("vacuum"));
    scene.signals.push_back(flag("seen"));
    Signal parts;
    parts.name = "parts";
    parts.type = SignalType::Int;
    scene.signals.push_back(parts);
    scene.signals.push_back(flag("plc_T0"));
    scene.signals.push_back(flag("plc_T1"));
    scene.signals.push_back(flag("plc_C0"));

    auto tag = [](const char* address, PlcValueKind kind) {
        PlcTag item;
        item.address = address;
        item.kind = kind;
        return item;
    };
    scene.softPlc.enabled = true;
    scene.softPlc.scanSeconds = 0.01;
    scene.softPlc.backend = "softplc";
    scene.softPlc.tags.push_back(tag("I0.0", PlcValueKind::Bool));
    scene.softPlc.tags.push_back(tag("Q0.0", PlcValueKind::Bool));
    scene.softPlc.tags.push_back(tag("Q0.1", PlcValueKind::Bool));
    scene.softPlc.tags.push_back(tag("Q0.2", PlcValueKind::Bool));
    PlcTag debounce = tag("T0", PlcValueKind::Timer);
    debounce.preset = 0.02;
    scene.softPlc.tags.push_back(debounce);
    PlcTag settle = tag("T1", PlcValueKind::Timer);
    settle.preset = 0.40;
    scene.softPlc.tags.push_back(settle);
    PlcTag liftTimer = tag("T2", PlcValueKind::Timer);
    liftTimer.preset = 0.12;
    scene.softPlc.tags.push_back(liftTimer);
    scene.softPlc.tags.push_back(tag("M0.0", PlcValueKind::Bool));
    scene.softPlc.tags.push_back(tag("M1.0", PlcValueKind::Bool));
    PlcTag counter = tag("C0", PlcValueKind::Counter);
    counter.preset = 1;
    scene.softPlc.tags.push_back(counter);
    scene.softPlc.tags.push_back(tag("气缸_Extending", PlcValueKind::Bool));

    auto bind = [](const char* device, const char* address, const char* direction) {
        PlcBinding item;
        item.device = device;
        item.address = address;
        item.direction = direction;
        return item;
    };
    scene.softPlc.bindings.push_back(bind("photo", "I0.0", "in"));
    scene.softPlc.bindings.push_back(bind("extend", "Q0.0", "out"));
    scene.softPlc.bindings.push_back(bind("belt_run", "Q0.1", "out"));
    scene.softPlc.bindings.push_back(bind("vacuum", "Q0.2", "out"));
    scene.softPlc.bindings.push_back(bind("seen", "M0.0", "out"));
    scene.softPlc.bindings.push_back(bind("parts", "C0", "out"));

    auto level = [](const char* address, bool normallyOpen) {
        PlcContact contact;
        contact.address = address;
        contact.normallyOpen = normallyOpen;
        return contact;
    };
    auto edge = [](const char* address, const char* which) {
        PlcContact contact;
        contact.address = address;
        contact.edge = which;
        return contact;
    };
    PlcRung arrive;
    arrive.series.push_back(edge("I0.0", "rising"));
    arrive.coil = "M0.0";
    arrive.action = "set";
    PlcRung leave;
    leave.series.push_back(edge("I0.0", "falling"));
    leave.coil = "M0.0";
    leave.action = "reset";
    PlcRung count;
    count.series.push_back(edge("I0.0", "rising"));
    count.coil = "C0";
    count.action = "ctu";
    PlcRung ton;
    ton.series.push_back(level("I0.0", true));
    ton.coil = "T0";
    ton.action = "ton";
    PlcRung extendRung;
    extendRung.series.push_back(level("T0", true));
    extendRung.series.push_back(level("M0.0", true));
    extendRung.series.push_back(level("M1.0", false));
    extendRung.coil = "Q0.0";
    extendRung.action = "out";
    PlcRung settleRung;
    settleRung.series.push_back(level("Q0.0", true));
    settleRung.coil = "T1";
    settleRung.action = "ton";
    PlcRung vacuumRung;
    vacuumRung.series.push_back(level("T1", true));
    vacuumRung.coil = "Q0.2";
    vacuumRung.action = "set";
    PlcRung liftTimerRung;
    liftTimerRung.series.push_back(level("Q0.2", true));
    liftTimerRung.coil = "T2";
    liftTimerRung.action = "ton";
    PlcRung liftRung;
    liftRung.series.push_back(level("T2", true));
    liftRung.coil = "M1.0";
    liftRung.action = "set";
    PlcRung beltRung;
    beltRung.series.push_back(level("I0.0", false));
    beltRung.series.push_back(level("M0.0", false));
    beltRung.coil = "Q0.1";
    beltRung.action = "out";
    scene.softPlc.rungs.push_back(std::move(arrive));
    scene.softPlc.rungs.push_back(std::move(leave));
    scene.softPlc.rungs.push_back(std::move(count));
    scene.softPlc.rungs.push_back(std::move(ton));
    scene.softPlc.rungs.push_back(std::move(extendRung));
    scene.softPlc.rungs.push_back(std::move(settleRung));
    scene.softPlc.rungs.push_back(std::move(vacuumRung));
    scene.softPlc.rungs.push_back(std::move(liftTimerRung));
    scene.softPlc.rungs.push_back(std::move(liftRung));
    scene.softPlc.rungs.push_back(std::move(beltRung));

    auto addRow = [&](const char* id, StationKind kind) {
        Station station;
        station.id = id;
        station.name = id;
        station.kind = kind;
        station.resource = id;
        scene.layout.stations.push_back(std::move(station));
    };
    addRow("输送", StationKind::Conveyor);
    addRow("气缸", StationKind::Manual);
    addRow("吸盘", StationKind::Manual);
    PlcCycleRule beltCycle;
    beltCycle.id = "belt_hold";
    beltCycle.name = "锁存";
    beltCycle.resource = "输送";
    beltCycle.stationName = "输送";
    beltCycle.startSignal = "M0.0";
    beltCycle.endSignal = "M0.0";
    beltCycle.endEdge = "falling";
    PlcCycleRule strokeCycle;
    strokeCycle.id = "cyl_extend";
    strokeCycle.name = "伸出";
    strokeCycle.resource = "气缸";
    strokeCycle.stationName = "气缸";
    strokeCycle.startSignal = "气缸_Extending";
    strokeCycle.endSignal = "气缸_Extending";
    strokeCycle.endEdge = "falling";
    PlcCycleRule vacuumCycle;
    vacuumCycle.id = "vacuum_on";
    vacuumCycle.name = "吸取";
    vacuumCycle.resource = "吸盘";
    vacuumCycle.stationName = "吸盘";
    vacuumCycle.startSignal = "Q0.2";
    vacuumCycle.endSignal = "Q0.2";
    vacuumCycle.endEdge = "falling";
    scene.softPlc.cycles.push_back(std::move(beltCycle));
    scene.softPlc.cycles.push_back(std::move(strokeCycle));
    scene.softPlc.cycles.push_back(std::move(vacuumCycle));
    scene.layout.title = "MCD 兼容";
    scene.markClean();
}

void buildLargeField(Scene& scene, int parts, int triangles) {
    scene.clear();
    scene.name = "large_field";
    const int count = std::max(1, parts);
    const std::string meshId = scene.addMesh(makeTriangleGrid(std::max(2, triangles), 40));
    const int side = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(count)))));
    scene.beginEdit();
    for (int i = 0; i < count; ++i) {
        auto node = scene.makeNode(NodeKind::Part, "p" + std::to_string(i));
        node->meshId = meshId;
        node->local.translation = {static_cast<double>(i % side) * 80.0, static_cast<double>(i / side) * 80.0, 0};
        const double tint = 0.15 + static_cast<double>(i % 7) * 0.08;
        node->color = {tint, 0.42, 0.78};
        scene.addChild(scene.root(), node);
    }
    scene.endEdit();
    scene.markClean();
}

}  // namespace sim
