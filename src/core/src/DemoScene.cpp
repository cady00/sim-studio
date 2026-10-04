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
    addOp("inspect", "检查", "branch", OpKind::Task, 0, "st_inspect", 6, "", "");
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
