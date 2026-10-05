#include "sim/core/Mech.hpp"
#include "sim/io/OpcUa.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/physics/PhysicsEngine.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using Catch::Approx;

namespace {

std::shared_ptr<sim::SceneNode> addPart(sim::Scene& scene, const std::string& name) {
    auto node = scene.makeNode(sim::NodeKind::Part, name);
    scene.addChild(scene.root(), node);
    return node;
}

void stepFor(sim::IPhysicsEngine& engine, double seconds) {
    const int steps = static_cast<int>(seconds / sim::Simulation::kFixedDt);
    for (int i = 0; i < steps; ++i) {
        engine.step(sim::Simulation::kFixedDt);
    }
}

double hingeDegrees(const sim::SceneNode& node) {
    const sim::Quat q = node.local.rotation.normalized();
    return sim::rad2deg(2.0 * std::atan2(q.z, q.w));
}

sim::Mech drivenHinge(const std::string& axisName) {
    sim::Mech mech;
    mech.kind = sim::MechKind::Hinge;
    mech.motion = sim::BodyMotion::Driven;
    mech.kinematic = true;
    mech.axis = axisName == "z" ? sim::Vec3{0, 0, 1} : sim::Vec3{0, 1, 0};
    mech.limited = true;
    mech.lower = -400;
    mech.upper = 400;
    mech.maxSpeed = 800;
    mech.maxAccel = 20000;
    mech.halfExtents = {40, 8, 8};
    return mech;
}

}  // namespace

TEST_CASE("mcd gear cam and electronic cam copy the master") {
    sim::Scene scene;
    auto master = addPart(scene, "GearIn");
    master->mech = drivenHinge("z");
    master->mech.drive = sim::DriveMode::Position;
    master->mech.target = 90;
    master->local.translation = {0, 0, 40};

    auto gear = addPart(scene, "GearOut");
    gear->mech = drivenHinge("z");
    gear->mech.drive = sim::DriveMode::None;
    gear->mech.couple = "gear";
    gear->mech.master = "GearIn";
    gear->mech.ratio = 2;
    gear->local.translation = {120, 0, 40};

    auto cam = addPart(scene, "CamOut");
    cam->mech = drivenHinge("z");
    cam->mech.couple = "cam";
    cam->mech.master = "GearIn";
    cam->mech.camIn = {0, 90};
    cam->mech.camOut = {0, 30};
    cam->local.translation = {240, 0, 40};

    auto ecam = addPart(scene, "Ecam");
    ecam->mech.kind = sim::MechKind::Slider;
    ecam->mech.motion = sim::BodyMotion::Driven;
    ecam->mech.kinematic = true;
    ecam->mech.axis = {1, 0, 0};
    ecam->mech.limited = false;
    ecam->mech.couple = "ecam";
    ecam->mech.master = "cam_master";
    ecam->mech.camIn = {0, 90};
    ecam->mech.camOut = {0, 30};
    ecam->local.translation = {0, 80, 20};
    scene.upsertSignal("cam_master", sim::SignalType::Real).real = 45;

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    stepFor(*engine, 0.8);
    REQUIRE(hingeDegrees(*master) == Approx(90).margin(2));
    REQUIRE(hingeDegrees(*gear) == Approx(180).margin(3));
    REQUIRE(hingeDegrees(*cam) == Approx(30).margin(2));
    REQUIRE(ecam->local.translation.x == Approx(15).margin(0.5));
}

TEST_CASE("mcd position and velocity controllers follow real signals") {
    sim::Scene scene;
    auto position = addPart(scene, "Pos");
    position->mech.kind = sim::MechKind::Slider;
    position->mech.motion = sim::BodyMotion::Driven;
    position->mech.kinematic = true;
    position->mech.axis = {1, 0, 0};
    position->mech.limited = true;
    position->mech.lower = 0;
    position->mech.upper = 200;
    position->mech.control = "position";
    position->mech.setpoint = "pos_cmd";
    position->mech.maxSpeed = 500;
    position->mech.maxAccel = 8000;
    position->local.translation = {0, 0, 30};
    scene.upsertSignal("pos_cmd", sim::SignalType::Real).real = 40;

    auto velocity = addPart(scene, "Vel");
    velocity->mech.kind = sim::MechKind::Slider;
    velocity->mech.motion = sim::BodyMotion::Driven;
    velocity->mech.kinematic = true;
    velocity->mech.axis = {1, 0, 0};
    velocity->mech.limited = true;
    velocity->mech.lower = 0;
    velocity->mech.upper = 400;
    velocity->mech.control = "velocity";
    velocity->mech.setpoint = "vel_cmd";
    velocity->mech.gain = 2;
    velocity->mech.maxSpeed = 200;
    velocity->mech.maxAccel = 20000;
    velocity->local.translation = {0, 40, 30};
    scene.upsertSignal("vel_cmd", sim::SignalType::Real).real = 30;

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    stepFor(*engine, 0.5);
    REQUIRE(position->local.translation.x == Approx(40).margin(1));
    REQUIRE(velocity->local.translation.x == Approx(30).margin(2));
}

TEST_CASE("mcd distance sensor reports the hit and a limit switch follows the joint") {
    sim::Scene scene;
    auto wall = addPart(scene, "Wall");
    wall->mech.kind = sim::MechKind::Collision;
    wall->mech.halfExtents = {20, 20, 20};
    wall->local.translation = {180, 0, 0};

    auto sensor = addPart(scene, "Range");
    sensor->mech.kind = sim::MechKind::DistanceSensor;
    sensor->mech.axis = {1, 0, 0};
    sensor->mech.range = 400;
    sensor->mech.signal = "seen";
    sensor->mech.valueSignal = "gap";

    auto master = addPart(scene, "Slide");
    master->mech.kind = sim::MechKind::Slider;
    master->mech.axis = {1, 0, 0};
    master->mech.drive = sim::DriveMode::Position;
    master->mech.target = 50;
    master->mech.maxSpeed = 400;
    master->mech.maxAccel = 8000;
    master->mech.limited = true;
    master->mech.lower = 0;
    master->mech.upper = 80;
    master->local.translation = {0, 80, 0};

    auto limit = addPart(scene, "Limit");
    limit->mech.kind = sim::MechKind::Limit;
    limit->mech.other = "Slide";
    limit->mech.trip = 20;
    limit->mech.signal = "at_limit";

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    engine->step(sim::Simulation::kFixedDt);
    const sim::Signal* gap = scene.findSignal("gap");
    REQUIRE(gap);
    REQUIRE(gap->real == Approx(160).margin(2));
    REQUIRE(scene.findSignal("seen")->flag);
    REQUIRE_FALSE(scene.findSignal("at_limit")->flag);
    stepFor(*engine, 0.4);
    REQUIRE(scene.findSignal("at_limit")->flag);
    REQUIRE(master->local.translation.x == Approx(50).margin(2));
}

TEST_CASE("mcd cylinder throttle and cushion do not jump onto the stop") {
    sim::Scene scene;
    auto fast = addPart(scene, "Fast");
    fast->mech.kind = sim::MechKind::Cylinder;
    fast->mech.axis = {1, 0, 0};
    fast->mech.lower = 0;
    fast->mech.upper = 100;
    fast->mech.maxSpeed = 5000;
    fast->mech.throttle = 1;
    fast->mech.cushion = 0.2;
    fast->mech.signal = "extend";
    fast->local.translation = {0, 0, 0};
    scene.upsertSignal("extend", sim::SignalType::Bool).flag = true;

    auto slow = addPart(scene, "Slow");
    slow->mech.kind = sim::MechKind::Cylinder;
    slow->mech.axis = {1, 0, 0};
    slow->mech.lower = 0;
    slow->mech.upper = 400;
    slow->mech.maxSpeed = 200;
    slow->mech.throttle = 0.25;
    slow->mech.cushion = 0;
    slow->mech.signal = "extend";
    slow->local.translation = {0, 40, 0};

    auto open = addPart(scene, "Open");
    open->mech = slow->mech;
    open->mech.throttle = 1;
    open->local.translation = {0, 80, 0};

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    engine->step(0.5);
    REQUIRE(fast->mech.command > 90);
    REQUIRE(fast->mech.command < 99.5);
    REQUIRE(slow->mech.command == Approx(25).margin(0.2));
    REQUIRE(open->mech.command == Approx(100).margin(0.2));
    REQUIRE(fast->local.translation.x == Approx(fast->mech.command).margin(0.1));
}

TEST_CASE("mcd gripper lifts a surface part off the conveyor", "[devices]") {
    sim::Scene scene;
    auto belt = addPart(scene, "Belt");
    belt->mech.kind = sim::MechKind::Conveyor;
    belt->mech.motion = sim::BodyMotion::Driven;
    belt->mech.kinematic = true;
    belt->mech.halfExtents = {80, 40, 16};
    belt->mech.direction = {1, 0, 0};
    belt->mech.speed = 0;
    belt->local.translation = {0, 0, 16};

    auto part = addPart(scene, "Part");
    part->mech.kind = sim::MechKind::Rigid;
    part->mech.motion = sim::BodyMotion::Surface;
    part->mech.surfaceRide = true;
    part->mech.kinematic = true;
    part->mech.halfExtents = {18, 18, 18};
    part->local.translation = {0, 0, 46};

    auto gripper = addPart(scene, "Gripper");
    gripper->mech.kind = sim::MechKind::Gripper;
    gripper->mech.graspSignal = "grip";
    gripper->mech.graspRadius = 40;
    gripper->local.translation = {0, 0, 70};
    scene.upsertSignal("grip", sim::SignalType::Bool).flag = true;

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    stepFor(*engine, 0.1);
    REQUIRE(gripper->mech.held == part->id);
    REQUIRE(part->mech.motion == sim::BodyMotion::Free);
    const double heldZ = part->local.translation.z;
    gripper->local.translation.z = 160;
    engine->step(sim::Simulation::kFixedDt);
    REQUIRE(part->local.translation.z > heldZ + 70);
}

TEST_CASE("mcd gripper attaches the nearest body and releases it") {
    sim::Scene scene;
    auto gripper = addPart(scene, "Gripper");
    gripper->mech.kind = sim::MechKind::Gripper;
    gripper->mech.graspSignal = "grip";
    gripper->mech.graspRadius = 80;
    gripper->local.translation = {0, 0, 120};

    auto part = addPart(scene, "Part");
    part->mech.kind = sim::MechKind::Rigid;
    part->mech.motion = sim::BodyMotion::Free;
    part->mech.autoMass = false;
    part->mech.mass = 0.3;
    part->mech.allowSleep = false;
    part->mech.halfExtents = {15, 15, 15};
    part->local.translation = {0, 0, 70};
    scene.upsertSignal("grip", sim::SignalType::Bool).flag = true;

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    stepFor(*engine, 0.2);
    REQUIRE(gripper->mech.held == part->id);
    REQUIRE(part->local.translation.z == Approx(70).margin(1));
    scene.findSignal("grip")->flag = false;
    const double heldZ = part->local.translation.z;
    stepFor(*engine, 0.15);
    REQUIRE(gripper->mech.held.empty());
    REQUIRE(part->local.translation.z < heldZ - 5);
}

TEST_CASE("mcd source waits for its signal and the sink counts a removal") {
    sim::Scene scene;
    auto blank = addPart(scene, "Blank");
    blank->mech.kind = sim::MechKind::Rigid;
    blank->mech.motion = sim::BodyMotion::Free;
    blank->mech.halfExtents = {10, 10, 10};
    blank->local.translation = {0, 0, 400};
    blank->visible = false;

    auto source = addPart(scene, "Source");
    source->mech.kind = sim::MechKind::Source;
    source->mech.prototype = "Blank";
    source->mech.interval = 0.2;
    source->mech.signal = "feed";
    source->local.translation = {0, 0, 80};
    scene.upsertSignal("feed", sim::SignalType::Bool).flag = false;

    auto scrap = addPart(scene, "Scrap");
    scrap->mech.kind = sim::MechKind::Rigid;
    scrap->mech.motion = sim::BodyMotion::Free;
    scrap->mech.halfExtents = {12, 12, 12};
    scrap->local.translation = {200, 0, 20};

    auto sink = addPart(scene, "Sink");
    sink->mech.kind = sim::MechKind::Sink;
    sink->mech.signal = "collected";
    sink->mech.valueSignal = "collected_n";
    sink->mech.halfExtents = {20, 20, 20};
    sink->local.translation = {200, 0, 20};

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    const auto before = scene.root()->children.size();
    stepFor(*engine, 0.5);
    REQUIRE(scene.root()->children.size() == before - 1);
    REQUIRE(scene.findByName("Scrap") == nullptr);
    REQUIRE(scene.findSignal("collected")->flag);
    REQUIRE(scene.findSignal("collected_n")->integer == 1);
    scene.findSignal("feed")->flag = true;
    stepFor(*engine, 0.25);
    REQUIRE(scene.root()->children.size() == before);
}

TEST_CASE("mcd logic and opc ua signals drive a cylinder and a position axis") {
    sim::Scene scene;
    auto cylinder = addPart(scene, "Cylinder");
    cylinder->mech.kind = sim::MechKind::Cylinder;
    cylinder->mech.axis = {1, 0, 0};
    cylinder->mech.lower = 0;
    cylinder->mech.upper = 80;
    cylinder->mech.maxSpeed = 200;
    cylinder->mech.throttle = 1;
    cylinder->mech.cushion = 0;
    cylinder->mech.signal = "extend";

    auto axis = addPart(scene, "Axis");
    axis->mech.kind = sim::MechKind::Slider;
    axis->mech.motion = sim::BodyMotion::Driven;
    axis->mech.kinematic = true;
    axis->mech.axis = {0, 1, 0};
    axis->mech.limited = true;
    axis->mech.lower = 0;
    axis->mech.upper = 100;
    axis->mech.control = "position";
    axis->mech.setpoint = "pos_cmd";
    axis->mech.maxSpeed = 400;
    axis->mech.maxAccel = 8000;
    axis->local.translation = {0, 0, 40};

    sim::LogicBlock block;
    block.name = "cell";
    sim::LogicAction extend;
    extend.kind = "set_output";
    extend.signal = "extend";
    sim::LogicAction command;
    command.kind = "set_value";
    command.signal = "pos_cmd";
    command.speed = 25;
    sim::LogicStep step;
    step.actions = {extend, command};
    block.steps.push_back(step);
    scene.logicBlocks.push_back(block);

    sim::LogicMachine logic;
    std::string error;
    REQUIRE(logic.start(scene, "cell", error));
    logic.step(scene, 0.016);
    REQUIRE(scene.findSignal("extend")->flag);
    REQUIRE(scene.findSignal("pos_cmd")->real == Approx(25));

    sim::publishPlcBool(scene, "extend", true);
    sim::publishPlcNumber(scene, "pos_cmd", 55);

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    REQUIRE(engine->build(scene, error));
    stepFor(*engine, 0.5);
    REQUIRE(cylinder->mech.command == Approx(80).margin(1));
    REQUIRE(axis->local.translation.y == Approx(55).margin(1));
}

TEST_CASE("mcd mechanism file keeps couple controller and gripper fields") {
    sim::Mech mech;
    mech.kind = sim::MechKind::Gripper;
    mech.couple = "ecam";
    mech.master = "cam_master";
    mech.ratio = 1.5;
    mech.camIn = {0, 10, 20};
    mech.camOut = {0, 4, 9};
    mech.control = "velocity";
    mech.setpoint = "speed_cmd";
    mech.gain = 3;
    mech.throttle = 0.4;
    mech.cushion = 0.15;
    mech.command = 12;
    mech.trip = 33;
    mech.graspSignal = "grip";
    mech.graspRadius = 60;
    mech.held = "n4";
    sim::Mech loaded;
    sim::mechFromJson(sim::mechToJson(mech), loaded);
    REQUIRE(loaded.kind == sim::MechKind::Gripper);
    REQUIRE(loaded.couple == "ecam");
    REQUIRE(loaded.master == "cam_master");
    REQUIRE(loaded.ratio == Approx(1.5));
    REQUIRE(loaded.camIn.size() == 3);
    REQUIRE(loaded.camOut[2] == Approx(9));
    REQUIRE(loaded.control == "velocity");
    REQUIRE(loaded.setpoint == "speed_cmd");
    REQUIRE(loaded.gain == Approx(3));
    REQUIRE(loaded.throttle == Approx(0.4));
    REQUIRE(loaded.cushion == Approx(0.15));
    REQUIRE(loaded.command == Approx(12));
    REQUIRE(loaded.trip == Approx(33));
    REQUIRE(loaded.graspSignal == "grip");
    REQUIRE(loaded.graspRadius == Approx(60));
    REQUIRE(loaded.held == "n4");
}
