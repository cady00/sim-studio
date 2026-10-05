#include "sim/core/DemoScene.hpp"
#include "sim/core/Plc.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/motion/SoftPlc.hpp"
#include "sim/physics/PhysicsEngine.hpp"

#include <nlohmann/json.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>

using Catch::Approx;

namespace {

sim::PlcTag* addTag(sim::SoftPlcDocument& doc, const char* address, sim::PlcValueKind kind) {
    sim::PlcTag tag;
    tag.address = address;
    tag.kind = kind;
    doc.tags.push_back(std::move(tag));
    return &doc.tags.back();
}

sim::PlcContact levelContact(const char* address, bool normallyOpen) {
    sim::PlcContact contact;
    contact.address = address;
    contact.normallyOpen = normallyOpen;
    return contact;
}

void bind(sim::SoftPlcDocument& doc, const char* device, const char* address, const char* direction) {
    sim::PlcBinding binding;
    binding.device = device;
    binding.address = address;
    binding.direction = direction;
    doc.bindings.push_back(std::move(binding));
}

}  // namespace

TEST_CASE("soft plc scans bool int real and timer on the simulation clock") {
    sim::Scene scene;
    sim::SoftPlcDocument& doc = scene.softPlc;
    doc.enabled = true;
    doc.scanSeconds = 0.01;
    doc.backend = "softplc";
    addTag(doc, "I0.0", sim::PlcValueKind::Bool);
    addTag(doc, "Q0.0", sim::PlcValueKind::Bool);
    addTag(doc, "Q0.1", sim::PlcValueKind::Bool);
    sim::PlcTag* timer = addTag(doc, "T0", sim::PlcValueKind::Timer);
    timer->preset = 0.02;
    addTag(doc, "D0", sim::PlcValueKind::Int);
    addTag(doc, "D1", sim::PlcValueKind::Real);
    bind(doc, "photo", "I0.0", "in");
    bind(doc, "extend", "Q0.0", "out");
    bind(doc, "belt_run", "Q0.1", "out");
    bind(doc, "count", "D0", "out");
    bind(doc, "gap", "D1", "out");

    sim::PlcRung ton;
    ton.series.push_back(levelContact("I0.0", true));
    ton.coil = "T0";
    ton.action = "ton";
    sim::PlcRung extend;
    extend.series.push_back(levelContact("T0", true));
    extend.coil = "Q0.0";
    extend.action = "out";
    sim::PlcRung belt;
    belt.series.push_back(levelContact("I0.0", false));
    belt.coil = "Q0.1";
    belt.action = "out";
    sim::PlcRung moveInt;
    moveInt.coil = "D0";
    moveInt.action = "move";
    moveInt.literal = 42;
    sim::PlcRung moveReal;
    moveReal.series.push_back(levelContact("Q0.0", true));
    moveReal.coil = "D1";
    moveReal.action = "move";
    moveReal.literal = 12.5;
    doc.rungs = {ton, extend, belt, moveInt, moveReal};
    scene.upsertSignal("photo", sim::SignalType::Bool).flag = false;

    sim::SoftPlcRuntime runtime;
    std::string error;
    REQUIRE(runtime.start(doc, error));
    runtime.scan(scene, 0.0, 0.004);
    runtime.scan(scene, 0.0, 0.004);
    REQUIRE(runtime.scans() == 0);
    runtime.scan(scene, 0.0, 0.004);
    REQUIRE(runtime.scans() == 1);
    REQUIRE(scene.findSignal("belt_run")->flag);
    REQUIRE_FALSE(scene.findSignal("extend")->flag);
    REQUIRE(scene.findSignal("count")->integer == 42);
    REQUIRE(scene.findSignal("gap")->real == 0);

    scene.findSignal("photo")->flag = true;
    runtime.scan(scene, 0.01, 0.01);
    REQUIRE(sim::findPlcTag(doc, "T0")->elapsed == Approx(0.01));
    REQUIRE_FALSE(sim::findPlcTag(doc, "T0")->done);
    REQUIRE_FALSE(scene.findSignal("extend")->flag);
    REQUIRE_FALSE(scene.findSignal("belt_run")->flag);

    runtime.scan(scene, 0.02, 0.01);
    REQUIRE(sim::findPlcTag(doc, "T0")->done);
    REQUIRE(scene.findSignal("extend")->flag);
    REQUIRE(scene.findSignal("plc_T0")->flag);
    REQUIRE(scene.findSignal("gap")->real == Approx(12.5));
    REQUIRE_FALSE(runtime.log().empty());

    sim::findPlcTag(doc, "Q0.0")->forced = true;
    sim::findPlcTag(doc, "Q0.0")->forceFlag = false;
    runtime.scan(scene, 0.03, 0.01);
    REQUIRE_FALSE(scene.findSignal("extend")->flag);
    REQUIRE(sim::findPlcTag(doc, "Q0.0")->flag);

    runtime.scan(scene, 0.0, 1.0);
    REQUIRE(runtime.scans() <= 8 + 4);
}

TEST_CASE("plcsim advanced adapter refuses to start and does not scan") {
    sim::PlcSimAdvancedBackend backend;
    REQUIRE_FALSE(backend.available());
    sim::SoftPlcDocument doc;
    doc.backend = "plcsim";
    doc.enabled = true;
    std::string error;
    REQUIRE_FALSE(backend.start(doc, error));
    REQUIRE(error == "PLCSIM Advanced Simulation Runtime is not implemented");
    sim::Scene scene;
    scene.softPlc = doc;
    scene.upsertSignal("photo", sim::SignalType::Bool).flag = true;
    backend.scan(scene, 0, 0.1);
    REQUIRE(scene.findSignal("photo")->flag);
}

TEST_CASE("soft plc opc exchange is optional and logged when disconnected") {
    sim::Scene scene;
    scene.softPlc.enabled = true;
    scene.softPlc.scanSeconds = 0.01;
    addTag(scene.softPlc, "I0.0", sim::PlcValueKind::Bool);
    addTag(scene.softPlc, "Q0.0", sim::PlcValueKind::Bool);
    sim::PlcBinding input;
    input.address = "I0.0";
    input.direction = "in";
    input.opcNode = "10001";
    sim::PlcBinding output;
    output.address = "Q0.0";
    output.direction = "out";
    output.opcNode = "10002";
    scene.softPlc.bindings = {input, output};
    sim::PlcRung rung;
    rung.series.push_back(levelContact("I0.0", true));
    rung.coil = "Q0.0";
    rung.action = "out";
    scene.softPlc.rungs.push_back(rung);

    int reads = 0;
    int writes = 0;
    sim::SoftPlcRuntime runtime;
    runtime.setOpcExchange([&](const std::string& node, bool write, bool& value, std::string& error) {
        error.clear();
        if (write) {
            ++writes;
            REQUIRE(node == "10002");
            REQUIRE(value);
            return true;
        }
        ++reads;
        REQUIRE(node == "10001");
        value = true;
        return true;
    });
    std::string error;
    REQUIRE(runtime.start(scene.softPlc, error));
    runtime.scan(scene, 0.0, 0.01);
    REQUIRE(reads == 1);
    REQUIRE(writes == 1);
    REQUIRE(sim::findPlcTag(scene.softPlc, "Q0.0")->flag);

    sim::SoftPlcRuntime offline;
    REQUIRE(offline.start(scene.softPlc, error));
    sim::findPlcTag(scene.softPlc, "I0.0")->flag = false;
    sim::findPlcTag(scene.softPlc, "Q0.0")->flag = false;
    offline.scan(scene, 0.0, 0.01);
    bool mentioned = false;
    for (const std::string& line : offline.log()) {
        if (line.find("is not connected") != std::string::npos) {
            mentioned = true;
        }
    }
    REQUIRE(mentioned);
}

TEST_CASE("soft plc document round trips through the project file") {
    sim::Scene scene;
    auto root = scene.makeNode(sim::NodeKind::Assembly, "Cell");
    scene.addChild(scene.root(), root);
    scene.softPlc.enabled = true;
    scene.softPlc.scanSeconds = 0.02;
    scene.softPlc.backend = "softplc";
    sim::PlcTag* timer = addTag(scene.softPlc, "T0", sim::PlcValueKind::Timer);
    timer->preset = 0.25;
    timer->forced = true;
    timer->forceFlag = true;
    bind(scene.softPlc, "photo", "I0.0", "in");
    scene.softPlc.bindings.back().opcNode = "10001";
    sim::PlcRung rung;
    rung.series.push_back(levelContact("I0.0", false));
    rung.coil = "T0";
    rung.action = "ton";
    scene.softPlc.rungs.push_back(rung);

    const auto path = std::filesystem::temp_directory_path() / "simstudio_softplc.simproj";
    const sim::ProjectResult saved = sim::ProjectStore::save(scene, path.string());
    REQUIRE(saved.ok);
    sim::Scene loaded;
    const sim::ProjectResult opened = sim::ProjectStore::load(path.string(), loaded);
    REQUIRE(opened.ok);
    REQUIRE(loaded.softPlc.enabled);
    REQUIRE(loaded.softPlc.scanSeconds == Approx(0.02));
    REQUIRE(loaded.softPlc.backend == "softplc");
    REQUIRE(loaded.softPlc.tags.size() == 1);
    REQUIRE(loaded.softPlc.tags[0].preset == Approx(0.25));
    REQUIRE(loaded.softPlc.tags[0].forced);
    REQUIRE(loaded.softPlc.bindings.size() == 1);
    REQUIRE(loaded.softPlc.bindings[0].opcNode == "10001");
    REQUIRE(loaded.softPlc.rungs.size() == 1);
    REQUIRE_FALSE(loaded.softPlc.rungs[0].series[0].normallyOpen);

    sim::SoftPlcDocument rejected;
    std::string error;
    const nlohmann::json bad = {{"backend", "vendor"}, {"tags", nlohmann::json::array()}};
    REQUIRE_FALSE(sim::softPlcFromJson(bad, rejected, error));
    REQUIRE(error == "unknown plc backend");
}

TEST_CASE("conveyor run signal stops the belt and an empty signal does not") {
    auto ride = [](bool named, bool run) {
        sim::Scene scene;
        scene.physicsDoc.engine = "builtin";
        auto belt = scene.makeNode(sim::NodeKind::Part, "Conveyor");
        belt->mech.kind = sim::MechKind::Conveyor;
        belt->mech.motion = sim::BodyMotion::Driven;
        belt->mech.kinematic = true;
        belt->mech.halfExtents = {400, 80, 16};
        belt->mech.direction = {1, 0, 0};
        belt->mech.speed = 200;
        if (named) {
            belt->mech.signal = "belt_run";
        }
        belt->local.translation = {0, 0, 16};
        belt->meshId = scene.addMesh(sim::makeBoxMesh(belt->mech.halfExtents));
        scene.addChild(scene.root(), belt);
        auto part = scene.makeNode(sim::NodeKind::Part, "Part");
        part->mech.kind = sim::MechKind::Rigid;
        part->mech.motion = sim::BodyMotion::Surface;
        part->mech.surfaceRide = true;
        part->mech.kinematic = true;
        part->mech.halfExtents = {20, 20, 16};
        part->local.translation = {0, 0, 44};
        part->meshId = scene.addMesh(sim::makeBoxMesh(part->mech.halfExtents));
        scene.addChild(scene.root(), part);
        if (named) {
            scene.upsertSignal("belt_run", sim::SignalType::Bool).flag = run;
        }
        auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
        std::string error;
        REQUIRE(engine->build(scene, error));
        engine->step(0.1);
        return part->local.translation.x;
    };
    REQUIRE(ride(true, false) == Approx(0).margin(0.2));
    REQUIRE(ride(true, true) > 10);
    REQUIRE(ride(false, false) > 10);
}

TEST_CASE("commission line runs the soft plc ahead of the logic block") {
    sim::Scene scene;
    sim::buildCommissionLine(scene);
    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    sim::Simulation simulation(scene, *engine);
    sim::SoftPlcRuntime plc;
    sim::LogicMachine logic;
    std::string error;
    simulation.setHooks([&] { REQUIRE(plc.start(scene.softPlc, error)); },
                        [&](double dt) {
                            plc.scan(scene, simulation.time(), dt);
                            logic.step(scene, dt);
                        },
                        [] {});
    simulation.play();
    REQUIRE(simulation.state() == sim::SimState::Playing);
    simulation.tick(0.5);
    const sim::Signal* photo = scene.findSignal("photo");
    const sim::Signal* extend = scene.findSignal("extend");
    const sim::Signal* belt = scene.findSignal("belt_run");
    const sim::Signal* grip = scene.findSignal("grip");
    REQUIRE(photo != nullptr);
    REQUIRE(photo->flag);
    REQUIRE(extend != nullptr);
    REQUIRE(extend->flag);
    REQUIRE(belt != nullptr);
    REQUIRE_FALSE(belt->flag);
    REQUIRE(grip != nullptr);
    REQUIRE(grip->flag);
    const auto cylinder = scene.findByName("Cylinder");
    REQUIRE(cylinder);
    REQUIRE(cylinder->mech.command > 40);
    const auto gripper = scene.findByName("Gripper");
    REQUIRE(gripper);
    REQUIRE_FALSE(gripper->mech.held.empty());
    REQUIRE_FALSE(plc.log().empty());
    REQUIRE(plc.scans() > 8);
    const sim::PlcTag* counter = sim::findPlcTag(scene.softPlc, "C0");
    const sim::PlcTag* latch = sim::findPlcTag(scene.softPlc, "M0.0");
    REQUIRE(counter != nullptr);
    REQUIRE(counter->integer == 1);
    REQUIRE(counter->done);
    REQUIRE(latch != nullptr);
    REQUIRE(latch->flag);
    REQUIRE(scene.findSignal("seen")->flag);
    REQUIRE(scene.findSignal("parts")->integer == 1);
}

TEST_CASE("soft plc or branches edges and count up") {
    sim::Scene scene;
    sim::SoftPlcDocument& doc = scene.softPlc;
    doc.enabled = true;
    doc.scanSeconds = 0.01;
    doc.backend = "softplc";
    addTag(doc, "I0.0", sim::PlcValueKind::Bool);
    addTag(doc, "I0.1", sim::PlcValueKind::Bool);
    addTag(doc, "I0.2", sim::PlcValueKind::Bool);
    addTag(doc, "I0.3", sim::PlcValueKind::Bool);
    addTag(doc, "Q0.0", sim::PlcValueKind::Bool);
    addTag(doc, "M0.0", sim::PlcValueKind::Bool);
    sim::PlcTag* counter = addTag(doc, "C0", sim::PlcValueKind::Counter);
    counter->preset = 3;
    bind(doc, "extend", "Q0.0", "out");
    bind(doc, "parts", "C0", "out");

    sim::PlcRung gate;
    gate.series.push_back(levelContact("I0.0", true));
    gate.series.push_back(levelContact("I0.1", true));
    sim::PlcContact side;
    side.address = "I0.2";
    gate.parallel.push_back({side});
    gate.coil = "Q0.0";
    gate.action = "out";

    sim::PlcContact rising;
    rising.address = "I0.0";
    rising.edge = "rising";
    sim::PlcContact falling;
    falling.address = "I0.0";
    falling.edge = "falling";

    sim::PlcRung arrive;
    arrive.series.push_back(rising);
    arrive.coil = "M0.0";
    arrive.action = "set";
    sim::PlcRung leave;
    leave.series.push_back(falling);
    leave.coil = "M0.0";
    leave.action = "reset";
    sim::PlcRung count;
    count.series.push_back(rising);
    count.coil = "C0";
    count.action = "ctu";
    sim::PlcRung clear;
    clear.series.push_back(levelContact("I0.3", true));
    clear.coil = "C0";
    clear.action = "ctr";
    doc.rungs = {gate, arrive, leave, count, clear};

    sim::SoftPlcRuntime runtime;
    std::string error;
    REQUIRE(runtime.start(doc, error));
    auto setPhoto = [&](bool on) {
        sim::findPlcTag(doc, "I0.0")->flag = on;
        runtime.scan(scene, 0.0, 0.01);
    };

    setPhoto(false);
    REQUIRE_FALSE(scene.findSignal("extend")->flag);
    REQUIRE(counter->integer == 0);

    sim::findPlcTag(doc, "I0.2")->flag = true;
    runtime.scan(scene, 0.0, 0.01);
    REQUIRE(scene.findSignal("extend")->flag);

    sim::findPlcTag(doc, "I0.2")->flag = false;
    sim::findPlcTag(doc, "I0.1")->flag = true;
    setPhoto(true);
    REQUIRE(scene.findSignal("extend")->flag);
    REQUIRE(sim::findPlcTag(doc, "M0.0")->flag);
    REQUIRE(counter->integer == 1);
    REQUIRE_FALSE(counter->done);

    runtime.scan(scene, 0.0, 0.01);
    REQUIRE(counter->integer == 1);

    setPhoto(false);
    REQUIRE_FALSE(sim::findPlcTag(doc, "M0.0")->flag);
    REQUIRE(counter->integer == 1);
    REQUIRE_FALSE(scene.findSignal("extend")->flag);

    setPhoto(true);
    setPhoto(false);
    setPhoto(true);
    REQUIRE(counter->integer == 3);
    REQUIRE(counter->done);
    REQUIRE(scene.findSignal("parts")->integer == 3);
    REQUIRE(scene.findSignal("plc_C0")->flag);

    sim::findPlcTag(doc, "I0.3")->flag = true;
    runtime.scan(scene, 0.0, 0.01);
    REQUIRE(counter->integer == 0);
    REQUIRE_FALSE(counter->done);

    const nlohmann::json json = sim::softPlcToJson(doc);
    sim::SoftPlcDocument loaded;
    REQUIRE(sim::softPlcFromJson(json, loaded, error));
    REQUIRE(loaded.rungs[0].parallel.size() == 1);
    REQUIRE(loaded.rungs[1].series[0].edge == "rising");
    REQUIRE(loaded.rungs[2].series[0].edge == "falling");
    REQUIRE(loaded.tags.back().kind == sim::PlcValueKind::Counter);

    sim::SoftPlcDocument rejected;
    nlohmann::json badEdge;
    nlohmann::json badRung;
    badRung["coil"] = "Q0.0";
    badRung["series"] = nlohmann::json::array({nlohmann::json{{"address", "I0.0"}, {"edge", "pulse"}}});
    badEdge["rungs"] = nlohmann::json::array({badRung});
    REQUIRE_FALSE(sim::softPlcFromJson(badEdge, rejected, error));
    REQUIRE(error == "unknown plc contact edge");
}

TEST_CASE("soft plc ctd counts down and tof holds after power drops") {
    sim::Scene scene;
    sim::SoftPlcDocument& doc = scene.softPlc;
    doc.enabled = true;
    doc.scanSeconds = 0.01;
    addTag(doc, "I0.0", sim::PlcValueKind::Bool);
    addTag(doc, "C0", sim::PlcValueKind::Counter);
    addTag(doc, "T0", sim::PlcValueKind::Timer);
    sim::PlcTag* counter = sim::findPlcTag(doc, "C0");
    sim::PlcTag* timer = sim::findPlcTag(doc, "T0");
    REQUIRE(counter != nullptr);
    REQUIRE(timer != nullptr);
    counter->integer = 2;
    timer->preset = 0.03;

    sim::PlcRung down;
    down.series.push_back(levelContact("I0.0", true));
    down.coil = "C0";
    down.action = "ctd";
    sim::PlcRung off;
    off.series.push_back(levelContact("I0.0", true));
    off.coil = "T0";
    off.action = "tof";
    doc.rungs = {down, off};

    sim::SoftPlcRuntime runtime;
    std::string error;
    REQUIRE(runtime.start(doc, error));
    auto pulse = [&](bool on) {
        sim::findPlcTag(doc, "I0.0")->flag = on;
        runtime.scan(scene, 0.0, 0.01);
    };

    pulse(false);
    REQUIRE(counter->integer == 2);
    REQUIRE_FALSE(counter->done);
    REQUIRE_FALSE(timer->done);
    REQUIRE(timer->elapsed == Approx(0.0));

    pulse(true);
    REQUIRE(counter->integer == 1);
    REQUIRE_FALSE(counter->done);
    REQUIRE(timer->done);
    REQUIRE(timer->elapsed == Approx(0.0));

    pulse(true);
    REQUIRE(counter->integer == 1);

    pulse(false);
    REQUIRE(counter->integer == 1);
    REQUIRE(timer->done);
    REQUIRE(timer->elapsed == Approx(0.01));

    pulse(true);
    REQUIRE(counter->integer == 0);
    REQUIRE(counter->done);
    REQUIRE(timer->done);
    REQUIRE(timer->elapsed == Approx(0.0));

    pulse(false);
    pulse(false);
    REQUIRE(timer->done);
    REQUIRE(timer->elapsed == Approx(0.02));
    pulse(false);
    REQUIRE_FALSE(timer->done);
    REQUIRE(timer->flag == false);

    pulse(true);
    REQUIRE(counter->integer == -1);
    REQUIRE(counter->done);
    REQUIRE(timer->done);

    const nlohmann::json json = sim::softPlcToJson(doc);
    sim::SoftPlcDocument loaded;
    REQUIRE(sim::softPlcFromJson(json, loaded, error));
    REQUIRE(loaded.rungs[0].action == "ctd");
    REQUIRE(loaded.rungs[1].action == "tof");
}

TEST_CASE("soft plc or sits inside an and branch", "[plc]") {
    sim::Scene scene;
    sim::SoftPlcRuntime plc;
    sim::SoftPlcDocument& doc = scene.softPlc;
    doc.enabled = true;
    doc.scanSeconds = 0.01;
    auto tag = [&](const char* address) {
        sim::PlcTag item;
        item.address = address;
        item.kind = sim::PlcValueKind::Bool;
        doc.tags.push_back(item);
    };
    tag("I0.0");
    tag("I0.1");
    tag("I0.2");
    tag("Q0.0");
    sim::PlcContact gate = levelContact("I0.0", true);
    sim::PlcContact either = levelContact("I0.1", true);
    either.alternatives.push_back(levelContact("I0.2", true));
    sim::PlcRung rung;
    rung.series.push_back(gate);
    rung.series.push_back(either);
    rung.coil = "Q0.0";
    rung.action = "out";
    doc.rungs.push_back(rung);

    auto pulse = [&](bool a, bool b, bool c) {
        sim::findPlcTag(doc, "I0.0")->flag = a;
        sim::findPlcTag(doc, "I0.1")->flag = b;
        sim::findPlcTag(doc, "I0.2")->flag = c;
        std::string error;
        REQUIRE(plc.start(doc, error));
        plc.scan(scene, 0.01, 0.01);
    };
    pulse(false, true, true);
    REQUIRE_FALSE(sim::findPlcTag(doc, "Q0.0")->flag);
    pulse(true, false, false);
    REQUIRE_FALSE(sim::findPlcTag(doc, "Q0.0")->flag);
    pulse(true, true, false);
    REQUIRE(sim::findPlcTag(doc, "Q0.0")->flag);
    pulse(true, false, true);
    REQUIRE(sim::findPlcTag(doc, "Q0.0")->flag);

    const nlohmann::json json = sim::softPlcToJson(doc);
    sim::SoftPlcDocument loaded;
    std::string error;
    REQUIRE(sim::softPlcFromJson(json, loaded, error));
    REQUIRE(loaded.rungs[0].series[1].alternatives.size() == 1);
    REQUIRE(loaded.rungs[0].series[1].alternatives[0].address == "I0.2");

    nlohmann::json nested = json;
    nested["rungs"][0]["series"][1]["or"][0]["or"] = nlohmann::json::array({nlohmann::json{{"address", "I0.3"}}});
    REQUIRE_FALSE(sim::softPlcFromJson(nested, loaded, error));
    REQUIRE(error == "plc contact or does not nest");
}

TEST_CASE("imported robot drafts unwired plc tags", "[plc]") {
    sim::SoftPlcDocument doc;
    sim::KinematicResource robot;
    robot.name = "kr420r3080_4";
    sim::appendRobotPlcDraft(doc, robot);

    auto bound = [&](const char* address) -> const sim::PlcBinding* {
        for (const sim::PlcBinding& binding : doc.bindings) {
            if (binding.address == address) {
                return &binding;
            }
        }
        return nullptr;
    };
    for (const char* address : {"R1_AtHome", "R1_Busy", "R1_Fault"}) {
        const sim::PlcTag* tag = sim::findPlcTag(doc, address);
        REQUIRE(tag != nullptr);
        REQUIRE_FALSE(tag->forced);
        const sim::PlcBinding* binding = bound(address);
        REQUIRE(binding != nullptr);
        REQUIRE(binding->device.empty());
        REQUIRE(binding->direction == "in");
    }
    for (const char* address : {"R1_Start", "R1_Grip"}) {
        const sim::PlcBinding* binding = bound(address);
        REQUIRE(binding != nullptr);
        REQUIRE(binding->device.empty());
        REQUIRE(binding->direction == "out");
    }

    sim::KinematicResource track;
    track.name = "irbt6004_0_d_10m_01";
    sim::KinJoint axis;
    axis.externalName = "E1";
    axis.externalKind = "track";
    axis.motion = sim::JointMotion::Prismatic;
    track.joints.push_back(axis);
    sim::appendRobotPlcDraft(doc, track);
    REQUIRE(sim::findPlcTag(doc, "E1_InPos") != nullptr);
    REQUIRE(bound("E1_InPos")->direction == "in");
    REQUIRE(bound("E1_InPos")->device.empty());
    REQUIRE(sim::findPlcTag(doc, "R2_AtHome") == nullptr);

    sim::KinematicResource second;
    second.name = "kr6";
    sim::appendRobotPlcDraft(doc, second);
    REQUIRE(sim::findPlcTag(doc, "R2_Start") != nullptr);
    REQUIRE(sim::findPlcTag(doc, "R3_Start") == nullptr);
}

TEST_CASE("commission robot cycle is started by the photoeye latch", "[plc]") {
    sim::Scene scene;
    sim::buildCommissionLine(scene);
    sim::attachCommissionRobot(scene);
    REQUIRE(scene.softPlc.zones.size() == 1);
    REQUIRE(scene.softPlc.zones[0].shape == "box");
    REQUIRE(scene.softPlc.zones[0].size.x == Approx(100));
    REQUIRE(scene.softPlc.zones[0].size.z == Approx(80));
    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    sim::Simulation simulation(scene, *engine);
    sim::SoftPlcRuntime plc;
    sim::LogicMachine logic;
    std::string error;
    simulation.setHooks([&] { REQUIRE(plc.start(scene.softPlc, error)); },
                        [&](double dt) {
                            plc.scan(scene, simulation.time(), dt);
                            logic.step(scene, dt);
                        },
                        [] {});
    simulation.play();
    double peak = 0;
    bool started = false;
    bool blocked = false;
    for (int i = 0; i < 80; ++i) {
        simulation.tick(0.02);
        const sim::KinematicResource* robot = sim::findResource(scene.resources, "r1");
        REQUIRE(robot != nullptr);
        peak = std::max(peak, std::abs(robot->joints.front().position));
        const sim::PlcTag* start = sim::findPlcTag(scene.softPlc, "R1_Start");
        const sim::PlcTag* zone = sim::findPlcTag(scene.softPlc, "Z1_Occupied");
        REQUIRE(start != nullptr);
        REQUIRE(zone != nullptr);
        if (start->flag) {
            started = true;
        }
        if (started && zone->flag && !start->flag) {
            blocked = true;
        }
    }
    REQUIRE(scene.findSignal("photo")->flag);
    REQUIRE_FALSE(scene.findSignal("belt_run")->flag);
    REQUIRE(peak > 20);
    REQUIRE(blocked);
    REQUIRE(scene.findSignal("Z1_Occupied")->flag);
    REQUIRE_FALSE(scene.findSignal("r1_start")->flag);
}

TEST_CASE("forcing R1_Start moves the robot without the photoeye", "[plc]") {
    sim::Scene scene;
    sim::buildCommissionLine(scene);
    sim::attachCommissionRobot(scene);
    if (const auto part = scene.findByName("Part")) {
        part->local.translation = {-240, 0, 46};
    }
    sim::PlcTag* start = sim::findPlcTag(scene.softPlc, "R1_Start");
    REQUIRE(start != nullptr);
    start->forced = true;
    start->forceFlag = true;
    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    sim::Simulation simulation(scene, *engine);
    sim::SoftPlcRuntime plc;
    sim::LogicMachine logic;
    std::string error;
    simulation.setHooks([&] { REQUIRE(plc.start(scene.softPlc, error)); },
                        [&](double dt) {
                            plc.scan(scene, simulation.time(), dt);
                            logic.step(scene, dt);
                        },
                        [] {});
    simulation.play();
    double peak = 0;
    for (int i = 0; i < 40; ++i) {
        simulation.tick(0.02);
        const sim::KinematicResource* robot = sim::findResource(scene.resources, "r1");
        peak = std::max(peak, std::abs(robot->joints.front().position));
    }
    REQUIRE_FALSE(scene.findSignal("photo")->flag);
    REQUIRE(scene.findSignal("r1_start")->flag);
    REQUIRE(peak > 30);
}

TEST_CASE("a watched sphere writes Z1_Occupied and can reset start", "[plc]") {
    sim::Scene scene;
    sim::SoftPlcDocument& doc = scene.softPlc;
    doc.enabled = true;
    doc.scanSeconds = 0.01;
    auto node = scene.makeNode(sim::NodeKind::Part, "ArmTip");
    node->local.translation = {0, 0, 0};
    scene.addChild(scene.root(), node);
    sim::PlcZoneWatch zone;
    zone.name = "Z1";
    zone.center = {100, 0, 0};
    zone.radius = 40;
    zone.watch = {"ArmTip"};
    doc.zones.push_back(zone);
    sim::PlcTag start;
    start.address = "R1_Start";
    start.flag = true;
    doc.tags.push_back(start);
    sim::PlcRung reset;
    reset.series.push_back(levelContact("Z1_Occupied", true));
    reset.coil = "R1_Start";
    reset.action = "reset";
    doc.rungs.push_back(reset);

    sim::SoftPlcRuntime plc;
    std::string error;
    REQUIRE(plc.start(doc, error));
    plc.scan(scene, 0.01, 0.01);
    REQUIRE_FALSE(sim::findPlcTag(scene.softPlc, "Z1_Occupied")->flag);
    REQUIRE(sim::findPlcTag(scene.softPlc, "R1_Start")->flag);

    node->local.translation = {100, 0, 0};
    plc.scan(scene, 0.02, 0.01);
    REQUIRE(sim::findPlcTag(scene.softPlc, "Z1_Occupied")->flag);
    REQUIRE_FALSE(sim::findPlcTag(scene.softPlc, "R1_Start")->flag);
    REQUIRE(scene.findSignal("Z1_Occupied")->flag);

    sim::findPlcTag(scene.softPlc, "Z1_Occupied")->forced = true;
    sim::findPlcTag(scene.softPlc, "Z1_Occupied")->forceFlag = false;
    plc.scan(scene, 0.03, 0.01);
    REQUIRE_FALSE(scene.findSignal("Z1_Occupied")->flag);
    REQUIRE(sim::findPlcTag(scene.softPlc, "R1_Start")->flag == false);

    nlohmann::json json = sim::softPlcToJson(scene.softPlc);
    sim::SoftPlcDocument loaded;
    REQUIRE(sim::softPlcFromJson(json, loaded, error));
    REQUIRE(loaded.zones.size() == 1);
    REQUIRE(loaded.zones[0].name == "Z1");
    REQUIRE(loaded.zones[0].shape == "sphere");
    REQUIRE(loaded.zones[0].radius == Approx(40));
    REQUIRE(loaded.zones[0].watch.size() == 1);
    json["zones"][0].erase("shape");
    json["zones"][0].erase("size");
    sim::SoftPlcDocument legacy;
    REQUIRE(sim::softPlcFromJson(json, legacy, error));
    REQUIRE(legacy.zones[0].shape == "sphere");
    REQUIRE(legacy.zones[0].radius == Approx(40));
}

TEST_CASE("mesh collision writes R1_Collision", "[plc]") {
    sim::Scene scene;
    auto arm = scene.makeNode(sim::NodeKind::Part, "ArmBox");
    arm->meshId = scene.addMesh(sim::makeBoxMesh({20, 20, 20}));
    arm->local.translation = {0, 0, 20};
    scene.addChild(scene.root(), arm);
    auto fixture = scene.makeNode(sim::NodeKind::Part, "FixtureBox");
    fixture->meshId = scene.addMesh(sim::makeBoxMesh({20, 20, 20}));
    fixture->local.translation = {0, 0, 20};
    scene.addChild(scene.root(), fixture);

    sim::KinematicResource resource;
    resource.name = "r1";
    sim::KinLink link;
    link.name = "arm";
    link.nodeId = arm->id;
    resource.links.push_back(link);
    scene.resources.push_back(resource);

    sim::SoftPlcDocument& doc = scene.softPlc;
    doc.enabled = true;
    doc.scanSeconds = 0.01;
    sim::PlcCollisionWatch watch;
    watch.address = "R1_Collision";
    watch.resource = "r1";
    doc.collisions.push_back(watch);

    sim::SoftPlcRuntime plc;
    std::string error;
    REQUIRE(plc.start(doc, error));
    plc.scan(scene, 0.01, 0.01);
    REQUIRE(sim::findPlcTag(scene.softPlc, "R1_Collision")->flag);
    REQUIRE(scene.findSignal("R1_Collision")->flag);

    fixture->local.translation = {200, 0, 20};
    plc.scan(scene, 0.02, 0.01);
    REQUIRE_FALSE(sim::findPlcTag(scene.softPlc, "R1_Collision")->flag);

    const nlohmann::json json = sim::softPlcToJson(scene.softPlc);
    sim::SoftPlcDocument loaded;
    REQUIRE(sim::softPlcFromJson(json, loaded, error));
    REQUIRE(loaded.collisions.size() == 1);
    REQUIRE(loaded.collisions[0].resource == "r1");
}

TEST_CASE("a box and a vertical cylinder write occupied bits", "[plc]") {
    sim::Scene scene;
    sim::SoftPlcDocument& doc = scene.softPlc;
    doc.enabled = true;
    doc.scanSeconds = 0.01;
    auto boxTip = scene.makeNode(sim::NodeKind::Part, "BoxTip");
    boxTip->local.translation = {0, 0, 0};
    scene.addChild(scene.root(), boxTip);
    auto cylTip = scene.makeNode(sim::NodeKind::Part, "CylTip");
    cylTip->local.translation = {0, 0, 0};
    scene.addChild(scene.root(), cylTip);

    sim::PlcZoneWatch box;
    box.name = "ZB";
    box.shape = "box";
    box.center = {0, 0, 0};
    box.size = {10, 20, 30};
    box.watch = {"BoxTip"};
    sim::PlcZoneWatch cylinder;
    cylinder.name = "ZC";
    cylinder.shape = "cylinder";
    cylinder.center = {0, 0, 0};
    cylinder.size = {10, 0, 20};
    cylinder.watch = {"CylTip"};
    doc.zones.push_back(box);
    doc.zones.push_back(cylinder);

    sim::SoftPlcRuntime plc;
    std::string error;
    REQUIRE(plc.start(doc, error));
    plc.scan(scene, 0.01, 0.01);
    REQUIRE(sim::findPlcTag(scene.softPlc, "ZB_Occupied")->flag);
    REQUIRE(sim::findPlcTag(scene.softPlc, "ZC_Occupied")->flag);

    boxTip->local.translation = {10.5, 0, 0};
    cylTip->local.translation = {0, 11, 0};
    plc.scan(scene, 0.02, 0.01);
    REQUIRE_FALSE(sim::findPlcTag(scene.softPlc, "ZB_Occupied")->flag);
    REQUIRE_FALSE(sim::findPlcTag(scene.softPlc, "ZC_Occupied")->flag);

    boxTip->local.translation = {0, 20, 30};
    cylTip->local.translation = {0, 0, 21};
    plc.scan(scene, 0.03, 0.01);
    REQUIRE(sim::findPlcTag(scene.softPlc, "ZB_Occupied")->flag);
    REQUIRE_FALSE(sim::findPlcTag(scene.softPlc, "ZC_Occupied")->flag);

    cylTip->local.translation = {0, 0, 19};
    plc.scan(scene, 0.04, 0.01);
    REQUIRE(sim::findPlcTag(scene.softPlc, "ZC_Occupied")->flag);

    const nlohmann::json json = sim::softPlcToJson(scene.softPlc);
    sim::SoftPlcDocument loaded;
    REQUIRE(sim::softPlcFromJson(json, loaded, error));
    REQUIRE(loaded.zones[0].shape == "box");
    REQUIRE(loaded.zones[0].size.y == Approx(20));
    REQUIRE(loaded.zones[1].shape == "cylinder");
    REQUIRE(loaded.zones[1].size.z == Approx(20));
}

TEST_CASE("scan log exports time, scan number, and signals", "[plc]") {
    const std::string csv = sim::softPlcScanCsv(
        {"softplc started", "t=0.000 #1 Z1_Occupied=0", "t=0.350 #36 R1_Start=0 Z1_Occupied=1", "t=0.400 #40 D0=1,2"});
    REQUIRE(csv.rfind("time,scan,signal,value\n", 0) == 0);
    REQUIRE(csv.find("softplc") == std::string::npos);
    REQUIRE(csv.find("0.000,1,Z1_Occupied,0\n") != std::string::npos);
    REQUIRE(csv.find("0.350,36,R1_Start,0\n") != std::string::npos);
    REQUIRE(csv.find("0.350,36,Z1_Occupied,1\n") != std::string::npos);
    REQUIRE(csv.find("0.400,40,D0,\"1,2\"\n") != std::string::npos);
}
