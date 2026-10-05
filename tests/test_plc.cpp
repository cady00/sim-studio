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
    ton.series.push_back({"I0.0", true});
    ton.coil = "T0";
    ton.action = "ton";
    sim::PlcRung extend;
    extend.series.push_back({"T0", true});
    extend.coil = "Q0.0";
    extend.action = "out";
    sim::PlcRung belt;
    belt.series.push_back({"I0.0", false});
    belt.coil = "Q0.1";
    belt.action = "out";
    sim::PlcRung moveInt;
    moveInt.coil = "D0";
    moveInt.action = "move";
    moveInt.literal = 42;
    sim::PlcRung moveReal;
    moveReal.series.push_back({"Q0.0", true});
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
    rung.series.push_back({"I0.0", true});
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
    rung.series.push_back({"I0.0", false});
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
}
