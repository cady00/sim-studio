#include "sim/core/DemoScene.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/physics/PhysicsEngine.hpp"
#include "sim/scripting/ScriptHost.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>

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

}  // namespace

TEST_CASE("slider position drive reaches the target and honors the upper limit") {
    sim::Scene scene;
    auto base = addPart(scene, "Base");
    base->mech.kind = sim::MechKind::Collision;
    base->mech.halfExtents = {30, 30, 30};

    auto slider = addPart(scene, "Slider");
    slider->mech.kind = sim::MechKind::Slider;
    slider->mech.other = "Base";
    slider->mech.axis = {1, 0, 0};
    slider->mech.limited = true;
    slider->mech.lower = 0;
    slider->mech.upper = 40;
    slider->mech.drive = sim::DriveMode::Position;
    slider->mech.target = 200;
    slider->mech.maxSpeed = 500;
    slider->mech.maxAccel = 8000;
    slider->mech.autoMass = false;
    slider->mech.mass = 0.4;
    slider->mech.halfExtents = {15, 15, 15};
    slider->local.translation = {0, 0, 80};
    const double startX = slider->local.translation.x;

    auto engine = sim::createPhysicsEngine();
    sim::Simulation simulation(scene, *engine);
    simulation.play();
    REQUIRE(simulation.lastError().empty());
    simulation.tick(1.5);
    REQUIRE(slider->local.translation.x == Approx(40).margin(3));
    REQUIRE(slider->local.translation.x <= 43);
    REQUIRE(slider->local.translation.x > startX);
    simulation.stop();
    REQUIRE(slider->local.translation.x == Approx(startX).margin(0.01));
    REQUIRE(simulation.state() == sim::SimState::Stopped);
    REQUIRE(simulation.time() == 0);
}

TEST_CASE("hinge position drive stays inside its limit") {
    sim::Scene scene;
    auto base = addPart(scene, "Base");
    base->mech.kind = sim::MechKind::Collision;
    base->mech.halfExtents = {20, 20, 20};

    auto hinge = addPart(scene, "Hinge");
    hinge->mech.kind = sim::MechKind::Hinge;
    hinge->mech.other = "Base";
    hinge->mech.axis = {0, 0, 1};
    hinge->mech.limited = true;
    hinge->mech.lower = 0;
    hinge->mech.upper = 30;
    hinge->mech.drive = sim::DriveMode::Position;
    hinge->mech.target = 90;
    hinge->mech.maxSpeed = 180;
    hinge->mech.maxAccel = 2000;
    hinge->mech.autoMass = false;
    hinge->mech.mass = 0.3;
    hinge->mech.halfExtents = {40, 8, 8};
    hinge->local.translation = {0, 0, 50};

    auto engine = sim::createPhysicsEngine();
    std::string error;
    REQUIRE(engine->build(scene, error));
    stepFor(*engine, 1.2);
    const sim::Quat q = hinge->local.rotation.normalized();
    const double degrees = sim::rad2deg(2.0 * std::atan2(q.z, q.w));
    REQUIRE(degrees == Approx(30).margin(6));
    REQUIRE(degrees < 40);
}

TEST_CASE("light barrier and collision sensor write boolean signals") {
    sim::Scene scene;
    auto box = addPart(scene, "Box");
    box->mech.kind = sim::MechKind::Collision;
    box->mech.halfExtents = {40, 40, 40};

    auto barrier = addPart(scene, "Barrier");
    barrier->mech.kind = sim::MechKind::LightBarrier;
    barrier->mech.axis = {0, 1, 0};
    barrier->mech.range = 400;
    barrier->mech.signal = "blocked";
    barrier->mech.valueSignal = "distance";
    barrier->local.translation = {0, -200, 0};

    auto miss = addPart(scene, "Miss");
    miss->mech.kind = sim::MechKind::LightBarrier;
    miss->mech.axis = {0, 1, 0};
    miss->mech.range = 80;
    miss->mech.signal = "clear_beam";
    miss->local.translation = {0, -200, 300};

    auto sensor = addPart(scene, "Touch");
    sensor->mech.kind = sim::MechKind::CollisionSensor;
    sensor->mech.halfExtents = {30, 30, 30};
    sensor->mech.signal = "touch";

    auto engine = sim::createPhysicsEngine();
    std::string error;
    REQUIRE(engine->build(scene, error));
    stepFor(*engine, 0.1);
    const sim::Signal* blocked = scene.findSignal("blocked");
    const sim::Signal* distance = scene.findSignal("distance");
    const sim::Signal* clearBeam = scene.findSignal("clear_beam");
    const sim::Signal* touch = scene.findSignal("touch");
    REQUIRE(blocked);
    REQUIRE(blocked->flag);
    REQUIRE(distance);
    REQUIRE(distance->real < 400);
    REQUIRE(distance->real > 100);
    REQUIRE(clearBeam);
    REQUIRE_FALSE(clearBeam->flag);
    REQUIRE(touch);
    REQUIRE(touch->flag);
}

TEST_CASE("project file keeps mechanisms, signals, and scripts") {
    sim::Scene scene;
    sim::buildConveyorLine(scene);
    const auto path = std::filesystem::temp_directory_path() / "simstudio_mech.simproj";
    REQUIRE(sim::ProjectStore::save(scene, path.string()).ok);
    sim::Scene loaded;
    const sim::ProjectResult result = sim::ProjectStore::load(path.string(), loaded);
    INFO(result.message);
    REQUIRE(result.ok);
    const auto pusher = loaded.findByName("Pusher");
    REQUIRE(pusher);
    REQUIRE(pusher->mech.kind == sim::MechKind::Slider);
    REQUIRE(pusher->mech.upper == Approx(140));
    REQUIRE(pusher->mech.drive == sim::DriveMode::Position);
    REQUIRE(loaded.findByName("Conveyor")->mech.kind == sim::MechKind::Conveyor);
    REQUIRE(loaded.findByName("Source")->mech.prototype == "Blank");
    REQUIRE(loaded.findByName("Sink")->mech.kind == sim::MechKind::Sink);
    REQUIRE(loaded.findSignal("barrier"));
    REQUIRE(loaded.activeScript == "pusher");
    REQUIRE(loaded.scripts.size() == 3);
    REQUIRE(loaded.scripts.front().source.find("set_joint_target") != std::string::npos);
    std::filesystem::remove(path);
}

TEST_CASE("python api writes signals, joint targets, and can spawn") {
    sim::Scene scene;
    auto slider = addPart(scene, "Slider");
    slider->mech.kind = sim::MechKind::Slider;
    auto host = sim::createScriptHost();
    host->bind(&scene);
    std::string error;
    const char* source = R"(
def on_start():
    sim.set_bool("go", True)
    sim.set_joint_target("Slider", 42)
    sim.set_real("gap", 1.25)
    sim.set_int("n", 7)

def on_step(dt):
    sim.set_joint_speed("Slider", 15)
    print("tick", dt)
)";
    REQUIRE(host->load(source, error));
    host->onStart();
    REQUIRE(scene.findSignal("go")->flag);
    REQUIRE(slider->mech.target == Approx(42));
    REQUIRE(slider->mech.drive == sim::DriveMode::Position);
    REQUIRE(scene.findSignal("gap")->real == Approx(1.25));
    REQUIRE(scene.findSignal("n")->integer == 7);
    host->onStep(0.016);
    REQUIRE(slider->mech.drive == sim::DriveMode::Speed);
    REQUIRE(slider->mech.target == Approx(15));
    REQUIRE(host->takeLog().find("tick") != std::string::npos);

    std::string output;
    REQUIRE(host->execute("spawned = sim.spawn('Slider')\nprint(spawned)\n", output, error));
    REQUIRE(scene.nodeCount() >= 3);
}

TEST_CASE("simulation speed and pause do not advance time while paused") {
    sim::Scene scene;
    sim::buildConveyorLine(scene);
    auto engine = sim::createPhysicsEngine();
    sim::Simulation simulation(scene, *engine);
    simulation.setSpeed(2);
    simulation.play();
    REQUIRE(simulation.lastError().empty());
    simulation.tick(0.05);
    const double played = simulation.time();
    REQUIRE(played > 0);
    simulation.pause();
    simulation.tick(0.2);
    REQUIRE(simulation.time() == Approx(played));
    simulation.reset();
    REQUIRE(simulation.time() == 0);
    REQUIRE(simulation.state() == sim::SimState::Stopped);
}
