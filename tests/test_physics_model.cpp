#include "sim/ai/Commands.hpp"
#include "sim/core/DemoScene.hpp"
#include "sim/core/PhysicsModel.hpp"
#include "sim/physics/PhysicsEngine.hpp"
#include "sim/scripting/ScriptHost.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <iostream>

using Catch::Approx;

namespace {

void stepFor(sim::IPhysicsEngine& engine, double seconds) {
    const int steps = static_cast<int>(std::round(seconds / sim::Simulation::kFixedDt));
    for (int i = 0; i < steps; ++i) {
        engine.step(sim::Simulation::kFixedDt);
    }
}

}  // namespace

TEST_CASE("motion profile, friction pairs, and encoder pulses") {
    REQUIRE(sim::stoppingDistance(10, 2) == Approx(25));
    REQUIRE(sim::stoppingTime(10, 2) == Approx(5));
    REQUIRE(sim::timeToSpeed(0, 10, 2) == Approx(5));

    sim::PhysicsDocument document;
    document.defaultFriction = 0.2;
    document.grades = {{"rubber", 0.64, 0.1}, {"steel", 0.25, 0.02}};
    REQUIRE(sim::pairFriction(document, "missing", "other") == Approx(0.2));
    REQUIRE(sim::pairFriction(document, "rubber", "steel") == Approx(0.4));
    document.pairs.push_back({"steel", "rubber", 0.55, 0.2});
    REQUIRE(sim::pairFriction(document, "rubber", "steel") == Approx(0.55));
    REQUIRE(sim::pairRestitution(document, "rubber", "steel") == Approx(0.2));
    document.referenceFriction = 0.5;
    REQUIRE(sim::conveyorScale(document, "rubber", "steel", 1) == Approx(1));
    REQUIRE(sim::conveyorScale(document, "slick", "rubber", 1) == Approx(0.4));

    sim::MotorAspect motor;
    motor.distance = 22;
    motor.pulseDistance = 10;
    motor.highRatio = 0.5;
    motor.grayCode = true;
    sim::EncoderSample sample = sim::sampleEncoder(motor);
    REQUIRE(sample.count == 2);
    REQUIRE(sample.gray == 3);
    REQUIRE(sample.pulseHigh);
    motor.distance = 27;
    sample = sim::sampleEncoder(motor);
    REQUIRE(sample.pulseHigh == false);
}

TEST_CASE("old mechanism json infers motion and round-trips the new fields") {
    nlohmann::json legacy = {{"kind", "rigid"}, {"kinematic", true}};
    sim::Mech mech;
    sim::mechFromJson(legacy, mech);
    REQUIRE(mech.motion == sim::BodyMotion::Driven);

    mech.motion = sim::BodyMotion::Surface;
    mech.surfaceRide = true;
    mech.shape = sim::CollisionShape::Cylinder;
    mech.surface = "rubber";
    mech.joinAncestor = true;
    const nlohmann::json saved = sim::mechToJson(mech);
    sim::Mech loaded;
    sim::mechFromJson(saved, loaded);
    REQUIRE(loaded.motion == sim::BodyMotion::Surface);
    REQUIRE(loaded.surfaceRide);
    REQUIRE(loaded.shape == sim::CollisionShape::Cylinder);
    REQUIRE(loaded.joinAncestor);
    REQUIRE(sim::effectiveMotion(loaded) == sim::BodyMotion::Surface);
}

TEST_CASE("surface ride keeps friction order and hands a load to free motion") {
    sim::Scene scene;
    sim::buildTransferLine(scene);
    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    const auto rubber = scene.findByName("RubberBox");
    const auto slick = scene.findByName("SlickBox");
    REQUIRE(rubber);
    REQUIRE(slick);
    const double rubberX = rubber->local.translation.x;
    const double slickX = slick->local.translation.x;
    stepFor(*engine, 1.0);
    REQUIRE(rubber->local.translation.x > rubberX + 200);
    REQUIRE(slick->local.translation.x > slickX + 10);
    REQUIRE(rubber->local.translation.x > slick->local.translation.x + 150);
    REQUIRE(rubber->mech.motion == sim::BodyMotion::Surface);
    const sim::Signal* count = scene.findSignal("encoder_count");
    REQUIRE(count != nullptr);
    REQUIRE(count->integer == 12);
    const sim::Signal* pulse = scene.findSignal("encoder_pulse");
    REQUIRE(pulse != nullptr);
    const int contacts = scene.physicsDoc.stats.contacts;
    REQUIRE(contacts == 2);
    REQUIRE(scene.physicsDoc.stats.continuous == 0);
    engine->step(sim::Simulation::kFixedDt);
    REQUIRE(scene.physicsDoc.stats.contacts < contacts * 3);

    sim::Scene handoff;
    auto belt = handoff.makeNode(sim::NodeKind::Part, "Belt");
    belt->mech.kind = sim::MechKind::Conveyor;
    belt->mech.motion = sim::BodyMotion::Driven;
    belt->mech.kinematic = true;
    belt->mech.halfExtents = {200, 80, 20};
    belt->mech.speed = 100;
    belt->mech.direction = {1, 0, 0};
    belt->local.translation = {0, 0, 20};
    handoff.addChild(handoff.root(), belt);
    auto gate = handoff.makeNode(sim::NodeKind::Part, "Gate");
    gate->mech.kind = sim::MechKind::Collision;
    gate->mech.handoff = true;
    gate->mech.halfExtents = {40, 40, 40};
    gate->local.translation = {0, 0, 40};
    handoff.addChild(handoff.root(), gate);
    auto load = handoff.makeNode(sim::NodeKind::Part, "Load");
    load->mech.kind = sim::MechKind::Rigid;
    load->mech.motion = sim::BodyMotion::Surface;
    load->mech.surfaceRide = true;
    load->mech.kinematic = true;
    load->mech.halfExtents = {20, 20, 20};
    load->local.translation = {0, 0, 50};
    handoff.addChild(handoff.root(), load);
    auto builtin = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    REQUIRE(builtin->build(handoff, error));
    builtin->step(sim::Simulation::kFixedDt);
    REQUIRE(load->mech.motion == sim::BodyMotion::Free);
    REQUIRE(load->mech.kinematic == false);
}

TEST_CASE("joined child stays with its ancestor while a free sibling falls") {
    sim::Scene scene;
    auto parent = scene.makeNode(sim::NodeKind::Part, "Parent");
    parent->mech.kind = sim::MechKind::Collision;
    parent->mech.motion = sim::BodyMotion::Anchored;
    parent->mech.halfExtents = {40, 40, 10};
    parent->local.translation = {0, 0, 0};
    scene.addChild(scene.root(), parent);
    auto child = scene.makeNode(sim::NodeKind::Part, "Child");
    child->mech.kind = sim::MechKind::Rigid;
    child->mech.joinAncestor = true;
    child->mech.motion = sim::BodyMotion::Free;
    child->mech.halfExtents = {10, 10, 10};
    child->local.translation = {0, 0, 400};
    scene.addChild(parent, child);
    auto sibling = scene.makeNode(sim::NodeKind::Part, "Sibling");
    sibling->mech.kind = sim::MechKind::Rigid;
    sibling->mech.motion = sim::BodyMotion::Free;
    sibling->mech.halfExtents = {10, 10, 10};
    sibling->local.translation = {80, 0, 400};
    scene.addChild(scene.root(), sibling);

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    stepFor(*engine, 0.25);
    REQUIRE(child->local.translation.z == Approx(400).margin(0.1));
    REQUIRE(sibling->local.translation.z < 350);
}

TEST_CASE("bounded numeric ik reaches a two link target") {
    const auto residual = [](const std::vector<double>& values, std::vector<double>& out) {
        const double q1 = sim::deg2rad(values[0]);
        const double q2 = sim::deg2rad(values[1]);
        const double x = 100 * std::cos(q1) + 80 * std::cos(q1 + q2);
        const double y = 100 * std::sin(q1) + 80 * std::sin(q1 + q2);
        out = {x - 100, y - 80};
    };
    sim::IkSolverSettings settings;
    settings.iterations = 60;
    settings.epsilon = 1e-3;
    const sim::IkNumberResult solved =
        sim::solveBoundedIk(residual, 2, {20, 20}, {-160, -160}, {160, 160}, sim::IkSolverKind::QuasiNewton, settings);
    REQUIRE(solved.ok);
    std::vector<double> check;
    residual(solved.values, check);
    REQUIRE(std::hypot(check[0], check[1]) < 0.05);

    const sim::IkNumberResult sqp =
        sim::solveBoundedIk(residual, 2, {20, 20}, {-160, -160}, {160, 160}, sim::IkSolverKind::SequentialQuadratic, settings);
    REQUIRE(sqp.solver == "sqp");
    REQUIRE(sqp.residual < 1.0);
}

TEST_CASE("commands and python edit motion, pairs, and motors") {
    sim::Scene scene;
    auto node = scene.makeNode(sim::NodeKind::Part, "Box");
    scene.addChild(scene.root(), node);
    sim::CommandExecutor executor(scene, nullptr, nullptr);
    REQUIRE(executor.execute({{"op", "physics_set_motion"}, {"name", "Box"}, {"motion", "anchored"}}).ok);
    REQUIRE(node->mech.motion == sim::BodyMotion::Anchored);
    REQUIRE(node->mech.kinematic);
    REQUIRE(executor.execute({{"op", "physics_pair"}, {"a", "rubber"}, {"b", "steel"}, {"friction", 0.33}, {"restitution", 0.1}}).ok);
    REQUIRE(sim::pairFriction(scene.physicsDoc, "steel", "rubber") == Approx(0.33));
    const sim::CommandResult profile = executor.execute({{"op", "motor_profile"}, {"speed", 10}, {"decel", 2}});
    REQUIRE(profile.ok);
    REQUIRE(profile.value.at("distance").get<double>() == Approx(25));
    REQUIRE(executor.execute({{"op", "motor_set"}, {"name", "drive"}, {"target", 12}, {"running", true}}).ok);
    REQUIRE(scene.motors.size() == 1);
    REQUIRE(scene.motors.front().running);

    auto host = sim::createScriptHost();
    host->bind(&scene);
    std::string output;
    std::string error;
    REQUIRE(host->execute("sim.set_body_motion('Box', 'surface')\nsim.set_surface_grade('Box', 'rubber')\nsim.set_motor('drive', 40, True)\nprint(sim.encoder_count('drive'))\n",
                          output, error));
    REQUIRE(error.empty());
    REQUIRE(node->mech.motion == sim::BodyMotion::Surface);
    REQUIRE(node->mech.surface == "rubber");
    REQUIRE(scene.motors.front().targetSpeed == Approx(40));
}
