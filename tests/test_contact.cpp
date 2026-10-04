#include "sim/core/DemoScene.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/physics/PhysicsEngine.hpp"
#include "sim/scripting/ScriptHost.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

using Catch::Approx;

namespace {

std::shared_ptr<sim::SceneNode> addPart(sim::Scene& scene, const std::string& name) {
    auto node = scene.makeNode(sim::NodeKind::Part, name);
    scene.addChild(scene.root(), node);
    return node;
}

void stepSeconds(sim::Scene& scene, double seconds) {
    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    REQUIRE(error.empty());
    const int steps = static_cast<int>(std::lround(seconds / sim::Simulation::kFixedDt));
    for (int i = 0; i < steps; ++i) {
        engine->step(sim::Simulation::kFixedDt);
    }
}

}  // namespace

TEST_CASE("mesh triangles leave a hole that the bounding box would have filled") {
    sim::Scene scene;
    sim::buildMeshGapDemo(scene);
    auto plate = scene.findByName("Plate");
    auto supported = scene.findByName("Supported");
    auto falling = scene.findByName("FallingGap");
    REQUIRE(plate);
    REQUIRE(supported);
    REQUIRE(falling);
    REQUIRE(std::abs(falling->local.translation.x - plate->local.translation.x) < plate->mech.halfExtents.x);
    stepSeconds(scene, 0.4);
    REQUIRE(falling->local.translation.z < 0);
    REQUIRE(supported->local.translation.z == Approx(33).margin(4));

    REQUIRE_FALSE(scene.convexCache.empty());
    bool nearPlate = false;
    bool aabbCorner = false;
    for (const sim::ConvexCacheEntry& entry : scene.convexCache) {
        REQUIRE_FALSE(entry.pieces.empty());
        for (const sim::ConvexPiece& piece : entry.pieces) {
            for (const sim::Vec3& vertex : piece.vertices) {
                if (vertex.x > 70 && vertex.x < 170 && std::abs(vertex.z) <= 20) {
                    nearPlate = true;
                }
                const bool xCorner = std::abs(std::abs(vertex.x) - plate->mech.halfExtents.x) < 2;
                const bool yCorner = std::abs(std::abs(vertex.y) - plate->mech.halfExtents.y) < 2;
                const bool zCorner = std::abs(std::abs(vertex.z) - plate->mech.halfExtents.z) < 2;
                if (xCorner && yCorner && zCorner) {
                    aabbCorner = true;
                }
            }
        }
    }
    REQUIRE(nearPlate);
    REQUIRE_FALSE(aabbCorner);
}

TEST_CASE("convex cache uses the mesh hull instead of the box extents") {
    sim::Scene scene;
    auto floor = addPart(scene, "HullFloor");
    floor->mech.kind = sim::MechKind::Collision;
    floor->mech.motion = sim::BodyMotion::Anchored;
    floor->mech.kinematic = true;
    floor->mech.shape = sim::CollisionShape::ConvexHull;
    floor->mech.halfExtents = {400, 400, 400};
    floor->local.translation = {0, 0, 10};
    floor->meshId = scene.addMesh(sim::makeBoxMesh({80, 80, 10}));

    auto box = addPart(scene, "Box");
    box->mech.kind = sim::MechKind::Rigid;
    box->mech.motion = sim::BodyMotion::Free;
    box->mech.halfExtents = {20, 20, 20};
    box->local.translation = {0, 0, 80};

    stepSeconds(scene, 0.7);
    REQUIRE(box->local.translation.z == Approx(40).margin(6));
    REQUIRE(box->local.translation.z < 100);

    auto miss = addPart(scene, "Miss");
    miss->mech.kind = sim::MechKind::Rigid;
    miss->mech.motion = sim::BodyMotion::Free;
    miss->mech.halfExtents = {15, 15, 15};
    miss->local.translation = {220, 0, 40};
    stepSeconds(scene, 0.35);
    auto missed = scene.findByName("Miss");
    REQUIRE(missed);
    REQUIRE(missed->local.translation.z < 0);
}

TEST_CASE("coulomb friction keeps a slick box behind a high-friction box") {
    sim::Scene scene;
    sim::buildSlipDemo(scene);
    stepSeconds(scene, 0.45);
    auto grip = scene.findByName("GripBox");
    auto slick = scene.findByName("SlipBox");
    REQUIRE(grip);
    REQUIRE(slick);
    REQUIRE(grip->local.translation.z > 50);
    REQUIRE(slick->local.translation.z > 50);
    REQUIRE(grip->local.translation.x > -60);
    REQUIRE(slick->local.translation.x < -140);
    REQUIRE(grip->local.translation.x > slick->local.translation.x + 80);
}

TEST_CASE("restitution sends a dropped box back up") {
    auto drop = [](double restitution) {
        sim::Scene scene;
        scene.physicsDoc.defaultRestitution = restitution;
        auto floor = addPart(scene, "Floor");
        floor->mech.kind = sim::MechKind::Collision;
        floor->mech.motion = sim::BodyMotion::Anchored;
        floor->mech.kinematic = true;
        floor->mech.halfExtents = {200, 200, 10};
        floor->local.translation = {0, 0, 10};
        auto box = addPart(scene, "Box");
        box->mech.kind = sim::MechKind::Rigid;
        box->mech.halfExtents = {20, 20, 20};
        box->local.translation = {0, 0, 200};
        box->mech.allowSleep = false;
        auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
        std::string error;
        REQUIRE(engine->build(scene, error));
        double peak = 0;
        bool hit = false;
        for (int i = 0; i < 50; ++i) {
            engine->step(sim::Simulation::kFixedDt);
            if (box->local.translation.z < 55) {
                hit = true;
            }
            if (hit) {
                peak = std::max(peak, box->local.translation.z);
            }
        }
        REQUIRE(hit);
        return peak;
    };
    REQUIRE(drop(0.0) < 55);
    REQUIRE(drop(0.8) > 80);
}

TEST_CASE("a ball joint can spin about an axis that is not its hinge axis") {
    sim::Scene scene;
    auto ball = addPart(scene, "Ball");
    ball->mech.kind = sim::MechKind::Ball;
    ball->mech.axis = {0, 0, 1};
    ball->mech.spin = {0, 40, 0};
    ball->mech.angularDamp = 0;
    ball->mech.drive = sim::DriveMode::None;
    ball->mech.limited = false;
    stepSeconds(scene, 0.5);
    const sim::Vec3 spun = ball->local.rotation.rotate({1, 0, 0});
    REQUIRE(std::abs(spun.y) < 0.08);
    REQUIRE(std::abs(spun.z) > 0.25);
    REQUIRE(spun.x > 0.85);
}

TEST_CASE("contact notices are visible to python") {
    sim::Scene scene;
    scene.physicsDoc.defaultRestitution = 0;
    auto floor = addPart(scene, "Floor");
    floor->mech.kind = sim::MechKind::Collision;
    floor->mech.motion = sim::BodyMotion::Anchored;
    floor->mech.kinematic = true;
    floor->mech.halfExtents = {80, 80, 10};
    floor->local.translation = {0, 0, 10};
    auto box = addPart(scene, "Crate");
    box->mech.kind = sim::MechKind::Rigid;
    box->mech.halfExtents = {20, 20, 20};
    box->local.translation = {0, 0, 50};
    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    std::string error;
    REQUIRE(engine->build(scene, error));
    for (int i = 0; i < 40; ++i) {
        engine->step(sim::Simulation::kFixedDt);
    }
    REQUIRE_FALSE(scene.contactNotices.empty());
    REQUIRE(scene.findSignal("contact") != nullptr);
    REQUIRE(scene.findSignal("contact")->flag);
    REQUIRE(scene.findSignal("contact_impulse")->real > 0);

    auto host = sim::createScriptHost();
    host->bind(&scene);
    std::string output;
    std::string scriptError;
    REQUIRE(host->execute("print(len(sim.contacts()))", output, scriptError));
    REQUIRE(scriptError.empty());
    REQUIRE(output.find("0") == std::string::npos);

    REQUIRE(host->load("seen = []\ndef on_contact(a, b, impulse):\n    seen.append((a, b, impulse))\n", scriptError));
    host->onContacts();
    output.clear();
    REQUIRE(host->execute("print(seen[0][0], seen[0][1], seen[0][2] > 0)", output, scriptError));
    REQUIRE(output.find("Crate") != std::string::npos);
    REQUIRE(output.find("Floor") != std::string::npos);
    REQUIRE(output.find("True") != std::string::npos);
}
