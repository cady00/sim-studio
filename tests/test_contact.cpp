#include "sim/core/DemoScene.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/physics/ContactNarrow.hpp"
#include "sim/physics/PhysicsEngine.hpp"
#include "sim/scripting/ScriptHost.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
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

    REQUIRE(scene.convexCache.size() == 1);
    bool nearPlate = false;
    bool aabbCorner = false;
    for (const sim::ConvexCacheEntry& entry : scene.convexCache) {
        REQUIRE(entry.pieces.size() == 1);
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
    REQUIRE(scene.contactNotices.size() == 1);
    REQUIRE(scene.contactNotices.front().self == "Crate");
    REQUIRE(scene.contactNotices.front().other == "Floor");
    REQUIRE(scene.contactNotices.front().impulse == Approx(9.4176).margin(1e-3));
    REQUIRE(scene.findSignal("contact") != nullptr);
    REQUIRE(scene.findSignal("contact")->flag);
    REQUIRE(scene.findSignal("contact_impulse")->real == Approx(9.4176).margin(1e-3));

    auto host = sim::createScriptHost();
    host->bind(&scene);
    std::string output;
    std::string scriptError;
    REQUIRE(host->execute("print(len(sim.contacts()))", output, scriptError));
    REQUIRE(scriptError.empty());
    REQUIRE(std::stoi(output) == 1);

    REQUIRE(host->load("seen = []\ndef on_contact(a, b, impulse):\n    seen.append((a, b, impulse))\n", scriptError));
    host->onContacts();
    output.clear();
    REQUIRE(host->execute("print(seen[0][0], seen[0][1], seen[0][2])", output, scriptError));
    REQUIRE(output.find("Crate") != std::string::npos);
    REQUIRE(output.find("Floor") != std::string::npos);
    REQUIRE(std::stod(output.substr(output.find_last_of(' ') + 1)) == Approx(9.4176).margin(1e-3));
}

TEST_CASE("two meshes report contacts on both sides") {
    const auto left = sim::makeBoxMesh({10, 10, 10});
    const auto right = sim::makeBoxMesh({10, 10, 10});
    const sim::TriangleBvh leftBvh = sim::buildTriangleBvh(*left);
    const sim::TriangleBvh rightBvh = sim::buildTriangleBvh(*right);
    sim::Mat4 shifted = sim::Mat4::identity();
    shifted(3, 0) = 12;
    const std::vector<sim::NarrowContact> hits = sim::queryMeshMesh(leftBvh, sim::Mat4::identity(), rightBvh, shifted, 8);
    REQUIRE(hits.size() == 4);
    bool opposite = false;
    for (std::size_t i = 0; i < hits.size() && !opposite; ++i) {
        for (std::size_t j = i + 1; j < hits.size(); ++j) {
            if (sim::dot(hits[i].normal, hits[j].normal) < -0.5) {
                opposite = true;
                break;
            }
        }
    }
    REQUIRE(opposite);
    shifted(3, 0) = 40;
    REQUIRE(sim::queryMeshMesh(leftBvh, sim::Mat4::identity(), rightBvh, shifted, 8).empty());
}

TEST_CASE("the solver pushes one mesh out of another") {
    sim::Scene scene;
    const auto shape = sim::makeBoxMesh({10, 10, 10});
    auto fixed = addPart(scene, "FixedMesh");
    fixed->mech.kind = sim::MechKind::Collision;
    fixed->mech.motion = sim::BodyMotion::Anchored;
    fixed->mech.kinematic = true;
    fixed->mech.shape = sim::CollisionShape::Mesh;
    fixed->local.translation = {12, 0, 10};
    fixed->meshId = scene.addMesh(shape);
    auto moving = addPart(scene, "MovingMesh");
    moving->mech.kind = sim::MechKind::Rigid;
    moving->mech.motion = sim::BodyMotion::Free;
    moving->mech.shape = sim::CollisionShape::Mesh;
    moving->mech.allowSleep = false;
    moving->local.translation = {0, 0, 10};
    moving->meshId = scene.addMesh(shape);
    stepSeconds(scene, 0.15);
    REQUIRE(moving->local.translation.x < -1);
}

TEST_CASE("a box on two pads keeps both contact points") {
    auto mesh = std::make_shared<sim::MeshData>();
    mesh->positions = {-40, -15, 0, -10, -15, 0, -25, 15, 0, 10, -15, 0, 40, -15, 0, 25, 15, 0};
    mesh->indices = {0, 1, 2, 3, 4, 5};
    mesh->recomputeBounds();
    const sim::TriangleBvh bvh = sim::buildTriangleBvh(*mesh);
    const std::vector<sim::NarrowContact> hits =
        sim::queryBoxMesh(bvh, sim::Mat4::identity(), {0, 0, 8}, {50, 20, 10}, 4);
    REQUIRE(hits.size() >= 2);
    REQUIRE(sim::length(hits[0].point - hits[1].point) > 20);
}

TEST_CASE("a fast box stops on a thin plate instead of jumping through") {
    double toi = 0;
    sim::Vec3 normal;
    REQUIRE(sim::sweepAabbToi({0, 0, 80}, {10, 10, 10}, {0, 0, -5000}, 1.0 / 60.0, {0, 0, 0}, {40, 40, 1}, toi, normal));
    REQUIRE(toi == Approx(69.0 / 5000.0).margin(1e-6));
    REQUIRE(normal.z == Approx(1.0).margin(1e-6));
    const auto plate = sim::makeBoxMesh({40, 40, 1});
    REQUIRE(sim::sweepAabbMesh({0, 0, 80}, {10, 10, 10}, {0, 0, -5000}, 1.0 / 60.0, sim::buildTriangleBvh(*plate),
                               sim::Mat4::identity(), toi, normal));
    REQUIRE(toi == Approx(69.0 / 5000.0).margin(1e-4));
    REQUIRE(normal.z == Approx(1.0).margin(1e-3));

    sim::Scene scene;
    auto floor = addPart(scene, "Plate");
    floor->mech.kind = sim::MechKind::Collision;
    floor->mech.motion = sim::BodyMotion::Anchored;
    floor->mech.kinematic = true;
    floor->mech.shape = sim::CollisionShape::Mesh;
    floor->mech.halfExtents = {40, 40, 1};
    floor->local.translation = {0, 0, 0};
    floor->meshId = scene.addMesh(plate);
    auto box = addPart(scene, "Fast");
    box->mech.kind = sim::MechKind::Rigid;
    box->mech.motion = sim::BodyMotion::Free;
    box->mech.halfExtents = {10, 10, 10};
    box->mech.continuous = true;
    box->mech.allowSleep = false;
    box->local.translation = {0, 0, 400};
    stepSeconds(scene, 1.2);
    REQUIRE(box->local.translation.z > 8);
    REQUIRE(box->local.translation.z < 40);
}

TEST_CASE("a ball joint swing stays inside its cone") {
    sim::Scene scene;
    auto ball = addPart(scene, "Cone");
    ball->mech.kind = sim::MechKind::Ball;
    ball->mech.axis = {0, 0, 1};
    ball->mech.spin = {0, 90, 0};
    ball->mech.angularDamp = 0;
    ball->mech.drive = sim::DriveMode::None;
    ball->mech.limited = true;
    ball->mech.upper = 12;
    stepSeconds(scene, 0.5);
    const sim::Vec3 swung = ball->local.rotation.rotate({0, 0, 1});
    const double cosine = std::clamp(sim::dot(sim::normalize(swung), sim::Vec3{0, 0, 1}), -1.0, 1.0);
    const double angle = std::acos(cosine) * 180.0 / 3.14159265358979323846;
    REQUIRE(angle > 5);
    REQUIRE(angle < 14);
}
