#include "sim/core/Mesh.hpp"
#include "sim/io/AssetImport.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <fstream>
#include <string>

namespace {

std::string readAll(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

sim::Scene sampleScene() {
    sim::Scene scene;
    scene.physicsDoc.defaultFriction = 0.35;
    scene.physicsDoc.defaultRestitution = 0.12;
    auto base = scene.makeNode(sim::NodeKind::Part, "Base");
    base->meshId = scene.addMesh(sim::makeBoxMesh({25, 25, 25}));
    base->color = {0.2, 0.4, 0.8};
    base->metallic = 0.25f;
    base->roughness = 0.4f;
    base->local.translation = {10, 20, 30};
    base->local.rotation = sim::Quat::fromAxisAngle({0, 0, 1}, sim::kPi / 2);
    base->mech.kind = sim::MechKind::Rigid;
    base->mech.mass = 3.5;
    base->mech.autoMass = false;
    base->mech.density = 1200;
    base->mech.shape = sim::CollisionShape::Box;
    base->mech.halfExtents = {25, 25, 25};
    base->mech.motion = sim::BodyMotion::Free;
    scene.addChild(scene.root(), base);

    auto lid = scene.makeNode(sim::NodeKind::Part, "Lid");
    lid->meshId = scene.addMesh(sim::makeBoxMesh({20, 20, 4}));
    lid->color = {0.9, 0.45, 0.15};
    lid->local.translation = {0, 0, 60};
    lid->mech.kind = sim::MechKind::Hinge;
    lid->mech.other = "Base";
    lid->mech.axis = {1, 0, 0};
    lid->mech.limited = true;
    lid->mech.lower = -15;
    lid->mech.upper = 40;
    lid->mech.mass = 0.4;
    lid->mech.autoMass = false;
    lid->mech.motion = sim::BodyMotion::Driven;
    scene.addChild(base, lid);

    sim::RobotPath path;
    path.name = "approach";
    path.resource = "kr420";
    sim::PathPoint start;
    start.name = "home";
    start.cartesian = true;
    start.move = sim::PathMove::Lin;
    start.position = {0, 0, 100};
    start.speed = 100;
    start.joints = {0, -10};
    sim::PathPoint goal = start;
    goal.name = "reach";
    goal.position = {100, 0, 100};
    goal.joints = {15, -10};
    path.points = {start, goal};
    scene.robotPaths.push_back(path);
    return scene;
}

}  // namespace

TEST_CASE("usda export round trips hierarchy mesh material physics and samples") {
    const sim::Scene scene = sampleScene();
    const std::string path = "/tmp/simstudio-usd-preview.usda";
    const sim::UsdIoResult written = sim::exportSceneUsda(scene, path);
    REQUIRE(written.ok);
    REQUIRE(written.meshCount == 2);
    REQUIRE(written.triangleCount == 24);
    REQUIRE(written.jointCount == 1);
    REQUIRE(written.sampleCount == 2);

    const std::string text = readAll(path);
    std::string error;
    REQUIRE(sim::validateUsdaText(text, error));
    REQUIRE(error.empty());
    REQUIRE(text.find("UsdPreviewSurface") != std::string::npos);
    REQUIRE(text.find("PhysicsRevoluteJoint") != std::string::npos);
    REQUIRE(text.find("metersPerUnit = 0.001") != std::string::npos);
    REQUIRE(text.find("upAxis = \"Z\"") != std::string::npos);

    sim::Scene loaded;
    const auto registry = sim::createBuiltinRegistry();
    const sim::AssetImportResult imported = registry->importPath(path, loaded);
    REQUIRE(imported.ok);
    REQUIRE(imported.format == "usda");
    REQUIRE(imported.meshCount == 2);
    REQUIRE(imported.triangleCount == 24);
    REQUIRE(imported.nodeCount == 2);

    const auto base = loaded.findByName("Base");
    const auto lid = loaded.findByName("Lid");
    REQUIRE(base);
    REQUIRE(lid);
    REQUIRE(base->local.translation.x == Catch::Approx(10));
    REQUIRE(base->local.translation.y == Catch::Approx(20));
    REQUIRE(base->local.translation.z == Catch::Approx(30));
    const sim::Vec3 turned = base->local.rotation.rotate({1, 0, 0});
    REQUIRE(turned.x == Catch::Approx(0).margin(1e-6));
    REQUIRE(turned.y == Catch::Approx(1).margin(1e-6));
    REQUIRE(base->color.x == Catch::Approx(0.2).margin(1e-5));
    REQUIRE(base->color.y == Catch::Approx(0.4).margin(1e-5));
    REQUIRE(base->metallic == Catch::Approx(0.25f));
    REQUIRE(base->roughness == Catch::Approx(0.4f));
    REQUIRE(base->mech.kind == sim::MechKind::Rigid);
    REQUIRE(base->mech.mass == Catch::Approx(3.5));
    REQUIRE(base->mech.density == Catch::Approx(1200));
    REQUIRE(base->mech.autoMass == false);
    REQUIRE(base->meshId.empty() == false);
    REQUIRE(loaded.mesh(base->meshId)->triangleCount() == 12);

    REQUIRE(lid->parent.lock() == base);
    REQUIRE(lid->local.translation.z == Catch::Approx(60));
    REQUIRE(lid->mech.kind == sim::MechKind::Hinge);
    REQUIRE(lid->mech.other == "Base");
    REQUIRE(lid->mech.axis.x == Catch::Approx(1));
    REQUIRE(lid->mech.lower == Catch::Approx(-15));
    REQUIRE(lid->mech.upper == Catch::Approx(40));
    REQUIRE(lid->mech.mass == Catch::Approx(0.4));
    REQUIRE(lid->color.x == Catch::Approx(0.9).margin(1e-5));

    REQUIRE(loaded.robotPaths.size() == 1);
    REQUIRE(loaded.robotPaths[0].name == "approach");
    REQUIRE(loaded.robotPaths[0].resource == "kr420");
    REQUIRE(loaded.robotPaths[0].points.size() == 2);
    REQUIRE(loaded.robotPaths[0].points[1].position.x == Catch::Approx(100));
    REQUIRE(loaded.robotPaths[0].points[1].joints.size() == 2);
    REQUIRE(loaded.robotPaths[0].points[1].joints[0] == Catch::Approx(15));
    REQUIRE(loaded.robotPaths[0].points[0].speed == Catch::Approx(100).margin(1e-4));
    REQUIRE(loaded.physicsDoc.defaultFriction == Catch::Approx(0.35));
    REQUIRE(loaded.physicsDoc.defaultRestitution == Catch::Approx(0.12));
}

TEST_CASE("usda import is best effort and rejects crate bytes") {
    const char* quad = R"USD(#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
)
def Xform "World"
{
    def Mesh "Plate"
    {
        int[] faceVertexCounts = [4]
        int[] faceVertexIndices = [0, 1, 2, 3]
        point3f[] points = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
        uniform token subdivisionScheme = "none"
    }
    def Xform "Remote" (
        references = @missing.usda@
    )
    {
    }
}
)USD";
    const std::string path = "/tmp/simstudio-usd-quad.usda";
    {
        std::ofstream out(path);
        out << quad;
    }
    sim::Scene scene;
    const sim::AssetImportResult imported = sim::createBuiltinRegistry()->importPath(path, scene);
    REQUIRE(imported.ok);
    REQUIRE(imported.triangleCount == 2);
    REQUIRE(imported.message.find("n-gon") != std::string::npos);
    REQUIRE(imported.message.find("references") != std::string::npos);
    REQUIRE(imported.message.find("upAxis") != std::string::npos);
    const auto plate = scene.findByName("Plate");
    REQUIRE(plate);
    REQUIRE(plate->local.translation.x == Catch::Approx(0));
    REQUIRE(scene.mesh(plate->meshId)->positions[3] == Catch::Approx(1000).margin(1e-2));

    std::string error;
    REQUIRE_FALSE(sim::validateUsdaText(quad, error));

    const std::string crate = "/tmp/simstudio-usd-crate.usd";
    {
        std::ofstream out(crate, std::ios::binary);
        out << "PXR-USDC\x00\x08";
    }
    sim::Scene rejected;
    const sim::AssetImportResult binary = sim::createBuiltinRegistry()->importPath(crate, rejected);
    REQUIRE_FALSE(binary.ok);
    REQUIRE(binary.message.find("crate") != std::string::npos);

    sim::Scene noise;
    const sim::AssetImportResult garbage = sim::createBuiltinRegistry()->importPath("/tmp/simstudio-usd-quad.usda", noise);
    REQUIRE(garbage.ok);
    {
        std::ofstream out("/tmp/simstudio-usd-broken.usda");
        out << "#usda 1.0\n(\n";
    }
    sim::Scene broken;
    const sim::AssetImportResult failed = sim::createBuiltinRegistry()->importPath("/tmp/simstudio-usd-broken.usda", broken);
    REQUIRE_FALSE(failed.ok);
    REQUIRE_FALSE(failed.message.empty());
}
