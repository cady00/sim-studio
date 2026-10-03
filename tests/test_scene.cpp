#include "sim/core/Pick.hpp"
#include "sim/core/Scene.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using Catch::Approx;

TEST_CASE("child world transform includes the parent") {
    sim::Scene scene;
    auto assembly = scene.makeNode(sim::NodeKind::Assembly, "Station");
    assembly->local.translation = {0, 0, 100};
    scene.addChild(scene.root(), assembly);
    auto part = scene.makeNode(sim::NodeKind::Part, "Tool");
    part->local.translation = {5, 0, 0};
    scene.addChild(assembly, part);
    const sim::Vec3 origin = sim::transformPoint(part->worldMatrix(), {0, 0, 0});
    REQUIRE(origin.x == Approx(5));
    REQUIRE(origin.z == Approx(100));
    REQUIRE(scene.nodeCount() == 3);
}

TEST_CASE("ray hits a triangle on the ground") {
    sim::Scene scene;
    auto mesh = std::make_shared<sim::MeshData>();
    mesh->positions = {0, 0, 0, 10, 0, 0, 0, 10, 0};
    mesh->normals = {0, 0, 1, 0, 0, 1, 0, 0, 1};
    mesh->indices = {0, 1, 2};
    mesh->recomputeBounds();
    auto part = scene.makeNode(sim::NodeKind::Part, "Plate");
    part->meshId = scene.addMesh(mesh);
    scene.addChild(scene.root(), part);
    const auto hit = sim::pickScene(scene, sim::Ray{{2, 2, 10}, {0, 0, -1}});
    REQUIRE(hit.has_value());
    REQUIRE(hit->nodeId == part->id);
    REQUIRE(hit->point.z == Approx(0).margin(1e-6));
    REQUIRE_FALSE(sim::pickScene(scene, sim::Ray{{20, 20, 10}, {0, 0, -1}}).has_value());
}

TEST_CASE("hidden parent hides the child from picking") {
    sim::Scene scene;
    auto assembly = scene.makeNode(sim::NodeKind::Assembly, "Hidden");
    assembly->visible = false;
    scene.addChild(scene.root(), assembly);
    auto mesh = std::make_shared<sim::MeshData>();
    mesh->positions = {0, 0, 0, 10, 0, 0, 0, 10, 0};
    mesh->normals = {0, 0, 1, 0, 0, 1, 0, 0, 1};
    mesh->indices = {0, 1, 2};
    auto part = scene.makeNode(sim::NodeKind::Part, "Child");
    part->meshId = scene.addMesh(mesh);
    scene.addChild(assembly, part);
    REQUIRE_FALSE(part->effectivelyVisible());
    REQUIRE_FALSE(sim::pickScene(scene, sim::Ray{{2, 2, 5}, {0, 0, -1}}).has_value());
}
