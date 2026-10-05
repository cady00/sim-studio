#include "sim/core/DemoScene.hpp"
#include "sim/core/Pick.hpp"
#include "sim/core/Scene.hpp"
#include "sim/render/Renderer.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <string>

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

TEST_CASE("collectNames finds one part among a hundred thousand") {
    sim::Scene scene;
    sim::buildLargeField(scene, 100000, 8);
    std::vector<std::string> ids;
    const auto started = std::chrono::steady_clock::now();
    const int found = scene.collectNames("p99999", ids, 10);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    REQUIRE(found == 1);
    REQUIRE(ids.size() == 1);
    REQUIRE(scene.find(ids[0])->name == "p99999");
    REQUIRE(ms < 150.0);
    REQUIRE(scene.collectNames("missing-part", ids, 10) == 0);
}

TEST_CASE("a nearer rectangle hides the box behind it") {
    sim::OcclusionBuffer buffer;
    buffer.reset(32, 32);
    sim::ScreenRect wall;
    wall.valid = true;
    wall.x0 = 0;
    wall.y0 = 0;
    wall.x1 = 31;
    wall.y1 = 31;
    wall.nearest = 0.2f;
    sim::rasterizeOccluder(buffer, wall);
    sim::ScreenRect behind = wall;
    behind.x0 = 4;
    behind.y0 = 4;
    behind.x1 = 12;
    behind.y1 = 12;
    behind.nearest = 0.8f;
    REQUIRE(sim::hiddenByOcclusion(buffer, behind));
    sim::ScreenRect closer = behind;
    closer.nearest = 0.05f;
    REQUIRE_FALSE(sim::hiddenByOcclusion(buffer, closer));
    sim::ScreenRect aside = behind;
    aside.x0 = 8;
    aside.x1 = 14;
    REQUIRE(sim::hiddenByOcclusion(buffer, aside));
    sim::ScreenRect rim = behind;
    rim.x0 = 0;
    rim.x1 = 1;
    rim.nearest = 0.8f;
    REQUIRE_FALSE(sim::hiddenByOcclusion(buffer, rim));
    sim::ScreenRect neighbor = behind;
    neighbor.nearest = 0.22f;
    REQUIRE_FALSE(sim::hiddenByOcclusion(buffer, neighbor));
}

TEST_CASE("the gpu frustum shader names its invocation") {
    const std::string source = sim::gpuFrustumCullingShader();
    REQUIRE(source.find("#version 430") != std::string::npos);
    REQUIRE(source.find("gl_GlobalInvocationID") != std::string::npos);
    REQUIRE(source.find("uViewProj") != std::string::npos);
}
