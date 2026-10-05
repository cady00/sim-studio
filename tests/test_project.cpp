#include "sim/io/ProjectStore.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>

using Catch::Approx;

TEST_CASE("project zip round-trips hierarchy and meshes") {
    sim::Scene scene;
    scene.name = "Cell";
    scene.sourceFiles.push_back("fixture.step");
    auto assembly = scene.makeNode(sim::NodeKind::Assembly, "Fixture");
    assembly->local.translation = {1, 2, 3};
    assembly->local.rotation = sim::Quat::fromEulerXYZ(0, 0, sim::deg2rad(90));
    scene.addChild(scene.root(), assembly);
    auto mesh = std::make_shared<sim::MeshData>();
    mesh->positions = {0, 0, 0, 4, 0, 0, 0, 5, 0};
    mesh->normals = {0, 0, 1, 0, 0, 1, 0, 0, 1};
    mesh->indices = {0, 1, 2};
    auto part = scene.makeNode(sim::NodeKind::Part, "BasePlate");
    part->color = {0.2, 0.3, 0.4};
    part->meshId = scene.addMesh(mesh);
    scene.addChild(assembly, part);

    const auto path = std::filesystem::temp_directory_path() / "simstudio_roundtrip.simproj";
    const sim::ProjectResult saved = sim::ProjectStore::save(scene, path.string());
    REQUIRE(saved.ok);

    sim::Scene loaded;
    const sim::ProjectResult opened = sim::ProjectStore::load(path.string(), loaded);
    REQUIRE(opened.ok);
    REQUIRE(loaded.name == "Cell");
    REQUIRE(loaded.sourceFiles.size() == 1);
    REQUIRE(loaded.root()->children.size() == 1);
    const auto loadedAssembly = loaded.root()->children.front();
    REQUIRE(loadedAssembly->name == "Fixture");
    REQUIRE(loadedAssembly->local.translation.z == Approx(3));
    REQUIRE(loadedAssembly->children.size() == 1);
    const auto loadedPart = loadedAssembly->children.front();
    REQUIRE(loadedPart->name == "BasePlate");
    REQUIRE(loadedPart->color.y == Approx(0.3));
    const auto loadedMesh = loaded.mesh(loadedPart->meshId);
    REQUIRE(loadedMesh);
    REQUIRE(loadedMesh->triangleCount() == 1);
    REQUIRE(loadedMesh->positions[3] == Approx(4));
    std::filesystem::remove(path);
}
