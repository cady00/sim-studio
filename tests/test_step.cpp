#include "sim/io/ProjectStore.hpp"
#include "sim/io/StepImporter.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>

using Catch::Approx;

namespace {

std::shared_ptr<sim::SceneNode> findName(const std::shared_ptr<sim::SceneNode>& node, const std::string& name) {
    if (!node) {
        return {};
    }
    if (node->name == name || node->name.find(name) != std::string::npos) {
        return node;
    }
    for (const auto& child : node->children) {
        if (auto found = findName(child, name)) {
            return found;
        }
    }
    return {};
}

void printTree(const sim::SceneNode& node, int depth) {
    std::cout << std::string(static_cast<std::size_t>(depth) * 2, ' ') << node.name << " children=" << node.children.size()
              << " t=(" << node.local.translation.x << "," << node.local.translation.y << "," << node.local.translation.z
              << ")\n";
    for (const auto& child : node.children) {
        if (child) {
            printTree(*child, depth + 1);
        }
    }
}

}  // namespace

TEST_CASE("sample fixture STEP keeps the assembly and shared pin mesh") {
    const std::filesystem::path path = std::filesystem::path(SIM_SOURCE_DIR) / "samples/step/sample_fixture.step";
    REQUIRE(std::filesystem::exists(path));
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult result = importer.importFile(path.string(), scene);
    INFO(result.message);
    REQUIRE(result.ok);
    REQUIRE(result.triangleCount == 136);
    const auto fixture = findName(scene.root(), "Fixture");
    REQUIRE(fixture);
    REQUIRE(fixture->kind == sim::NodeKind::Assembly);
    const auto base = findName(fixture, "BasePlate");
    const auto pinA = findName(fixture, "Pin_A");
    const auto pinB = findName(fixture, "Pin_B");
    REQUIRE(base);
    REQUIRE(pinA);
    REQUIRE(pinB);
    REQUIRE(pinA->meshId == pinB->meshId);
    const auto pinMesh = scene.mesh(pinA->meshId);
    REQUIRE(pinMesh);
    REQUIRE(pinMesh->triangleCount() == 124);
    const auto baseMesh = scene.mesh(base->meshId);
    REQUIRE(baseMesh);
    REQUIRE(baseMesh->bounds.max.x == Approx(120).margin(2));
    REQUIRE(baseMesh->bounds.max.y == Approx(80).margin(2));
    REQUIRE(baseMesh->bounds.max.z == Approx(12).margin(2));
    REQUIRE(pinA->local.translation.x == Approx(30).margin(0.5));
    REQUIRE(pinA->local.translation.y == Approx(25).margin(0.5));
    REQUIRE(pinA->local.translation.z == Approx(12).margin(0.5));

    const auto project = std::filesystem::temp_directory_path() / "simstudio_fixture.simproj";
    REQUIRE(sim::ProjectStore::save(scene, project.string()).ok);
    sim::Scene loaded;
    REQUIRE(sim::ProjectStore::load(project.string(), loaded).ok);
    REQUIRE(findName(loaded.root(), "Pin_B"));
    REQUIRE(loaded.meshes().size() == scene.meshes().size());
    std::filesystem::remove(project);
}

TEST_CASE("optional local robot STEP exposes the NX assembly") {
    const char* env = std::getenv("SIM_LOCAL_STEP");
    if (env == nullptr || env[0] == '\0') {
        SUCCEED("SIM_LOCAL_STEP is not set; vendor model stays outside the repo");
        return;
    }
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult result = importer.importFile(env, scene, [](const sim::ImportProgress& progress) {
        std::cout << progress.stageKey << " " << progress.percent << " " << progress.current << "/" << progress.total
                  << "\n";
    }, nullptr);
    std::cout << "local import nodes=" << result.nodeCount << " meshes=" << result.meshCount
              << " triangles=" << result.triangleCount << " ms=" << result.elapsedMs << "\n";
    if (scene.root()) {
        printTree(*scene.root(), 0);
    }
    INFO(result.message);
    REQUIRE(result.ok);
    const auto rootProduct = findName(scene.root(), "394.088-10.104.00");
    REQUIRE(rootProduct);
    REQUIRE(rootProduct->kind == sim::NodeKind::Assembly);
    REQUIRE(findName(rootProduct, "394.088-10.105.10"));
    REQUIRE(findName(rootProduct, "394.088-10.104.20"));
    REQUIRE(findName(rootProduct, "394.088-10.105.30"));
    REQUIRE(findName(rootProduct, "394.088-10.105.40"));
    REQUIRE(findName(rootProduct, "394.088-10.104.50"));
    REQUIRE(result.triangleCount > 1000);
}
