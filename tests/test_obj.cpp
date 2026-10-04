#include "sim/io/GeometryKernel.hpp"
#include "sim/io/ObjImporter.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>

TEST_CASE("obj importer triangulates a quad") {
    const auto path = std::filesystem::temp_directory_path() / "simstudio_quad.obj";
    {
        std::ofstream out(path);
        out << "o Panel\n";
        out << "v 0 0 0\nv 10 0 0\nv 10 5 0\nv 0 5 0\n";
        out << "f 1 2 3 4\n";
    }
    sim::Scene scene;
    sim::ObjImporter importer;
    const sim::ImportResult result = importer.importFile(path.string(), scene);
    REQUIRE(result.ok);
    REQUIRE(result.triangleCount == 2);
    REQUIRE(scene.root()->children.size() == 1);
    REQUIRE(scene.root()->children.front()->name == "Panel");
    std::filesystem::remove(path);
}

TEST_CASE("gltf importer reports that it is not in phase 1") {
    sim::Scene scene;
    sim::GltfImporter importer;
    const sim::ImportResult result = importer.importFile("robot.gltf", scene);
    REQUIRE_FALSE(result.ok);
    REQUIRE(result.message.find("Phase 1") != std::string::npos);
}

TEST_CASE("geometry kernel names occt and keeps obj on the in-house parser") {
    const auto kernel = sim::createGeometryKernel();
    REQUIRE(std::string(kernel->backendName()) == "occt");
    const auto step = kernel->createImporter("step");
    REQUIRE(step);
    REQUIRE(std::string(step->backendName()) == "occt");
    REQUIRE(std::string(step->format()) == "step");
    const auto obj = kernel->createImporter("obj");
    REQUIRE(obj);
    REQUIRE(std::string(obj->backendName()) == "builtin");
    REQUIRE(kernel->createImporter("iges") == nullptr);
}
