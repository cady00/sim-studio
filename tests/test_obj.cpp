#include "sim/io/AssetImport.hpp"
#include "sim/io/GeometryKernel.hpp"
#include "sim/io/ObjImporter.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>

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

TEST_CASE("gltf importer reads a triangle instead of refusing the format") {
    sim::Scene source;
    auto node = source.makeNode(sim::NodeKind::Part, "tri");
    node->meshId = source.addMesh(sim::makeBoxMesh({10, 20, 5}));
    source.addChild(source.root(), node);
    const auto path = std::filesystem::temp_directory_path() / "simstudio_box.glb";
    std::string error;
    REQUIRE(sim::exportSceneGltf(source, path.string(), error));
    std::ifstream in(path, std::ios::binary);
    char magic[4] = {};
    in.read(magic, 4);
    REQUIRE(std::string(magic, 4) == "glTF");
    sim::Scene scene;
    sim::GltfImporter importer;
    const sim::ImportResult result = importer.importFile(path.string(), scene);
    REQUIRE(result.ok);
    REQUIRE(result.triangleCount == 12);
    std::filesystem::remove(path);
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
    const auto iges = kernel->createImporter("iges");
    REQUIRE(iges);
    REQUIRE(std::string(iges->format()) == "iges");
    REQUIRE(std::string(iges->backendName()) == "occt");
    REQUIRE(kernel->createImporter("igs"));
    const auto sample = std::filesystem::temp_directory_path() / "simstudio_kernel_box.iges";
    std::string error;
    REQUIRE(sim::writeSampleShape("iges", sample.string(), error));
    sim::Scene imported;
    const sim::ImportResult read = iges->importFile(sample.string(), imported);
    REQUIRE(read.ok);
    REQUIRE(read.triangleCount == 12);
    REQUIRE(imported.root()->children.size() == 1);
    std::filesystem::remove(sample);
}
