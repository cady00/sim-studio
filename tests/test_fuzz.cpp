#include "sim/core/LineModel.hpp"
#include "sim/io/AssetImport.hpp"
#include "sim/io/ObjImporter.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/motion/Olp.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

std::filesystem::path scratch() {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "simstudio-fuzz";
    std::filesystem::create_directories(dir);
    return dir;
}

void writeBytes(const std::filesystem::path& path, const std::string& bytes) {
    std::ofstream out(path, std::ios::binary);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

std::string garbage(std::size_t size, unsigned seed) {
    std::string bytes(size, '\0');
    unsigned state = seed;
    for (char& byte : bytes) {
        state = state * 1664525u + 1013904223u;
        byte = static_cast<char>(state >> 16);
    }
    return bytes;
}

}  // namespace

TEST_CASE("fuzz importers reject empty truncated and oversized files") {
    const std::filesystem::path dir = scratch();
    auto registry = sim::createBuiltinRegistry();
    const std::string blob = garbage(256 * 1024, 7);
    const std::vector<std::pair<std::string, std::string>> files = {
        {"empty.stl", ""},
        {"truncated.stl", std::string(40, '\0')},
        {"noise.stl", blob},
        {"empty.obj", ""},
        {"bad.obj", "v 1 2\nf 99999999999999999999/1/1 1 2\n"},
        {"noise.obj", blob},
        {"empty.dxf", ""},
        {"binary.dxf", std::string("noise\0inside", 11)},
        {"noise.dxf", blob},
        {"empty.gltf", ""},
        {"broken.gltf", "{"},
        {"noise.gltf", blob},
        {"empty.glb", ""},
        {"short.glb", std::string("glTF\x02\x00\x00\x00", 8) + std::string("\xff\xff\xff\x7f\x4a\x53\x4f\x4e", 8)},
        {"noise.glb", std::string("glTF", 4) + blob},
        {"empty.urdf", ""},
        {"broken.urdf", "<robot><link name=\"a\">"},
        {"noise.urdf", blob},
        {"empty.xml", ""},
        {"broken.xml", "<plant><cell>"},
        {"noise.xml", blob},
        {"noise.stp", std::string(128, '\x01')},
    };
    {
        std::string claimed(84, '\0');
        claimed[80] = '\xff';
        claimed[81] = '\xff';
        claimed[82] = '\xff';
        claimed[83] = '\x7f';
        writeBytes(dir / "huge.stl", claimed);
        sim::Scene scene;
        const sim::AssetImportResult result = registry->importPath((dir / "huge.stl").string(), scene);
        REQUIRE_FALSE(result.ok);
        REQUIRE(result.message.find("triangle") != std::string::npos);
    }
    for (const auto& file : files) {
        const std::filesystem::path path = dir / file.first;
        writeBytes(path, file.second);
        sim::Scene scene;
        const sim::AssetImportResult result = registry->importPath(path.string(), scene);
        INFO(file.first);
        REQUIRE_FALSE(result.ok);
        REQUIRE_FALSE(result.message.empty());
    }

    sim::DxfResult dxf = sim::parseDxf(garbage(64 * 1024, 3), 1);
    REQUIRE(dxf.lines.size() < 100000);

    sim::KinematicResource robot;
    robot.name = "kr420";
    robot.joints.resize(6);
    auto post = sim::createOlpProgram("kuka");
    REQUIRE(post != nullptr);
    sim::RobotPath path;
    std::string error;
    REQUIRE_FALSE(post->importProgram(&robot, garbage(128 * 1024, 9), garbage(1024, 11), path, error));

    sim::Scene project;
    const sim::ProjectResult loaded = sim::ProjectStore::load((dir / "noise.stl").string(), project);
    REQUIRE_FALSE(loaded.ok);
}

TEST_CASE("fuzz a dead session restores the autosave") {
    const std::filesystem::path dir = scratch() / "recovery";
    std::filesystem::remove_all(dir);
    sim::Scene scene;
    scene.name = "cell";
    sim::WeldSpot spot;
    spot.id = "w1";
    spot.position = {10, 20, 30};
    scene.welds.push_back(spot);
    const sim::ProjectResult saved = sim::ProjectStore::autosave(scene, dir.string());
    REQUIRE(saved.ok);
    {
        std::ofstream pid(dir / "session.pid");
        pid << 2147483646;
    }
    REQUIRE(sim::ProjectStore::sessionInterrupted(dir.string()));
    sim::Scene restored;
    const sim::ProjectResult loaded = sim::ProjectStore::restoreAutosave(dir.string(), restored);
    REQUIRE(loaded.ok);
    REQUIRE(restored.name == "cell");
    REQUIRE(restored.welds.size() == 1);
    REQUIRE(restored.welds.front().id == "w1");
    sim::ProjectStore::clearSession(dir.string());
    REQUIRE_FALSE(sim::ProjectStore::sessionInterrupted(dir.string()));
    sim::Scene missing;
    REQUIRE_FALSE(sim::ProjectStore::restoreAutosave(dir.string(), missing).ok);
}
