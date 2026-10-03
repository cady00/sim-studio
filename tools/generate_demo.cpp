#include "sim/core/DemoScene.hpp"
#include "sim/io/ProjectStore.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

bool writeText(const std::filesystem::path& path, const char* text) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        std::cerr << "could not write " << path << "\n";
        return false;
    }
    out << text;
    return static_cast<bool>(out);
}

}  // namespace

int main(int argc, char** argv) {
    const std::filesystem::path project = argc > 1 ? argv[1] : "samples/conveyor_line.simproj";
    const std::filesystem::path scriptDir = argc > 2 ? argv[2] : "samples/scripts";
    std::error_code errorCode;
    std::filesystem::create_directories(project.parent_path(), errorCode);
    std::filesystem::create_directories(scriptDir, errorCode);

    sim::Scene scene;
    sim::buildConveyorLine(scene);
    const sim::ProjectResult saved = sim::ProjectStore::save(scene, project.string());
    if (!saved.ok) {
        std::cerr << saved.message << "\n";
        return 1;
    }
    if (!writeText(scriptDir / "pusher.py", sim::pusherScriptSource()) ||
        !writeText(scriptDir / "sequence.py", sim::sequenceScriptSource()) ||
        !writeText(scriptDir / "counter.py", sim::counterScriptSource())) {
        return 1;
    }
    std::cout << "wrote " << project << " and scripts in " << scriptDir << "\n";
    return 0;
}
