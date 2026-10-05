#pragma once

#include "sim/core/Scene.hpp"

#include <atomic>
#include <functional>
#include <string>
#include <vector>

namespace sim {

struct ImportResult {
    bool ok = false;
    std::string message;
    int nodeCount = 0;
    int meshCount = 0;
    int triangleCount = 0;
    double elapsedMs = 0;
    // One line: schema, file products / assembly occurrences / solids, scene nodes / meshes / triangles / empty parts.
    std::string report;
    // Each entry names the file item and why it did not become a mesh, or quotes an Open CASCADE check.
    std::vector<std::string> warnings;
    int fileProductCount = 0;
    int fileOccurrenceCount = 0;
    int fileSolidCount = 0;
    int partCount = 0;
    int assemblyCount = 0;
    int emptyPartCount = 0;
};

struct ImportProgress {
    // Negative percent means the stage length is unknown (busy indicator).
    int percent = -1;
    std::string stageKey;
    int current = 0;
    int total = 0;
};

using ImportProgressFn = std::function<void(const ImportProgress&)>;

class IGeometryImporter {
public:
    virtual ~IGeometryImporter() = default;
    virtual ImportResult importFile(const std::string& path, Scene& scene) = 0;
};

}  // namespace sim
