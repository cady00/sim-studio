#pragma once

#include "sim/core/Scene.hpp"

#include <atomic>
#include <functional>
#include <string>

namespace sim {

struct ImportResult {
    bool ok = false;
    std::string message;
    int nodeCount = 0;
    int meshCount = 0;
    int triangleCount = 0;
    double elapsedMs = 0;
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
