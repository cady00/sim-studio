#pragma once

#include "sim/core/Scene.hpp"

#include <string>

namespace sim {

struct ProjectResult {
    bool ok = false;
    std::string message;
};

// .simproj is a ZIP (store method, no compression) of project.json + mesh cache.
class ProjectStore {
public:
    static ProjectResult save(const Scene& scene, const std::string& path);
    static ProjectResult load(const std::string& path, Scene& scene);
};

}  // namespace sim
