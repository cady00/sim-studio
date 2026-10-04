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

    // autosave.simproj plus session.pid. A dead pid means the last session crashed.
    static ProjectResult autosave(const Scene& scene, const std::string& directory);
    static void markSession(const std::string& directory);
    static void clearSession(const std::string& directory);
    static bool sessionInterrupted(const std::string& directory);
    static ProjectResult restoreAutosave(const std::string& directory, Scene& scene);
};

}  // namespace sim
