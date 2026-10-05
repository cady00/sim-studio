#pragma once

#include "sim/core/Scene.hpp"
#include "sim/physics/PhysicsEngine.hpp"

#include <nlohmann/json.hpp>

#include <string>

namespace sim {

// Typed operations for agents. Python `simstudio` calls the same scene objects.
// A later MCP transport exposes this catalog; it must not grow a second API.
// Scripts still share only the signal table.
struct CommandResult {
    bool ok = false;
    std::string error;
    nlohmann::json value = nlohmann::json::object();
};

class CommandExecutor {
public:
    CommandExecutor(Scene& scene, Simulation* simulation, IPhysicsEngine* physics);

    // One object: {"op":"set_bool","name":"go","value":true}
    CommandResult execute(const nlohmann::json& command) const;
    static nlohmann::json catalog();

private:
    Scene& scene_;
    Simulation* simulation_ = nullptr;
    IPhysicsEngine* physics_ = nullptr;
};

}  // namespace sim
