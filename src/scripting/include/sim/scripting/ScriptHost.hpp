#pragma once

#include "sim/core/Scene.hpp"

#include <functional>
#include <memory>
#include <string>

namespace sim {

// Embedded CPython. Scripts define on_start(), on_step(dt) and on_stop().
class IScriptHost {
public:
    virtual ~IScriptHost() = default;
    virtual const char* backendName() const = 0;
    virtual bool available() const = 0;
    virtual void bind(Scene* scene) = 0;
    virtual void setPhysicsHooks(std::function<void(const std::string&)> onSpawned,
                                 std::function<void(const std::string&)> onRemoved) = 0;
    virtual bool load(const std::string& source, std::string& error) = 0;
    virtual void onStart() = 0;
    virtual void onStep(double dtSeconds) = 0;
    virtual void onStop() = 0;
    virtual bool execute(const std::string& code, std::string& output, std::string& error) = 0;
    virtual std::string takeLog() = 0;
};

std::unique_ptr<IScriptHost> createScriptHost();

}  // namespace sim
