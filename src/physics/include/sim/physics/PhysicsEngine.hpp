#pragma once

#include <memory>
#include <string>

namespace sim {

// Phase 2 plugs NVIDIA PhysX 5 behind this seam (rigid bodies, joints, sensors, conveyors).
class IPhysicsEngine {
public:
    virtual ~IPhysicsEngine() = default;
    virtual const char* backendName() const = 0;
    virtual bool available() const = 0;
    virtual bool initialize(std::string& error) = 0;
    virtual void step(double dtSeconds) = 0;
    virtual void shutdown() = 0;
};

class NullPhysicsEngine final : public IPhysicsEngine {
public:
    const char* backendName() const override { return "null"; }
    bool available() const override { return false; }
    bool initialize(std::string& error) override {
        error = "PhysX is not linked in Phase 1";
        return false;
    }
    void step(double) override {}
    void shutdown() override {}
};

std::unique_ptr<IPhysicsEngine> createPhysicsEngine();

}  // namespace sim
