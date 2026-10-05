#pragma once

#include "sim/physics/PhysicsEngine.hpp"

#include <memory>

namespace sim {

class JoltPhysicsEngine final : public IPhysicsEngine {
public:
    JoltPhysicsEngine();
    ~JoltPhysicsEngine() override;

    const char* backendName() const override { return "jolt"; }
    bool build(Scene& scene, std::string& error) override;
    void step(double dtSeconds) override;
    void shutdown() override;
    void adoptSpawned(const std::string& id) override;
    void releaseBody(const std::string& id) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sim
