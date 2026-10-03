#pragma once

#include "sim/core/Scene.hpp"

#include <functional>
#include <memory>
#include <string>

namespace sim {

// Bullet 3 sits behind this seam. A later PhysX backend can replace it without
// changing the scene, signals, or Python API.
class IPhysicsEngine {
public:
    virtual ~IPhysicsEngine() = default;
    virtual const char* backendName() const = 0;
    virtual bool build(Scene& scene, std::string& error) = 0;
    virtual void step(double dtSeconds) = 0;
    virtual void shutdown() = 0;
    // Used when a script spawns or deletes a body while the world is alive.
    virtual void adoptSpawned(const std::string&) {}
    virtual void releaseBody(const std::string&) {}
};

enum class SimState { Stopped, Playing, Paused };

// Fixed-step playback. Rest poses are captured on play and restored by stop/reset.
class Simulation {
public:
    static constexpr double kFixedDt = 1.0 / 60.0;

    Simulation(Scene& scene, IPhysicsEngine& engine);
    void setHooks(std::function<void()> onStart, std::function<void(double)> onStep, std::function<void()> onStop);

    void play();
    void pause();
    void stop();
    void reset();
    void tick(double realDtSeconds);

    SimState state() const { return state_; }
    double time() const { return time_; }
    double speed() const { return speed_; }
    void setSpeed(double factor);
    const std::string& lastError() const { return lastError_; }

private:
    void captureRest();
    void restoreRest();
    void discardTransients();

    Scene& scene_;
    IPhysicsEngine& engine_;
    SimState state_ = SimState::Stopped;
    double time_ = 0;
    double speed_ = 1;
    double accumulator_ = 0;
    std::string lastError_;
    std::vector<std::pair<std::string, Transform>> rest_;
    std::function<void()> onStart_;
    std::function<void(double)> onStep_;
    std::function<void()> onStop_;
};

std::unique_ptr<IPhysicsEngine> createPhysicsEngine();

}  // namespace sim
