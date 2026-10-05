#pragma once

#include "sim/core/Scene.hpp"

#include <functional>
#include <memory>
#include <string>

namespace sim {

// Rigid-body world. Scene, signals, and the Python API do not include a physics SDK.
// Jolt is one backend. Builtin is the in-house solver and does not link Jolt.
// createPhysicsEngine() keeps Jolt when it was compiled in, so existing scenes
// do not change backend by surprise. Pass PhysicsBackend::Builtin to opt in.
//
// A six-axis robot does not belong in this class. Phase 3 adds an articulation
// solver (IRobotSolver) that writes joint angles onto scene nodes and may publish
// collision bodies through adoptSpawned. Generic hinge motors stay for slides,
// turntables, and other user-authored mechanisms.
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
// kFixedDt is the deterministic core an agent can replay. A seed and an ordered
// event log belong here later; they are not recorded yet. Headless batch runs
// should call this class directly and skip the Qt window.
class Simulation {
public:
    static constexpr double kFixedDt = 1.0 / 60.0;

    Simulation(Scene& scene, IPhysicsEngine& engine);
    void setHooks(std::function<void()> onStart, std::function<void(double)> onStep, std::function<void()> onStop);
    // Runs after each engine step, so scripts can read contacts produced by that step.
    void setAfterStep(std::function<void()> afterStep);

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
    std::function<void()> afterStep_;
};

// Not constructed yet. Phase 3 owns the motion planner and cycle-time model.
// RRS/RCS, if added, is an adapter in front of that planner, not the planner.
class IRobotSolver {
public:
    virtual ~IRobotSolver() = default;
    virtual const char* backendName() const = 0;
    virtual bool build(Scene& scene, std::string& error) = 0;
    virtual void step(double dtSeconds) = 0;
    virtual void shutdown() = 0;
};

enum class PhysicsBackend { Jolt, Builtin };

const char* physicsBackendName(PhysicsBackend backend);
std::unique_ptr<IPhysicsEngine> createPhysicsEngine();
std::unique_ptr<IPhysicsEngine> createPhysicsEngine(PhysicsBackend backend);

}  // namespace sim
