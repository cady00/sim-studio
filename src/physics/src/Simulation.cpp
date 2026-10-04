#include "sim/physics/PhysicsEngine.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

void captureNode(const std::shared_ptr<SceneNode>& node, std::vector<std::pair<std::string, Transform>>& rest) {
    if (!node || node->transient) {
        return;
    }
    rest.emplace_back(node->id, node->local);
    for (const auto& child : node->children) {
        captureNode(child, rest);
    }
}

void collectTransient(const std::shared_ptr<SceneNode>& node, std::vector<std::string>& ids) {
    if (!node) {
        return;
    }
    if (node->transient) {
        ids.push_back(node->id);
    }
    for (const auto& child : node->children) {
        collectTransient(child, ids);
    }
}

}  // namespace

Simulation::Simulation(Scene& scene, IPhysicsEngine& engine) : scene_(scene), engine_(engine) {}

void Simulation::setHooks(std::function<void()> onStart, std::function<void(double)> onStep, std::function<void()> onStop) {
    onStart_ = std::move(onStart);
    onStep_ = std::move(onStep);
    onStop_ = std::move(onStop);
}

void Simulation::setSpeed(double factor) { speed_ = std::clamp(factor, 0.0, 16.0); }

void Simulation::captureRest() {
    rest_.clear();
    captureNode(scene_.root(), rest_);
}

void Simulation::restoreRest() {
    for (const auto& [id, pose] : rest_) {
        if (auto node = scene_.find(id)) {
            node->local = pose;
        }
    }
}

void Simulation::discardTransients() {
    std::vector<std::string> ids;
    collectTransient(scene_.root(), ids);
    for (const std::string& id : ids) {
        scene_.removeNode(id);
    }
}

void Simulation::play() {
    if (state_ == SimState::Playing) {
        return;
    }
    if (state_ == SimState::Stopped) {
        captureRest();
        if (!engine_.build(scene_, lastError_)) {
            state_ = SimState::Stopped;
            return;
        }
        if (onStart_) {
            onStart_();
        }
    }
    state_ = SimState::Playing;
}

void Simulation::pause() {
    if (state_ == SimState::Playing) {
        state_ = SimState::Paused;
    }
}

void Simulation::stop() {
    if (state_ == SimState::Stopped) {
        return;
    }
    if (onStop_) {
        onStop_();
    }
    engine_.shutdown();
    discardTransients();
    restoreRest();
    time_ = 0;
    accumulator_ = 0;
    state_ = SimState::Stopped;
    scene_.notify(ChangeKind::Structure);
}

void Simulation::reset() { stop(); }

void Simulation::tick(double realDtSeconds) {
    if (state_ != SimState::Playing) {
        return;
    }
    const double stepDt = scene_.physicsDoc.timestep > 1e-4 && scene_.physicsDoc.timestep < 0.1 ? scene_.physicsDoc.timestep
                                                                                                : kFixedDt;
    accumulator_ += std::min(std::max(realDtSeconds, 0.0), 2.0) * speed_;
    int steps = 0;
    while (accumulator_ >= stepDt && steps < 240) {
        if (onStep_) {
            onStep_(stepDt);
        }
        engine_.step(stepDt);
        time_ += stepDt;
        accumulator_ -= stepDt;
        ++steps;
    }
    scene_.notify(ChangeKind::Simulation);
}

}  // namespace sim
