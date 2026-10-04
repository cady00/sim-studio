#include "sim/physics/PhysicsEngine.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace sim {
namespace {

struct BodyState {
    std::string id;
    bool dynamic = false;
    Vec3 velocity{};
};

struct JointState {
    std::string id;
    Vec3 axis{1, 0, 0};
    bool hinge = false;
    Vec3 restTranslation{};
    Quat restRotation = Quat::identity();
    double position = 0;
    double speed = 0;
};

Vec3 worldTranslation(const SceneNode& node) {
    const Mat4 world = node.worldMatrix();
    return {world(3, 0), world(3, 1), world(3, 2)};
}

struct Box {
    Vec3 center{};
    Vec3 half{};
};

Box boxOf(const SceneNode& node) {
    Box box;
    box.center = worldTranslation(node);
    box.half = {std::abs(node.mech.halfExtents.x), std::abs(node.mech.halfExtents.y), std::abs(node.mech.halfExtents.z)};
    if (node.mech.shape == CollisionShape::Sphere) {
        const double r = std::abs(node.mech.radius);
        box.half = {r, r, r};
    }
    return box;
}

bool separated(const Box& a, const Box& b, Vec3& normal, double& depth) {
    const Vec3 delta = b.center - a.center;
    const double px = a.half.x + b.half.x - std::abs(delta.x);
    const double py = a.half.y + b.half.y - std::abs(delta.y);
    const double pz = a.half.z + b.half.z - std::abs(delta.z);
    if (px <= 0 || py <= 0 || pz <= 0) {
        return false;
    }
    depth = px;
    normal = {delta.x < 0 ? -1.0 : 1.0, 0, 0};
    if (py < depth) {
        depth = py;
        normal = {0, delta.y < 0 ? -1.0 : 1.0, 0};
    }
    if (pz < depth) {
        depth = pz;
        normal = {0, 0, delta.z < 0 ? -1.0 : 1.0};
    }
    return true;
}

bool rayHitsBox(Vec3 origin, Vec3 direction, double range, const Box& box, double& distance) {
    const Vec3 dir = normalize(direction);
    double tEnter = 0;
    double tExit = range;
    const double originC[3] = {origin.x, origin.y, origin.z};
    const double dirC[3] = {dir.x, dir.y, dir.z};
    const double centerC[3] = {box.center.x, box.center.y, box.center.z};
    const double halfC[3] = {box.half.x, box.half.y, box.half.z};
    for (int axis = 0; axis < 3; ++axis) {
        const double minB = centerC[axis] - halfC[axis];
        const double maxB = centerC[axis] + halfC[axis];
        if (std::abs(dirC[axis]) < 1e-12) {
            if (originC[axis] < minB || originC[axis] > maxB) {
                return false;
            }
            continue;
        }
        double t0 = (minB - originC[axis]) / dirC[axis];
        double t1 = (maxB - originC[axis]) / dirC[axis];
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        tEnter = std::max(tEnter, t0);
        tExit = std::min(tExit, t1);
        if (tEnter > tExit) {
            return false;
        }
    }
    if (tExit < 0 || tEnter > range) {
        return false;
    }
    distance = tEnter >= 0 ? tEnter : 0;
    return true;
}

void integrateJoint(double& position, double& speed, const Mech& mech, double dt) {
    if (mech.drive == DriveMode::None || dt <= 0) {
        return;
    }
    const double maxSpeed = std::abs(mech.maxSpeed);
    const double maxDelta = std::max(0.0, mech.maxAccel) * dt;
    double desired = 0;
    if (mech.drive == DriveMode::Speed) {
        desired = std::clamp(mech.target, -maxSpeed, maxSpeed);
    } else if (dt > 0) {
        desired = std::clamp((mech.target - position) / dt, -maxSpeed, maxSpeed);
    }
    desired = std::clamp(desired, speed - maxDelta, speed + maxDelta);
    speed = std::clamp(desired, -maxSpeed, maxSpeed);
    position += speed * dt;
    if (mech.limited) {
        const double lower = std::min(mech.lower, mech.upper);
        const double upper = std::max(mech.lower, mech.upper);
        if (position < lower) {
            position = lower;
            speed = 0;
        } else if (position > upper) {
            position = upper;
            speed = 0;
        }
    }
}

class BuiltinPhysicsEngine final : public IPhysicsEngine {
public:
    const char* backendName() const override { return "builtin"; }

    bool build(Scene& scene, std::string& error) override {
        shutdown();
        scene_ = &scene;
        std::vector<std::shared_ptr<SceneNode>> stack{scene.root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (mechCreatesBody(node->mech.kind)) {
                BodyState body;
                body.id = node->id;
                body.dynamic = node->mech.kind == MechKind::Rigid && !node->mech.kinematic;
                bodies_.push_back(std::move(body));
            }
            if (mechIsJoint(node->mech.kind)) {
                JointState joint;
                joint.id = node->id;
                joint.axis = length(node->mech.axis) > 1e-8 ? normalize(node->mech.axis) : Vec3{1, 0, 0};
                joint.hinge = node->mech.kind == MechKind::Hinge;
                joint.restTranslation = node->local.translation;
                joint.restRotation = node->local.rotation;
                joints_.push_back(std::move(joint));
            }
        }
        error.clear();
        return true;
    }

    void step(double dtSeconds) override {
        if (scene_ == nullptr || dtSeconds <= 0) {
            return;
        }
        const double dt = dtSeconds;
        for (JointState& joint : joints_) {
            auto node = scene_->find(joint.id);
            if (!node) {
                continue;
            }
            integrateJoint(joint.position, joint.speed, node->mech, dt);
            if (joint.hinge) {
                const Quat delta = Quat::fromAxisAngle(joint.axis, deg2rad(joint.position));
                node->local.rotation = (delta * joint.restRotation).normalized();
                node->local.translation = joint.restTranslation;
            } else {
                node->local.translation = joint.restTranslation + joint.axis * joint.position;
                node->local.rotation = joint.restRotation;
            }
        }

        constexpr double kGravity = -9810.0;
        for (BodyState& body : bodies_) {
            if (!body.dynamic) {
                continue;
            }
            auto node = scene_->find(body.id);
            if (!node) {
                continue;
            }
            body.velocity.z += kGravity * dt;
            node->local.translation = node->local.translation + body.velocity * dt;
        }
        resolveContacts();
        updateSensors();
        updateSources(dt);
        updateSinks();
    }

    void shutdown() override {
        bodies_.clear();
        joints_.clear();
        sourceClock_.clear();
        scene_ = nullptr;
    }

    void adoptSpawned(const std::string& id) override {
        if (scene_ == nullptr) {
            return;
        }
        auto node = scene_->find(id);
        if (!node) {
            return;
        }
        for (const BodyState& body : bodies_) {
            if (body.id == id) {
                return;
            }
        }
        if (mechCreatesBody(node->mech.kind)) {
            BodyState body;
            body.id = id;
            body.dynamic = node->mech.kind == MechKind::Rigid && !node->mech.kinematic;
            bodies_.push_back(std::move(body));
        }
    }

    void releaseBody(const std::string& id) override {
        bodies_.erase(std::remove_if(bodies_.begin(), bodies_.end(), [&](const BodyState& body) { return body.id == id; }),
                      bodies_.end());
    }

private:
    void resolveContacts() {
        for (int iteration = 0; iteration < 4; ++iteration) {
            for (BodyState& body : bodies_) {
                if (!body.dynamic) {
                    continue;
                }
                auto node = scene_->find(body.id);
                if (!node) {
                    continue;
                }
                const Box moving = boxOf(*node);
                for (const BodyState& other : bodies_) {
                    if (other.id == body.id || other.dynamic) {
                        continue;
                    }
                    auto obstacle = scene_->find(other.id);
                    if (!obstacle) {
                        continue;
                    }
                    Vec3 normal;
                    double depth = 0;
                    if (!separated(moving, boxOf(*obstacle), normal, depth)) {
                        continue;
                    }
                    node->local.translation = node->local.translation - normal * (depth + 0.01);
                    const double into = dot(body.velocity, normal);
                    if (into > 0) {
                        body.velocity = body.velocity - normal * into;
                    }
                    if (obstacle->mech.kind == MechKind::Conveyor) {
                        const Vec3 belt = normalize(obstacle->mech.direction) * obstacle->mech.speed;
                        body.velocity.x = belt.x;
                        body.velocity.y = belt.y;
                    }
                }
            }
        }
    }

    void updateSensors() {
        std::vector<std::shared_ptr<SceneNode>> stack{scene_->root()};
        std::vector<std::shared_ptr<SceneNode>> nodes;
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            nodes.push_back(node);
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
        }
        for (const auto& node : nodes) {
            if (node->mech.kind == MechKind::CollisionSensor) {
                const Box sensor = boxOf(*node);
                bool hit = false;
                for (const BodyState& body : bodies_) {
                    if (body.id == node->id) {
                        continue;
                    }
                    auto other = scene_->find(body.id);
                    if (!other || other->mech.kind == MechKind::CollisionSensor) {
                        continue;
                    }
                    Vec3 normal;
                    double depth = 0;
                    if (separated(sensor, boxOf(*other), normal, depth)) {
                        hit = true;
                        break;
                    }
                }
                const std::string name = node->mech.signal.empty() ? node->name : node->mech.signal;
                scene_->upsertSignal(name, SignalType::Bool).flag = hit;
            }
            if (node->mech.kind != MechKind::LightBarrier && node->mech.kind != MechKind::DistanceSensor) {
                continue;
            }
            Vec3 axis = length(node->mech.axis) > 1e-8 ? normalize(node->mech.axis) : Vec3{0, 1, 0};
            axis = node->local.rotation.rotate(axis);
            const double range = std::max(node->mech.range, 1.0);
            const Vec3 origin = worldTranslation(*node);
            double best = range;
            bool blocked = false;
            for (const BodyState& body : bodies_) {
                auto other = scene_->find(body.id);
                if (!other) {
                    continue;
                }
                double distance = 0;
                if (rayHitsBox(origin, axis, range, boxOf(*other), distance) && distance < best) {
                    best = distance;
                    blocked = true;
                }
            }
            const std::string name = node->mech.signal.empty() ? node->name : node->mech.signal;
            scene_->upsertSignal(name, SignalType::Bool).flag = blocked;
            if (!node->mech.valueSignal.empty()) {
                scene_->upsertSignal(node->mech.valueSignal, SignalType::Real).real = best;
            }
        }
    }

    void updateSources(double dt) {
        std::vector<std::shared_ptr<SceneNode>> stack{scene_->root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (node->mech.kind != MechKind::Source || node->mech.prototype.empty()) {
                continue;
            }
            double& clock = sourceClock_[node->id];
            clock += dt;
            const double interval = std::max(node->mech.interval, 0.05);
            if (clock < interval) {
                continue;
            }
            clock = 0;
            auto prototype = scene_->findByName(node->mech.prototype);
            if (!prototype) {
                continue;
            }
            if (auto clone = scene_->spawnClone(prototype, node->local)) {
                adoptSpawned(clone->id);
            }
        }
    }

    void updateSinks() {
        std::vector<std::string> remove;
        std::vector<std::shared_ptr<SceneNode>> stack{scene_->root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (node->mech.kind != MechKind::Sink) {
                continue;
            }
            const Box sink = boxOf(*node);
            for (const BodyState& body : bodies_) {
                if (!body.dynamic) {
                    continue;
                }
                auto other = scene_->find(body.id);
                if (!other) {
                    continue;
                }
                Vec3 normal;
                double depth = 0;
                if (separated(sink, boxOf(*other), normal, depth)) {
                    remove.push_back(body.id);
                }
            }
        }
        for (const std::string& id : remove) {
            releaseBody(id);
            scene_->removeNode(id);
        }
    }

    Scene* scene_ = nullptr;
    std::vector<BodyState> bodies_;
    std::vector<JointState> joints_;
    std::unordered_map<std::string, double> sourceClock_;
};

}  // namespace

std::unique_ptr<IPhysicsEngine> createBuiltinPhysicsEngine() { return std::make_unique<BuiltinPhysicsEngine>(); }

}  // namespace sim
