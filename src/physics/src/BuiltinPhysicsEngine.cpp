#include "sim/physics/PhysicsEngine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace sim {
namespace {

struct BodyState {
    std::string id;
    bool dynamic = false;
    bool riding = false;
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
    double accel = std::max(0.0, mech.maxAccel);
    const double mass = std::max(massKilograms(mech), 1e-4);
    if (mech.maxForce > 0 && mech.kind != MechKind::Hinge && mech.kind != MechKind::Ball) {
        accel = std::min(accel, mech.maxForce / mass);
    }
    if (mech.maxTorque > 0 && (mech.kind == MechKind::Hinge || mech.kind == MechKind::Ball)) {
        const double radius = std::max(length(mech.halfExtents), 1.0) * 0.001;
        const double inertia = mass * radius * radius;
        accel = std::min(accel, rad2deg(mech.maxTorque / inertia));
    }
    const double maxSpeed = std::abs(mech.maxSpeed);
    const double maxDelta = accel * dt;
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
            if (mechCreatesBody(node->mech.kind) && !node->mech.joinAncestor) {
                BodyState body;
                body.id = node->id;
                const BodyMotion motion = effectiveMotion(node->mech);
                body.dynamic = node->mech.kind == MechKind::Rigid && motion == BodyMotion::Free;
                body.riding = node->mech.kind == MechKind::Rigid && (motion == BodyMotion::Surface || node->mech.surfaceRide);
                rememberHull(scene, *node);
                bodies_.push_back(std::move(body));
            }
            if (mechIsJoint(node->mech.kind)) {
                JointState joint;
                joint.id = node->id;
                joint.axis = length(node->mech.axis) > 1e-8 ? normalize(node->mech.axis) : Vec3{1, 0, 0};
                joint.hinge = node->mech.kind == MechKind::Hinge || node->mech.kind == MechKind::Ball;
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
        scene_->physicsDoc.stats.contacts = 0;
        int continuous = 0;
        for (const BodyState& body : bodies_) {
            auto node = scene_->find(body.id);
            if (node && node->mech.continuous) {
                ++continuous;
            }
        }
        scene_->physicsDoc.stats.continuous = continuous;
        if (!scene_->physicsDoc.enabled) {
            return;
        }
        const double dt = dtSeconds;
        const auto started = std::chrono::steady_clock::now();
        stepMotors(dt);
        rideSurfaces(dt);
        const auto detected = std::chrono::steady_clock::now();
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
            if (node->mech.asleep && node->mech.allowSleep) {
                continue;
            }
            const int slices = node->mech.continuous ? 4 : 1;
            const double sub = dt / static_cast<double>(slices);
            for (int slice = 0; slice < slices; ++slice) {
                body.velocity.z += kGravity * sub;
                node->local.translation = node->local.translation + body.velocity * sub;
            }
            if (node->mech.allowSleep && length(body.velocity) < 2.0) {
                node->mech.asleep = true;
                body.velocity = {};
            }
        }
        resolveContacts();
        const auto solved = std::chrono::steady_clock::now();
        scene_->physicsDoc.stats.detectMs =
            std::chrono::duration<double, std::milli>(detected - started).count();
        scene_->physicsDoc.stats.solveMs = std::chrono::duration<double, std::milli>(solved - detected).count();
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
            const BodyMotion motion = effectiveMotion(node->mech);
            body.dynamic = node->mech.kind == MechKind::Rigid && motion == BodyMotion::Free;
            body.riding = node->mech.kind == MechKind::Rigid && (motion == BodyMotion::Surface || node->mech.surfaceRide);
            bodies_.push_back(std::move(body));
        }
    }

    void releaseBody(const std::string& id) override {
        bodies_.erase(std::remove_if(bodies_.begin(), bodies_.end(), [&](const BodyState& body) { return body.id == id; }),
                      bodies_.end());
    }

private:
    void rememberHull(Scene& scene, const SceneNode& node) {
        if (node.mech.shape != CollisionShape::ConvexHull && node.mech.shape != CollisionShape::Mesh &&
            node.mech.shape != CollisionShape::Extrusion) {
            return;
        }
        const std::string key = node.meshId.empty() ? node.id : node.meshId;
        for (const ConvexCacheEntry& entry : scene.convexCache) {
            if (entry.meshId == key) {
                return;
            }
        }
        ConvexCacheEntry entry;
        entry.meshId = key;
        entry.fingerprint = 1;
        ConvexPiece piece;
        const Vec3 h = node.mech.halfExtents;
        for (int corner = 0; corner < 8; ++corner) {
            piece.vertices.push_back({(corner & 1) ? h.x : -h.x, (corner & 2) ? h.y : -h.y, (corner & 4) ? h.z : -h.z});
        }
        entry.pieces.push_back(std::move(piece));
        scene.convexCache.push_back(std::move(entry));
    }

    Vec3 beltVelocity(const SceneNode& belt, const SceneNode& load) const {
        Vec3 direction = belt.mech.direction;
        if (length(direction) < 1e-8) {
            direction = {1, 0, 0};
        } else {
            direction = normalize(direction);
        }
        const double scale = conveyorScale(scene_->physicsDoc, load.mech.surface, belt.mech.surface, belt.mech.grip);
        return direction * belt.mech.speed * scale;
    }

    bool insideHandoff(const SceneNode& load) const {
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
            if (!node->mech.handoff) {
                continue;
            }
            Vec3 normal;
            double depth = 0;
            if (separated(boxOf(load), boxOf(*node), normal, depth)) {
                return true;
            }
        }
        return false;
    }

    SceneNode* supportingBelt(const SceneNode& load) const {
        std::vector<SceneNode*> hits;
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
            if (node->mech.kind != MechKind::Conveyor) {
                continue;
            }
            Vec3 normal;
            double depth = 0;
            if (!separated(boxOf(load), boxOf(*node), normal, depth)) {
                continue;
            }
            hits.push_back(node.get());
        }
        for (SceneNode* belt : hits) {
            if (!load.mech.nextName.empty() && belt->name == load.mech.nextName) {
                return belt;
            }
        }
        for (SceneNode* belt : hits) {
            if (belt->mech.nextName.empty()) {
                continue;
            }
            for (SceneNode* other : hits) {
                if (other->name == belt->mech.nextName) {
                    return other;
                }
            }
        }
        return hits.empty() ? nullptr : hits.front();
    }

    void stepMotors(double dt) {
        for (MotorAspect& motor : scene_->motors) {
            const MotorEvent event = advanceMotor(motor, dt);
            if (!motor.owner.empty()) {
                if (auto owner = scene_->findByName(motor.owner)) {
                    if (owner->mech.kind == MechKind::Conveyor || owner->mech.motor == motor.name) {
                        owner->mech.speed = motor.speed;
                    }
                }
            }
            const EncoderSample sample = sampleEncoder(motor);
            if (!motor.countSignal.empty()) {
                scene_->upsertSignal(motor.countSignal, SignalType::Int).integer = sample.count;
            }
            if (!motor.pulseSignal.empty()) {
                scene_->upsertSignal(motor.pulseSignal, SignalType::Bool).flag = sample.pulseHigh;
            }
            if (!motor.graySignal.empty()) {
                scene_->upsertSignal(motor.graySignal, SignalType::Int).integer = sample.gray;
            }
            if (!motor.runningSignal.empty()) {
                scene_->upsertSignal(motor.runningSignal, SignalType::Bool).flag = motor.running;
            }
            if (!motor.speedSignal.empty()) {
                scene_->upsertSignal(motor.speedSignal, SignalType::Real).real = motor.speed;
            }
            if (!motor.distanceSignal.empty()) {
                scene_->upsertSignal(motor.distanceSignal, SignalType::Bool).flag = event.distanceNotified;
            }
            if (!motor.startedSignal.empty()) {
                scene_->upsertSignal(motor.startedSignal, SignalType::Bool).flag = event.started;
            }
            if (!motor.stoppedSignal.empty()) {
                scene_->upsertSignal(motor.stoppedSignal, SignalType::Bool).flag = event.stopped;
            }
            if (!motor.changedSignal.empty()) {
                scene_->upsertSignal(motor.changedSignal, SignalType::Bool).flag = event.speedChanged;
            }
        }
    }

    void rideSurfaces(double dt) {
        for (BodyState& body : bodies_) {
            if (!body.riding) {
                continue;
            }
            auto node = scene_->find(body.id);
            if (!node) {
                continue;
            }
            if (insideHandoff(*node)) {
                body.riding = false;
                body.dynamic = true;
                node->mech.motion = BodyMotion::Free;
                node->mech.kinematic = false;
                node->mech.asleep = false;
                continue;
            }
            SceneNode* belt = supportingBelt(*node);
            if (belt == nullptr) {
                body.riding = false;
                body.dynamic = true;
                node->mech.motion = BodyMotion::Free;
                node->mech.kinematic = false;
                continue;
            }
            const Vec3 velocity = beltVelocity(*belt, *node);
            body.velocity = velocity;
            node->local.translation = node->local.translation + velocity * dt;
            if (belt->mech.guide > 0) {
                Vec3 direction = length(belt->mech.direction) > 1e-8 ? normalize(belt->mech.direction) : Vec3{1, 0, 0};
                Vec3 side = cross(direction, Vec3{0, 0, 1});
                if (length(side) > 1e-6) {
                    side = normalize(side);
                    const double lateral = dot(worldTranslation(*node) - worldTranslation(*belt), side);
                    if (std::abs(lateral) > belt->mech.guide) {
                        node->local.translation =
                            node->local.translation - side * (lateral - std::copysign(belt->mech.guide, lateral));
                    }
                }
            }
            scene_->physicsDoc.stats.contacts += 1;
        }
    }

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
                    node->mech.asleep = false;
                    scene_->physicsDoc.stats.contacts += 1;
                    if (obstacle->mech.kind == MechKind::Conveyor && !body.riding) {
                        const Vec3 belt = beltVelocity(*obstacle, *node);
                        const double bounce = pairRestitution(scene_->physicsDoc, node->mech.surface, obstacle->mech.surface);
                        body.velocity.x = belt.x;
                        body.velocity.y = belt.y;
                        body.velocity.z *= -bounce;
                        if (node->mech.surfaceRide && normal.z > 0.5 && !insideHandoff(*node)) {
                            body.dynamic = false;
                            body.riding = true;
                            node->mech.motion = BodyMotion::Surface;
                            node->mech.kinematic = true;
                        }
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
