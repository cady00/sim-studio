#include "sim/physics/PhysicsEngine.hpp"

#include "sim/collision/Query.hpp"
#include "sim/physics/ContactNarrow.hpp"

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
    bool ball = false;
    Vec3 restTranslation{};
    Quat restRotation = Quat::identity();
    double position = 0;
    double speed = 0;
    // Degrees per second in the parent frame. Orthogonal to `axis` stays free on a ball joint.
    Vec3 omega{};
    Quat swing = Quat::identity();
};

struct Probe {
    bool hit = false;
    Vec3 push{0, 0, 1};
    double depth = 0;
    Vec3 point{};
};

struct Touch {
    bool used = false;
    Vec3 push{0, 0, 1};
    double depth = 0;
    Vec3 point{};
    std::vector<Vec3> points;
    std::string otherId;
    std::string otherName;
    bool conveyor = false;
    Vec3 belt{};
    double friction = 0.4;
    double restitution = 0.05;
    bool ride = false;
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

void integrateBall(JointState& joint, SceneNode& node, double dt) {
    const Vec3 axis = joint.axis;
    if (node.mech.drive != DriveMode::None) {
        double axisSpeed = dot(joint.omega, axis);
        integrateJoint(joint.position, axisSpeed, node.mech, dt);
        joint.omega = joint.omega - axis * dot(joint.omega, axis) + axis * axisSpeed;
    }
    const Vec3 along = axis * dot(joint.omega, axis);
    const double keep = std::max(0.0, 1.0 - std::max(0.0, node.mech.angularDamp) * dt);
    joint.omega = along + (joint.omega - along) * keep;
    const double rate = length(joint.omega);
    if (rate > 1e-8 && dt > 0) {
        const Quat delta = Quat::fromAxisAngle(normalize(joint.omega), deg2rad(rate * dt));
        joint.swing = (delta * joint.swing).normalized();
    }
    if (node.mech.limited) {
        const double cone = std::max(0.0, node.mech.upper);
        const Vec3 rest = length(joint.axis) > 1e-8 ? normalize(joint.axis) : Vec3{0, 0, 1};
        const Vec3 swung = joint.swing.rotate(rest);
        const double cosine = std::clamp(dot(rest, normalize(swung)), -1.0, 1.0);
        const double angle = rad2deg(std::acos(cosine));
        if (angle > cone + 1e-3) {
            Vec3 correction = cross(swung, rest);
            if (length(correction) < 1e-8) {
                correction = std::abs(rest.z) < 0.9 ? cross(rest, Vec3{0, 0, 1}) : cross(rest, Vec3{1, 0, 0});
            }
            correction = normalize(correction);
            const Quat back = Quat::fromAxisAngle(correction, deg2rad(angle - cone));
            joint.swing = (back * joint.swing).normalized();
            const double closing = dot(joint.omega, correction);
            if (closing < 0) {
                joint.omega = joint.omega - correction * closing;
            }
        }
    }
    node.local.rotation = (joint.swing * joint.restRotation).normalized();
    node.local.translation = joint.restTranslation;
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
                joint.hinge = node->mech.kind == MechKind::Hinge;
                joint.ball = node->mech.kind == MechKind::Ball;
                joint.omega = node->mech.spin;
                joint.restTranslation = node->local.translation;
                joint.restRotation = node->local.rotation;
                joints_.push_back(std::move(joint));
            }
        }
        error.clear();
        return true;
    }

    bool stopAtImpact(BodyState& body, SceneNode& node, double dt) {
        const Box self = boxOf(node);
        double best = dt + 1.0;
        Vec3 normal{0, 0, 1};
        bool hit = false;
        for (const BodyState& other : bodies_) {
            if (other.id == body.id || other.dynamic) {
                continue;
            }
            auto obstacle = scene_->find(other.id);
            if (!obstacle) {
                continue;
            }
            double toi = 0;
            Vec3 hitNormal{0, 0, 1};
            bool found = false;
            if (obstacle->mech.shape == CollisionShape::Mesh) {
                if (const TriangleBvh* bvh = trianglesFor(*obstacle)) {
                    found = sweepAabbMesh(self.center, self.half, body.velocity, dt, *bvh, obstacle->worldMatrix(), toi, hitNormal);
                }
            } else {
                const Box box = boxOf(*obstacle);
                found = sweepAabbToi(self.center, self.half, body.velocity, dt, box.center, box.half, toi, hitNormal);
            }
            if (found && toi < best) {
                best = toi;
                normal = hitNormal;
                hit = true;
            }
        }
        if (!hit) {
            return false;
        }
        node.local.translation = node.local.translation + body.velocity * std::max(0.0, best);
        if (dot(body.velocity, normal) < 0) {
            body.velocity = body.velocity - normal * dot(body.velocity, normal);
        }
        return true;
    }

    void step(double dtSeconds) override {
        if (scene_ == nullptr || dtSeconds <= 0) {
            return;
        }
        scene_->contactNotices.clear();
        if (Signal* contact = scene_->findSignal("contact")) {
            contact->flag = false;
        }
        if (Signal* impulse = scene_->findSignal("contact_impulse")) {
            impulse->real = 0;
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
            if (joint.ball) {
                integrateBall(joint, *node, dt);
            } else {
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
            if (node->mech.continuous) {
                body.velocity.z += kGravity * dt;
                if (!stopAtImpact(body, *node, dt)) {
                    node->local.translation = node->local.translation + body.velocity * dt;
                }
            } else {
                body.velocity.z += kGravity * dt;
                node->local.translation = node->local.translation + body.velocity * dt;
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
        meshBvh_.clear();
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
            rememberHull(*scene_, *node);
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
        ConvexCacheEntry entry;
        entry.meshId = key;
        bool stored = false;
        if (!node.meshId.empty()) {
            if (const auto mesh = scene.mesh(node.meshId); mesh && !mesh->empty()) {
                entry.fingerprint = meshFingerprint(*mesh);
                for (const ConvexCacheEntry& existing : scene.convexCache) {
                    if (existing.meshId == key && existing.fingerprint == entry.fingerprint && !existing.pieces.empty()) {
                        return;
                    }
                }
                for (std::vector<Vec3>& piece : decomposeMesh(*mesh)) {
                    if (piece.size() < 3) {
                        continue;
                    }
                    ConvexPiece storedPiece;
                    storedPiece.vertices = std::move(piece);
                    entry.pieces.push_back(std::move(storedPiece));
                }
                stored = !entry.pieces.empty();
            }
        }
        if (!stored) {
            const double radius = std::max(std::abs(node.mech.halfExtents.x), 1.0);
            const double halfHeight = std::max(std::abs(node.mech.halfExtents.z), 1.0);
            ConvexPiece ring;
            constexpr int kSides = 10;
            for (int side = 0; side < kSides; ++side) {
                const double angle = (2.0 * kPi * side) / static_cast<double>(kSides);
                const double x = radius * std::cos(angle);
                const double y = radius * std::sin(angle);
                ring.vertices.push_back({x, y, halfHeight});
                ring.vertices.push_back({x, y, -halfHeight});
            }
            entry.pieces.push_back(std::move(ring));
            entry.fingerprint = 2;
        }
        scene.convexCache.erase(std::remove_if(scene.convexCache.begin(), scene.convexCache.end(),
                                               [&](const ConvexCacheEntry& existing) { return existing.meshId == key; }),
                                scene.convexCache.end());
        scene.convexCache.push_back(std::move(entry));
    }

    const ConvexCacheEntry* cacheFor(const SceneNode& node) const {
        const std::string key = node.meshId.empty() ? node.id : node.meshId;
        for (const ConvexCacheEntry& entry : scene_->convexCache) {
            if (entry.meshId == key && !entry.pieces.empty()) {
                return &entry;
            }
        }
        return nullptr;
    }

    const TriangleBvh* trianglesFor(const SceneNode& node) {
        if (node.meshId.empty()) {
            return nullptr;
        }
        const auto found = meshBvh_.find(node.meshId);
        if (found == meshBvh_.end()) {
            const auto mesh = scene_->mesh(node.meshId);
            TriangleBvh built = mesh && !mesh->empty() ? buildTriangleBvh(*mesh) : TriangleBvh{};
            const auto inserted = meshBvh_.emplace(node.meshId, std::move(built));
            if (inserted.first->second.root < 0) {
                return nullptr;
            }
            return &inserted.first->second;
        }
        if (found->second.root < 0) {
            return nullptr;
        }
        return &found->second;
    }

    Probe probeBoxes(const SceneNode& moving, const SceneNode& obstacle) const {
        Vec3 toward;
        double depth = 0;
        if (!separated(boxOf(moving), boxOf(obstacle), toward, depth)) {
            return {};
        }
        Probe probe;
        probe.hit = true;
        probe.push = toward * -1.0;
        probe.depth = depth;
        probe.point = worldTranslation(moving);
        return probe;
    }

    std::vector<Probe> probeConvex(const SceneNode& boxNode, const SceneNode& hullNode, bool flip) {
        const ConvexCacheEntry* entry = cacheFor(hullNode);
        if (entry == nullptr) {
            return {};
        }
        const Box bounds = boxOf(boxNode);
        ConvexShape box;
        box.kind = ConvexShape::Kind::Box;
        box.center = bounds.center;
        box.half = bounds.half;
        box.rotation = Quat::identity();
        Probe best;
        for (const ConvexPiece& piece : entry->pieces) {
            if (piece.vertices.size() < 3) {
                continue;
            }
            ConvexShape hull;
            hull.kind = ConvexShape::Kind::Hull;
            hull.localPoints = piece.vertices;
            hull.useMatrix = true;
            hull.world = hullNode.worldMatrix();
            const ContactHit hit = queryConvex(box, hull);
            const double depth = hit.penetration > 0 ? hit.penetration : 0;
            if (depth <= best.depth) {
                continue;
            }
            Vec3 center{};
            for (const Vec3& vertex : piece.vertices) {
                center = center + transformPoint(hull.world, vertex);
            }
            center = center * (1.0 / static_cast<double>(piece.vertices.size()));
            Vec3 push = hit.normal;
            if (dot(push, bounds.center - center) < 0) {
                push = push * -1.0;
            }
            if (flip) {
                push = push * -1.0;
            }
            best.hit = true;
            best.push = length(push) > 1e-8 ? normalize(push) : Vec3{0, 0, 1};
            best.depth = depth;
            best.point = hit.pointA;
        }
        if (!best.hit) {
            return {};
        }
        return {best};
    }

    std::vector<Probe> narrowProbes(const SceneNode& moving, const SceneNode& obstacle) {
        if (moving.mech.shape == CollisionShape::Mesh && obstacle.mech.shape == CollisionShape::Mesh) {
            const TriangleBvh* first = trianglesFor(moving);
            const TriangleBvh* second = trianglesFor(obstacle);
            std::vector<Probe> probes;
            if (first != nullptr && second != nullptr) {
                const std::vector<NarrowContact> hits =
                    queryMeshMesh(*first, moving.worldMatrix(), *second, obstacle.worldMatrix(), 8);
                for (std::size_t i = 0; i < hits.size(); i += 2) {
                    Probe probe;
                    probe.hit = true;
                    probe.push = hits[i].normal;
                    probe.depth = hits[i].depth;
                    probe.point = hits[i].point;
                    probes.push_back(probe);
                }
            }
            return probes;
        }
        if (obstacle.mech.shape == CollisionShape::Mesh) {
            if (const TriangleBvh* bvh = trianglesFor(obstacle)) {
                const Box box = boxOf(moving);
                std::vector<Probe> probes;
                for (const NarrowContact& hit : queryBoxMesh(*bvh, obstacle.worldMatrix(), box.center, box.half, 4)) {
                    Probe probe;
                    probe.hit = true;
                    probe.push = hit.normal;
                    probe.depth = hit.depth;
                    probe.point = hit.point;
                    probes.push_back(probe);
                }
                return probes;
            }
        }
        if (moving.mech.shape == CollisionShape::Mesh) {
            if (const TriangleBvh* bvh = trianglesFor(moving)) {
                const Box box = boxOf(obstacle);
                std::vector<Probe> probes;
                for (const NarrowContact& hit : queryBoxMesh(*bvh, moving.worldMatrix(), box.center, box.half, 4)) {
                    Probe probe;
                    probe.hit = true;
                    probe.push = hit.normal * -1.0;
                    probe.depth = hit.depth;
                    probe.point = hit.point;
                    probes.push_back(probe);
                }
                return probes;
            }
        }
        if (obstacle.mech.shape == CollisionShape::ConvexHull || obstacle.mech.shape == CollisionShape::Extrusion) {
            return probeConvex(moving, obstacle, false);
        }
        if (moving.mech.shape == CollisionShape::ConvexHull || moving.mech.shape == CollisionShape::Extrusion) {
            return probeConvex(obstacle, moving, true);
        }
        const Probe boxes = probeBoxes(moving, obstacle);
        if (!boxes.hit) {
            return {};
        }
        return {boxes};
    }

    void applyFriction(BodyState& body, const Touch& touch, const SceneNode& node) {
        Vec3 velocity = body.velocity;
        const Vec3 push = length(touch.push) > 1e-8 ? normalize(touch.push) : Vec3{0, 0, 1};
        const double approach = dot(velocity, push);
        double normalDelta = 0;
        if (approach < 0) {
            // One gravity substep is about 163 mm/s. Keep that band inelastic so a resting
            // body does not chatter, and let faster impacts use the pair restitution.
            const bool resting = -approach < 250.0;
            const double after = resting ? 0.0 : -touch.restitution * approach;
            normalDelta = after - approach;
            velocity = velocity + push * normalDelta;
        }
        Vec3 target = touch.belt - push * dot(touch.belt, push);
        const Vec3 tangent = velocity - push * dot(velocity, push);
        const Vec3 slip = tangent - target;
        const double slipLength = length(slip);
        const double maxReduce = std::max(0.0, touch.friction) * std::abs(normalDelta);
        if (slipLength > 1e-9 && maxReduce > 0) {
            const double reduce = std::min(slipLength, maxReduce);
            velocity = velocity - slip * (reduce / slipLength);
        }
        body.velocity = velocity;
        const double impulseValue = massKilograms(node.mech) * std::abs(normalDelta);
        const std::vector<Vec3> points = touch.points.empty() ? std::vector<Vec3>{touch.point} : touch.points;
        for (const Vec3& point : points) {
            ContactNotice notice;
            notice.self = node.name;
            notice.other = touch.otherName;
            notice.impulse = impulseValue;
            notice.normal = push;
            notice.point = point;
            scene_->contactNotices.push_back(notice);
        }
        scene_->upsertSignal("contact", SignalType::Bool).flag = true;
        Signal& impulse = scene_->upsertSignal("contact_impulse", SignalType::Real);
        impulse.real = std::max(impulse.real, impulseValue);
        if (touch.ride) {
            body.dynamic = false;
            body.riding = true;
            if (auto live = scene_->find(body.id)) {
                live->mech.motion = BodyMotion::Surface;
                live->mech.kinematic = true;
            }
        }
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
        std::vector<Touch> touches(bodies_.size());
        for (int iteration = 0; iteration < 4; ++iteration) {
            for (std::size_t index = 0; index < bodies_.size(); ++index) {
                BodyState& body = bodies_[index];
                if (!body.dynamic) {
                    continue;
                }
                auto node = scene_->find(body.id);
                if (!node) {
                    continue;
                }
                for (const BodyState& other : bodies_) {
                    if (other.id == body.id || other.dynamic) {
                        continue;
                    }
                    auto obstacle = scene_->find(other.id);
                    if (!obstacle) {
                        continue;
                    }
                    const std::vector<Probe> probes = narrowProbes(*node, *obstacle);
                    if (probes.empty()) {
                        continue;
                    }
                    const Probe* deepest = &probes.front();
                    for (const Probe& probe : probes) {
                        if (probe.depth > deepest->depth) {
                            deepest = &probe;
                        }
                    }
                    const double slop = obstacle->mech.shape == CollisionShape::Mesh ? 0.05 : 0.01;
                    node->local.translation = node->local.translation + deepest->push * (deepest->depth + slop);
                    node->mech.asleep = false;
                    Touch& touch = touches[index];
                    if (!touch.used || deepest->depth > touch.depth) {
                        if (!touch.used) {
                            scene_->physicsDoc.stats.contacts += static_cast<int>(probes.size());
                        }
                        touch.used = true;
                        touch.push = deepest->push;
                        touch.depth = deepest->depth;
                        touch.point = deepest->point;
                        touch.points.clear();
                        for (const Probe& probe : probes) {
                            touch.points.push_back(probe.point);
                        }
                        touch.otherId = obstacle->id;
                        touch.otherName = obstacle->name;
                        touch.conveyor = obstacle->mech.kind == MechKind::Conveyor;
                        touch.friction = pairFriction(scene_->physicsDoc, node->mech.surface, obstacle->mech.surface);
                        touch.restitution = pairRestitution(scene_->physicsDoc, node->mech.surface, obstacle->mech.surface);
                        touch.belt = {};
                        if (touch.conveyor) {
                            Vec3 direction = obstacle->mech.direction;
                            direction = length(direction) > 1e-8 ? normalize(direction) : Vec3{1, 0, 0};
                            touch.belt = direction * obstacle->mech.speed;
                        }
                        touch.ride = touch.conveyor && node->mech.surfaceRide && deepest->push.z > 0.5 && !insideHandoff(*node);
                    }
                }
            }
        }
        for (std::size_t index = 0; index < bodies_.size(); ++index) {
            if (!touches[index].used || !bodies_[index].dynamic) {
                continue;
            }
            auto node = scene_->find(bodies_[index].id);
            if (!node) {
                continue;
            }
            applyFriction(bodies_[index], touches[index], *node);
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
    std::unordered_map<std::string, TriangleBvh> meshBvh_;
};

}  // namespace

std::unique_ptr<IPhysicsEngine> createBuiltinPhysicsEngine() { return std::make_unique<BuiltinPhysicsEngine>(); }

}  // namespace sim
