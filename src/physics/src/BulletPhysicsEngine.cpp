#include "sim/physics/BulletPhysicsEngine.hpp"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace sim {
namespace {

constexpr double kMmToM = 0.001;

btVector3 toBullet(Vec3 v, double scale = kMmToM) {
    return btVector3(static_cast<btScalar>(v.x * scale), static_cast<btScalar>(v.y * scale),
                     static_cast<btScalar>(v.z * scale));
}

btQuaternion toBullet(Quat q) {
    const Quat n = q.normalized();
    return btQuaternion(static_cast<btScalar>(n.x), static_cast<btScalar>(n.y), static_cast<btScalar>(n.z),
                        static_cast<btScalar>(n.w));
}

btTransform toBullet(const Transform& transform) {
    btTransform out;
    out.setIdentity();
    out.setOrigin(toBullet(transform.translation));
    out.setRotation(toBullet(transform.rotation));
    return out;
}

btMatrix3x3 basisWithX(Vec3 axis) {
    btVector3 x = toBullet(axis, 1.0);
    if (x.length2() < btScalar(1e-12)) {
        x = btVector3(1, 0, 0);
    }
    x.normalize();
    const btVector3 hint = std::abs(x.z()) < btScalar(0.9) ? btVector3(0, 0, 1) : btVector3(0, 1, 0);
    btVector3 y = hint.cross(x).normalized();
    btVector3 z = x.cross(y).normalized();
    return btMatrix3x3(x.x(), y.x(), z.x(), x.y(), y.y(), z.y(), x.z(), y.z(), z.z());
}

double volumeCubicMeters(const Mech& mech) {
    const double hx = std::max(mech.halfExtents.x, 0.1) * kMmToM;
    const double hy = std::max(mech.halfExtents.y, 0.1) * kMmToM;
    const double hz = std::max(mech.halfExtents.z, 0.1) * kMmToM;
    const double radius = std::max(mech.radius, 0.1) * kMmToM;
    const double height = std::max(mech.height, 0.1) * kMmToM;
    switch (mech.shape) {
        case CollisionShape::Sphere:
            return 4.0 / 3.0 * kPi * radius * radius * radius;
        case CollisionShape::Capsule: {
            const double cylinder = kPi * radius * radius * std::max(height - 2.0 * radius, 0.0);
            const double caps = 4.0 / 3.0 * kPi * radius * radius * radius;
            return cylinder + caps;
        }
        case CollisionShape::Box:
        case CollisionShape::ConvexHull:
            return 8.0 * hx * hy * hz;
    }
    return 8.0 * hx * hy * hz;
}

void collect(const std::shared_ptr<SceneNode>& node, std::vector<SceneNode*>& out) {
    if (!node) {
        return;
    }
    out.push_back(node.get());
    for (const auto& child : node->children) {
        collect(child, out);
    }
}

void writePose(SceneNode& node, const btTransform& world) {
    const btVector3 origin = world.getOrigin();
    const btQuaternion rotation = world.getRotation();
    Transform pose;
    pose.translation = {origin.x() / kMmToM, origin.y() / kMmToM, origin.z() / kMmToM};
    pose.rotation = Quat{rotation.w(), rotation.x(), rotation.y(), rotation.z()}.normalized();
    auto parent = node.parent.lock();
    if (!parent || parent->id == "root") {
        node.local.translation = pose.translation;
        node.local.rotation = pose.rotation;
        return;
    }
    const Mat4 local = mul(inverseAffine(parent->worldMatrix()), pose.matrix());
    node.local.translation = {local(3, 0), local(3, 1), local(3, 2)};
    const btMatrix3x3 basis(static_cast<btScalar>(local(0, 0)), static_cast<btScalar>(local(1, 0)),
                            static_cast<btScalar>(local(2, 0)), static_cast<btScalar>(local(0, 1)),
                            static_cast<btScalar>(local(1, 1)), static_cast<btScalar>(local(2, 1)),
                            static_cast<btScalar>(local(0, 2)), static_cast<btScalar>(local(1, 2)),
                            static_cast<btScalar>(local(2, 2)));
    btQuaternion localRotation;
    basis.getRotation(localRotation);
    node.local.rotation = Quat{localRotation.w(), localRotation.x(), localRotation.y(), localRotation.z()}.normalized();
}

bool pointInside(Vec3 point, const SceneNode& box) {
    const Vec3 half = box.mech.halfExtents;
    const Vec3 d = point - box.local.translation;
    return std::abs(d.x) <= half.x && std::abs(d.y) <= half.y && std::abs(d.z) <= half.z;
}

}  // namespace

BulletPhysicsEngine::BulletPhysicsEngine() = default;

BulletPhysicsEngine::~BulletPhysicsEngine() { shutdown(); }

bool BulletPhysicsEngine::build(Scene& scene, std::string& error) {
    shutdown();
    scene_ = &scene;
    config_ = new btDefaultCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(config_);
    broadphase_ = new btDbvtBroadphase();
    solver_ = new btSequentialImpulseConstraintSolver();
    world_ = new btDiscreteDynamicsWorld(dispatcher_, broadphase_, solver_, config_);
    world_->setGravity(btVector3(0, 0, static_cast<btScalar>(-9.81)));
    world_->getSolverInfo().m_numIterations = 20;
    ghosts_ = new btGhostPairCallback();
    world_->getPairCache()->setInternalGhostPairCallback(ghosts_);

    anchorShape_ = new btSphereShape(static_cast<btScalar>(0.02));
    btTransform identity;
    identity.setIdentity();
    auto* motion = new btDefaultMotionState(identity);
    btRigidBody::btRigidBodyConstructionInfo info(0, motion, anchorShape_, btVector3(0, 0, 0));
    anchor_ = new btRigidBody(info);
    anchor_->setCollisionFlags(anchor_->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    world_->addRigidBody(anchor_, 0, 0);

    std::vector<SceneNode*> nodes;
    collect(scene.root(), nodes);
    for (SceneNode* node : nodes) {
        if (node->visible && mechCreatesBody(node->mech.kind)) {
            addBody(*node);
        }
    }
    for (SceneNode* node : nodes) {
        if (node->visible && mechIsJoint(node->mech.kind)) {
            addJoint(*node);
        } else if (node->visible && node->mech.kind == MechKind::CollisionSensor) {
            addGhost(*node);
        }
    }
    for (SceneNode* node : nodes) {
        if (node->mech.kind == MechKind::Source) {
            sourceClocks_.emplace_back(node->id, 0);
        }
    }
    error.clear();
    return true;
}

void BulletPhysicsEngine::addBody(SceneNode& node) {
    const Mech& mech = node.mech;
    const bool kinematic = mech.kinematic || mech.kind == MechKind::Conveyor || mech.kind == MechKind::Collision;
    btCollisionShape* shape = nullptr;
    if (mech.shape == CollisionShape::Sphere) {
        shape = new btSphereShape(static_cast<btScalar>(std::max(mech.radius, 0.5) * kMmToM));
    } else if (mech.shape == CollisionShape::Capsule) {
        const btScalar radius = static_cast<btScalar>(std::max(mech.radius, 0.5) * kMmToM);
        const btScalar height = static_cast<btScalar>(std::max(mech.height - 2.0 * mech.radius, 0.5) * kMmToM);
        shape = new btCapsuleShapeZ(radius, height);
    } else if (mech.shape == CollisionShape::ConvexHull) {
        auto* hull = new btConvexHullShape();
        if (scene_ != nullptr) {
            if (const auto mesh = scene_->mesh(node.meshId)) {
                const int count = static_cast<int>(mesh->vertexCount());
                const int stride = std::max(1, count / 96);
                for (int i = 0; i < count; i += stride) {
                    hull->addPoint(btVector3(static_cast<btScalar>(mesh->positions[static_cast<std::size_t>(i) * 3] * kMmToM),
                                             static_cast<btScalar>(mesh->positions[static_cast<std::size_t>(i) * 3 + 1] * kMmToM),
                                             static_cast<btScalar>(mesh->positions[static_cast<std::size_t>(i) * 3 + 2] * kMmToM)),
                                   false);
                }
                hull->recalcLocalAabb();
            }
        }
        if (hull->getNumPoints() < 4) {
            delete hull;
            shape = new btBoxShape(toBullet(mech.halfExtents));
        } else {
            shape = hull;
        }
    } else {
        shape = new btBoxShape(toBullet(mech.halfExtents));
    }
    shape->setMargin(static_cast<btScalar>(0.002));

    double mass = 0;
    if (!kinematic && mech.kind != MechKind::Collision) {
        mass = mech.autoMass ? std::max(0.05, volumeCubicMeters(mech) * 900.0) : std::max(0.01, mech.mass);
    }
    btVector3 inertia(0, 0, 0);
    if (mass > 0) {
        shape->calculateLocalInertia(static_cast<btScalar>(mass), inertia);
    }
    auto* motion = new btDefaultMotionState(toBullet(node.local));
    btRigidBody::btRigidBodyConstructionInfo info(static_cast<btScalar>(mass), motion, shape, inertia);
    info.m_friction = static_cast<btScalar>(0.85);
    info.m_restitution = static_cast<btScalar>(0.02);
    auto* body = new btRigidBody(info);
    body->setDamping(static_cast<btScalar>(0.04), static_cast<btScalar>(0.2));
    if (kinematic) {
        body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        body->setActivationState(DISABLE_DEACTIVATION);
    } else if (mass > 0) {
        body->setActivationState(DISABLE_DEACTIVATION);
    }
    world_->addRigidBody(body);

    BodyRecord record;
    record.id = node.id;
    record.body = body;
    record.shape = shape;
    record.dynamic = mass > 0;
    record.conveyor = mech.kind == MechKind::Conveyor;
    if (record.conveyor) {
        Vec3 direction = mech.direction;
        if (length(direction) < 1e-8) {
            direction = {1, 0, 0};
        }
        direction = normalize(node.local.rotation.rotate(direction));
        record.beltMeters = direction * (mech.speed * kMmToM);
    }
    bodies_.push_back(record);
}

void BulletPhysicsEngine::addJoint(SceneNode& node) {
    btRigidBody* bodyA = nullptr;
    btRigidBody* bodyB = anchor_;
    for (const BodyRecord& record : bodies_) {
        if (record.id == node.id) {
            bodyA = record.body;
        }
    }
    if (!node.mech.other.empty() && scene_ != nullptr) {
        if (const auto other = scene_->findByName(node.mech.other)) {
            for (const BodyRecord& record : bodies_) {
                if (record.id == other->id) {
                    bodyB = record.body;
                }
            }
        }
    }
    if (bodyA == nullptr || bodyB == nullptr) {
        return;
    }

    // Bullet's slider frame X points opposite the axis we pass in. Flip sliders
    // so a positive target moves along the authored axis. Hinges keep the axis
    // and invert motor velocity instead (see applyDrives).
    const Vec3 axis = node.mech.kind == MechKind::Slider ? node.mech.axis * -1.0 : node.mech.axis;
    btTransform frameA;
    frameA.setIdentity();
    frameA.setBasis(basisWithX(axis));
    btTransform frameB = bodyB->getWorldTransform().inverse() * bodyA->getWorldTransform() * frameA;

    btTypedConstraint* constraint = nullptr;
    bool slider = false;
    bool hinge = false;
    if (node.mech.kind == MechKind::Slider) {
        auto* joint = new btSliderConstraint(*bodyA, *bodyB, frameA, frameB, true);
        const double lower = node.mech.limited ? node.mech.lower * kMmToM : -2.0;
        const double upper = node.mech.limited ? node.mech.upper * kMmToM : 2.0;
        joint->setLowerLinLimit(static_cast<btScalar>(std::min(lower, upper)));
        joint->setUpperLinLimit(static_cast<btScalar>(std::max(lower, upper)));
        joint->setLowerAngLimit(0);
        joint->setUpperAngLimit(0);
        joint->setPoweredLinMotor(true);
        joint->setMaxLinMotorForce(static_cast<btScalar>(4000));
        constraint = joint;
        slider = true;
    } else if (node.mech.kind == MechKind::Hinge) {
        const btVector3 axis = basisWithX(node.mech.axis).getColumn(0);
        const btVector3 pivotA = frameA.getOrigin();
        const btVector3 pivotB = frameB.getOrigin();
        const btVector3 axisB = frameB.getBasis() * btVector3(1, 0, 0);
        auto* joint = new btHingeConstraint(*bodyA, *bodyB, pivotA, pivotB, axis, axisB, true);
        if (node.mech.limited) {
            // Bullet's hinge angle increases opposite the authored axis.
            const double a = -std::max(node.mech.lower, node.mech.upper);
            const double b = -std::min(node.mech.lower, node.mech.upper);
            joint->setLimit(static_cast<btScalar>(deg2rad(a)), static_cast<btScalar>(deg2rad(b)));
        } else {
            joint->setLimit(static_cast<btScalar>(-kPi), static_cast<btScalar>(kPi));
        }
        constraint = joint;
        hinge = true;
        (void)axis;
    } else {
        constraint = new btFixedConstraint(*bodyA, *bodyB, frameA, frameB);
    }
    world_->addConstraint(constraint, true);
    joints_.push_back(JointRecord{node.id, constraint, slider, hinge, 0});
}

void BulletPhysicsEngine::addGhost(SceneNode& node) {
    auto* shape = new btBoxShape(toBullet(node.mech.halfExtents));
    shape->setMargin(static_cast<btScalar>(0.002));
    auto* ghost = new btGhostObject();
    ghost->setCollisionShape(shape);
    ghost->setWorldTransform(toBullet(node.local));
    ghost->setCollisionFlags(ghost->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    world_->addCollisionObject(ghost, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);
    ghostRecords_.push_back(GhostRecord{node.id, ghost, shape});
}

void BulletPhysicsEngine::applyDrives(double dt) {
    if (scene_ == nullptr || dt <= 0) {
        return;
    }
    for (JointRecord& joint : joints_) {
        auto node = scene_->find(joint.id);
        if (!node || node->mech.drive == DriveMode::None) {
            continue;
        }
        const double maxDelta = std::max(0.0, node->mech.maxAccel) * dt;
        if (joint.slider) {
            auto* slider = static_cast<btSliderConstraint*>(joint.constraint);
            double desired = 0;
            if (node->mech.drive == DriveMode::Speed) {
                desired = node->mech.target;
            } else {
                const double current = slider->getLinearPos() / kMmToM;
                desired = std::clamp((node->mech.target - current) / dt, -node->mech.maxSpeed, node->mech.maxSpeed);
            }
            desired = std::clamp(desired, joint.lastSpeed - maxDelta, joint.lastSpeed + maxDelta);
            desired = std::clamp(desired, -std::abs(node->mech.maxSpeed), std::abs(node->mech.maxSpeed));
            joint.lastSpeed = desired;
            slider->setPoweredLinMotor(true);
            slider->setTargetLinMotorVelocity(static_cast<btScalar>(desired * kMmToM));
            slider->setMaxLinMotorForce(static_cast<btScalar>(4000));
        } else if (joint.hinge) {
            auto* hinge = static_cast<btHingeConstraint*>(joint.constraint);
            hinge->enableMotor(true);
            hinge->setMaxMotorImpulse(static_cast<btScalar>(80));
            const double current = -rad2deg(hinge->getHingeAngle());
            if (node->mech.drive == DriveMode::Speed) {
                const double speed = std::clamp(node->mech.target, -std::abs(node->mech.maxSpeed), std::abs(node->mech.maxSpeed));
                hinge->enableAngularMotor(true, static_cast<btScalar>(deg2rad(-speed)), static_cast<btScalar>(80));
                joint.lastSpeed = speed;
            } else {
                double step = std::clamp(node->mech.target - current, -std::abs(node->mech.maxSpeed) * dt,
                                         std::abs(node->mech.maxSpeed) * dt);
                const double accelStep = std::max(0.0, node->mech.maxAccel) * dt * dt;
                const double previousStep = joint.lastSpeed * dt;
                step = std::clamp(step, previousStep - accelStep, previousStep + accelStep);
                joint.lastSpeed = dt > 0 ? step / dt : 0;
                hinge->setMotorTarget(static_cast<btScalar>(deg2rad(-(current + step))), static_cast<btScalar>(dt));
            }
        }
    }
}

void BulletPhysicsEngine::applyConveyors() {
    if (world_ == nullptr) {
        return;
    }
    std::unordered_map<const btCollisionObject*, Vec3> belts;
    for (const BodyRecord& record : bodies_) {
        if (record.conveyor) {
            belts.emplace(record.body, record.beltMeters);
        }
    }
    const int manifolds = dispatcher_->getNumManifolds();
    for (int i = 0; i < manifolds; ++i) {
        btPersistentManifold* manifold = dispatcher_->getManifoldByIndexInternal(i);
        if (manifold == nullptr || manifold->getNumContacts() == 0) {
            continue;
        }
        const btCollisionObject* a = manifold->getBody0();
        const btCollisionObject* b = manifold->getBody1();
        const auto beltA = belts.find(a);
        const auto beltB = belts.find(b);
        btRigidBody* dynamic = nullptr;
        Vec3 belt{};
        if (beltA != belts.end()) {
            dynamic = const_cast<btRigidBody*>(btRigidBody::upcast(b));
            belt = beltA->second;
        } else if (beltB != belts.end()) {
            dynamic = const_cast<btRigidBody*>(btRigidBody::upcast(a));
            belt = beltB->second;
        }
        if (dynamic == nullptr || dynamic->getInvMass() <= 0) {
            continue;
        }
        btVector3 velocity = dynamic->getLinearVelocity();
        velocity.setX(static_cast<btScalar>(belt.x));
        velocity.setY(static_cast<btScalar>(belt.y));
        dynamic->setLinearVelocity(velocity);
        dynamic->activate(true);
    }
}

void BulletPhysicsEngine::syncTransforms() {
    if (scene_ == nullptr) {
        return;
    }
    for (const BodyRecord& record : bodies_) {
        if (!record.dynamic && !record.conveyor) {
            auto node = scene_->find(record.id);
            if (!node || !mechIsJoint(node->mech.kind)) {
                continue;
            }
        }
        auto node = scene_->find(record.id);
        if (!node || record.body == nullptr) {
            continue;
        }
        if (!record.dynamic && !mechIsJoint(node->mech.kind)) {
            continue;
        }
        writePose(*node, record.body->getWorldTransform());
    }
}

void BulletPhysicsEngine::updateSensors() {
    if (scene_ == nullptr || world_ == nullptr) {
        return;
    }
    for (const GhostRecord& ghost : ghostRecords_) {
        auto node = scene_->find(ghost.id);
        if (!node) {
            continue;
        }
        const std::string signalName = node->mech.signal.empty() ? node->name : node->mech.signal;
        bool hit = false;
        for (int i = 0; i < ghost.ghost->getNumOverlappingObjects(); ++i) {
            const btCollisionObject* object = ghost.ghost->getOverlappingObject(i);
            if (object != nullptr && object != anchor_) {
                hit = true;
                break;
            }
        }
        Signal& signal = scene_->upsertSignal(signalName, SignalType::Bool);
        signal.flag = hit;
    }

    std::vector<SceneNode*> nodes;
    collect(scene_->root(), nodes);
    for (SceneNode* node : nodes) {
        if (node->mech.kind != MechKind::DistanceSensor && node->mech.kind != MechKind::LightBarrier) {
            continue;
        }
        Vec3 axis = node->mech.axis;
        if (length(axis) < 1e-8) {
            axis = {0, 1, 0};
        }
        axis = normalize(node->local.rotation.rotate(axis));
        const double range = std::max(node->mech.range, 1.0);
        const btVector3 from = toBullet(node->local.translation);
        const btVector3 to = toBullet(node->local.translation + axis * range);
        btCollisionWorld::ClosestRayResultCallback callback(from, to);
        world_->rayTest(from, to, callback);
        const bool blocked = callback.hasHit();
        const double distance = blocked ? callback.m_closestHitFraction * range : range;
        const std::string signalName = node->mech.signal.empty() ? node->name : node->mech.signal;
        Signal& signal = scene_->upsertSignal(signalName, SignalType::Bool);
        signal.flag = blocked;
        if (!node->mech.valueSignal.empty()) {
            Signal& value = scene_->upsertSignal(node->mech.valueSignal, SignalType::Real);
            value.real = distance;
        }
    }
}

void BulletPhysicsEngine::updateSources(double dt) {
    if (scene_ == nullptr) {
        return;
    }
    for (auto& [id, clock] : sourceClocks_) {
        auto source = scene_->find(id);
        if (!source || source->mech.prototype.empty()) {
            continue;
        }
        clock += dt;
        const double interval = std::max(source->mech.interval, 0.05);
        if (clock < interval) {
            continue;
        }
        clock = 0;
        auto prototype = scene_->findByName(source->mech.prototype);
        if (!prototype) {
            continue;
        }
        Transform pose = source->local;
        auto clone = scene_->spawnClone(prototype, pose);
        if (clone) {
            addBody(*clone);
        }
    }
}

void BulletPhysicsEngine::updateSinks() {
    if (scene_ == nullptr) {
        return;
    }
    std::vector<SceneNode*> nodes;
    collect(scene_->root(), nodes);
    std::vector<std::string> doomed;
    for (SceneNode* sink : nodes) {
        if (sink->mech.kind != MechKind::Sink) {
            continue;
        }
        for (const BodyRecord& record : bodies_) {
            if (!record.dynamic || record.body == nullptr) {
                continue;
            }
            auto node = scene_->find(record.id);
            if (!node || !node->transient) {
                continue;
            }
            const btVector3 origin = record.body->getWorldTransform().getOrigin();
            const Vec3 point{origin.x() / kMmToM, origin.y() / kMmToM, origin.z() / kMmToM};
            if (pointInside(point, *sink)) {
                doomed.push_back(record.id);
            }
        }
    }
    std::sort(doomed.begin(), doomed.end());
    doomed.erase(std::unique(doomed.begin(), doomed.end()), doomed.end());
    for (const std::string& id : doomed) {
        destroyBody(id);
        if (scene_ != nullptr) {
            scene_->removeNode(id);
        }
    }
}

void BulletPhysicsEngine::destroyBody(const std::string& id) {
    for (auto it = joints_.begin(); it != joints_.end();) {
        if (it->id == id && it->constraint != nullptr && world_ != nullptr) {
            world_->removeConstraint(it->constraint);
            delete it->constraint;
            it = joints_.erase(it);
        } else {
            ++it;
        }
    }
    for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
        if (it->id != id) {
            continue;
        }
        if (world_ != nullptr && it->body != nullptr) {
            world_->removeRigidBody(it->body);
            delete it->body->getMotionState();
            delete it->body;
        }
        delete it->shape;
        bodies_.erase(it);
        break;
    }
}

void BulletPhysicsEngine::adoptSpawned(const std::string& id) {
    if (world_ == nullptr || scene_ == nullptr) {
        return;
    }
    for (const BodyRecord& record : bodies_) {
        if (record.id == id) {
            return;
        }
    }
    if (auto node = scene_->find(id)) {
        addBody(*node);
    }
}

void BulletPhysicsEngine::releaseBody(const std::string& id) { destroyBody(id); }

void BulletPhysicsEngine::step(double dtSeconds) {
    if (world_ == nullptr || dtSeconds <= 0) {
        return;
    }
    applyDrives(dtSeconds);
    applyConveyors();
    world_->stepSimulation(static_cast<btScalar>(dtSeconds), 0);
    syncTransforms();
    updateSensors();
    updateSources(dtSeconds);
    updateSinks();
}

void BulletPhysicsEngine::shutdown() {
    for (JointRecord& joint : joints_) {
        if (world_ != nullptr && joint.constraint != nullptr) {
            world_->removeConstraint(joint.constraint);
        }
        delete joint.constraint;
    }
    joints_.clear();
    for (GhostRecord& ghost : ghostRecords_) {
        if (world_ != nullptr && ghost.ghost != nullptr) {
            world_->removeCollisionObject(ghost.ghost);
        }
        delete ghost.ghost;
        delete ghost.shape;
    }
    ghostRecords_.clear();
    for (BodyRecord& record : bodies_) {
        if (world_ != nullptr && record.body != nullptr) {
            world_->removeRigidBody(record.body);
            delete record.body->getMotionState();
            delete record.body;
        }
        delete record.shape;
    }
    bodies_.clear();
    if (world_ != nullptr && anchor_ != nullptr) {
        world_->removeRigidBody(anchor_);
        delete anchor_->getMotionState();
        delete anchor_;
        anchor_ = nullptr;
    }
    delete anchorShape_;
    anchorShape_ = nullptr;
    delete world_;
    world_ = nullptr;
    delete ghosts_;
    ghosts_ = nullptr;
    delete solver_;
    solver_ = nullptr;
    delete broadphase_;
    broadphase_ = nullptr;
    delete dispatcher_;
    dispatcher_ = nullptr;
    delete config_;
    config_ = nullptr;
    sourceClocks_.clear();
    scene_ = nullptr;
}

std::unique_ptr<IPhysicsEngine> createPhysicsEngine() { return std::make_unique<BulletPhysicsEngine>(); }

}  // namespace sim
