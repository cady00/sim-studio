#include "sim/physics/JoltPhysicsEngine.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace sim {
namespace {

constexpr double kMmToM = 0.001;

namespace Layers {
constexpr JPH::ObjectLayer kStatic = 0;
constexpr JPH::ObjectLayer kMoving = 1;
constexpr JPH::ObjectLayer kAnchor = 2;
constexpr JPH::ObjectLayer kCount = 3;
}  // namespace Layers

namespace BroadLayers {
constexpr JPH::BroadPhaseLayer kStatic{0};
constexpr JPH::BroadPhaseLayer kMoving{1};
constexpr unsigned kCount = 2;
}  // namespace BroadLayers

class BroadPhaseMap final : public JPH::BroadPhaseLayerInterface {
public:
    BroadPhaseMap() {
        map_[Layers::kStatic] = BroadLayers::kStatic;
        map_[Layers::kMoving] = BroadLayers::kMoving;
        map_[Layers::kAnchor] = BroadLayers::kStatic;
    }
    unsigned GetNumBroadPhaseLayers() const override { return BroadLayers::kCount; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override { return map_[layer]; }

private:
    JPH::BroadPhaseLayer map_[Layers::kCount] = {};
};

class ObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broad) const override {
        if (layer == Layers::kAnchor) {
            return false;
        }
        if (layer == Layers::kStatic) {
            return broad == BroadLayers::kMoving;
        }
        return true;
    }
};

class ObjectPairFilter final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        if (a == Layers::kAnchor || b == Layers::kAnchor) {
            return false;
        }
        return a == Layers::kMoving || b == Layers::kMoving;
    }
};

class IgnoreBodyFilter final : public JPH::BodyFilter {
public:
    explicit IgnoreBodyFilter(JPH::BodyID ignore) : ignore_(ignore) {}
    bool ShouldCollide(const JPH::BodyID& id) const override { return id != ignore_; }
    bool ShouldCollideLocked(const JPH::Body& body) const override { return body.GetID() != ignore_; }

private:
    JPH::BodyID ignore_;
};

void joltTrace(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    std::fputc('\n', stderr);
    va_end(args);
}

void ensureJolt() {
    static bool ready = false;
    if (ready) {
        return;
    }
    JPH::RegisterDefaultAllocator();
    JPH::Trace = joltTrace;
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
    ready = true;
}

JPH::RVec3 toPos(Vec3 mm) {
    return JPH::RVec3(static_cast<float>(mm.x * kMmToM), static_cast<float>(mm.y * kMmToM),
                      static_cast<float>(mm.z * kMmToM));
}

JPH::Vec3 toDir(Vec3 v) {
    return JPH::Vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

JPH::Quat toRot(Quat q) {
    const Quat n = q.normalized();
    return JPH::Quat(static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z), static_cast<float>(n.w));
}

Quat fromRot(JPH::Quat q) { return Quat{q.GetW(), q.GetX(), q.GetY(), q.GetZ()}.normalized(); }

Quat quatFromMatrix(const Mat4& m) {
    const double trace = m(0, 0) + m(1, 1) + m(2, 2);
    Quat q;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m(1, 2) - m(2, 1)) / s;
        q.y = (m(2, 0) - m(0, 2)) / s;
        q.z = (m(0, 1) - m(1, 0)) / s;
    } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
        const double s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.0;
        q.w = (m(1, 2) - m(2, 1)) / s;
        q.x = 0.25 * s;
        q.y = (m(1, 0) + m(0, 1)) / s;
        q.z = (m(2, 0) + m(0, 2)) / s;
    } else if (m(1, 1) > m(2, 2)) {
        const double s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.0;
        q.w = (m(2, 0) - m(0, 2)) / s;
        q.x = (m(1, 0) + m(0, 1)) / s;
        q.y = 0.25 * s;
        q.z = (m(2, 1) + m(1, 2)) / s;
    } else {
        const double s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.0;
        q.w = (m(0, 1) - m(1, 0)) / s;
        q.x = (m(2, 0) + m(0, 2)) / s;
        q.y = (m(2, 1) + m(1, 2)) / s;
        q.z = 0.25 * s;
    }
    return q.normalized();
}

struct WorldPose {
    Vec3 translation;
    Quat rotation;
};

WorldPose worldPose(const SceneNode& node) {
    const Mat4 world = node.worldMatrix();
    return WorldPose{{world(3, 0), world(3, 1), world(3, 2)}, quatFromMatrix(world)};
}

void writePose(SceneNode& node, JPH::RVec3 origin, JPH::Quat rotation) {
    Transform pose;
    pose.translation = {origin.GetX() / kMmToM, origin.GetY() / kMmToM, origin.GetZ() / kMmToM};
    pose.rotation = fromRot(rotation);
    auto parent = node.parent.lock();
    if (!parent || parent->id == "root") {
        node.local.translation = pose.translation;
        node.local.rotation = pose.rotation;
        return;
    }
    const Mat4 local = mul(inverseAffine(parent->worldMatrix()), pose.matrix());
    node.local.translation = {local(3, 0), local(3, 1), local(3, 2)};
    node.local.rotation = quatFromMatrix(local);
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

bool pointInside(Vec3 point, const SceneNode& box) {
    const Vec3 half = box.mech.halfExtents;
    const Vec3 d = point - box.local.translation;
    return std::abs(d.x) <= half.x && std::abs(d.y) <= half.y && std::abs(d.z) <= half.z;
}

Vec3 authoredAxis(const SceneNode& node) {
    Vec3 axis = node.mech.axis;
    if (length(axis) < 1e-8) {
        axis = {1, 0, 0};
    }
    return normalize(node.local.rotation.rotate(normalize(axis)));
}

Vec3 perpendicular(Vec3 axis) {
    const Vec3 hint = std::abs(axis.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{0, 1, 0};
    return normalize(cross(axis, hint));
}

void configureMotor(JPH::MotorSettings& motor) {
    motor.mSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
    motor.mSpringSettings.mFrequency = 0.0f;
    motor.mSpringSettings.mDamping = 1.0f;
    motor.mMinForceLimit = -1.0e6f;
    motor.mMaxForceLimit = 1.0e6f;
    motor.mMinTorqueLimit = -500.0f;
    motor.mMaxTorqueLimit = 500.0f;
}

}  // namespace

struct JoltPhysicsEngine::Impl {
    struct BodyRecord {
        std::string id;
        JPH::BodyID body;
        bool dynamic = false;
        bool conveyor = false;
        Vec3 beltMeters{0, 0, 0};
    };
    struct JointRecord {
        std::string id;
        JPH::Ref<JPH::TwoBodyConstraint> constraint;
        JPH::BodyID heavy;
        JPH::BodyID light;
        bool slider = false;
        bool hinge = false;
        double lastSpeed = 0;
    };

    class Contacts final : public JPH::ContactListener {
    public:
        explicit Contacts(Impl& owner) : owner_(owner) {}
        JPH::ValidateResult OnContactValidate(const JPH::Body&, const JPH::Body&, JPH::RVec3Arg,
                                              const JPH::CollideShapeResult&) override {
            return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
        }
        void OnContactAdded(const JPH::Body& a, const JPH::Body& b, const JPH::ContactManifold&,
                            JPH::ContactSettings&) override {
            note(a, b);
        }
        void OnContactPersisted(const JPH::Body& a, const JPH::Body& b, const JPH::ContactManifold&,
                                JPH::ContactSettings&) override {
            note(a, b);
        }
        void OnContactRemoved(const JPH::SubShapeIDPair&) override {}

        void note(const JPH::Body& a, const JPH::Body& b) {
            const auto keyA = a.GetID().GetIndexAndSequenceNumber();
            const auto keyB = b.GetID().GetIndexAndSequenceNumber();
            std::lock_guard<std::mutex> lock(owner_.contactMutex_);
            const auto beltA = owner_.belts_.find(keyA);
            const auto beltB = owner_.belts_.find(keyB);
            if (beltA != owner_.belts_.end() && b.IsDynamic()) {
                owner_.pendingBelts_[keyB] = beltA->second;
            } else if (beltB != owner_.belts_.end() && a.IsDynamic()) {
                owner_.pendingBelts_[keyA] = beltB->second;
            }
        }

        Impl& owner_;
    };

    Impl() : contacts_(*this) {}

    Scene* scene = nullptr;
    BroadPhaseMap broadMap_;
    ObjectVsBroadPhase objectBroad_;
    ObjectPairFilter pairs_;
    std::unique_ptr<JPH::TempAllocatorImpl> temp_;
    std::unique_ptr<JPH::JobSystemSingleThreaded> jobs_;
    std::unique_ptr<JPH::PhysicsSystem> physics_;
    Contacts contacts_;
    JPH::BodyID anchor_;
    std::vector<BodyRecord> bodies_;
    std::vector<JointRecord> joints_;
    std::unordered_map<std::uint32_t, Vec3> belts_;
    std::unordered_map<std::uint32_t, Vec3> activeBelts_;
    std::unordered_map<std::uint32_t, Vec3> pendingBelts_;
    std::mutex contactMutex_;
    std::vector<std::pair<std::string, double>> sourceClocks_;

    BodyRecord* findBody(const std::string& id) {
        for (BodyRecord& record : bodies_) {
            if (record.id == id) {
                return &record;
            }
        }
        return nullptr;
    }

    void addBody(SceneNode& node);
    void addJoint(SceneNode& node);
    void destroyBody(const std::string& id);
    void applyDrives(double dt);
    void applyBelts();
    void syncTransforms();
    void updateSensors();
    void updateSources(double dt);
    void updateSinks();
};

void JoltPhysicsEngine::Impl::addBody(SceneNode& node) {
    const Mech& mech = node.mech;
    const bool kinematic = mech.kinematic || mech.kind == MechKind::Conveyor || mech.kind == MechKind::Collision;
    const WorldPose pose = worldPose(node);
    JPH::RefConst<JPH::Shape> shape;
    const auto boxHalf = [&](Vec3 halfMm) {
        const float hx = std::max(static_cast<float>(halfMm.x * kMmToM), 1.0e-4f);
        const float hy = std::max(static_cast<float>(halfMm.y * kMmToM), 1.0e-4f);
        const float hz = std::max(static_cast<float>(halfMm.z * kMmToM), 1.0e-4f);
        float radius = 0.001f;
        const float minHalf = std::min(hx, std::min(hy, hz));
        if (minHalf <= radius) {
            radius = std::max(minHalf * 0.2f, 1.0e-5f);
        }
        return JPH::BoxShapeSettings(JPH::Vec3(hx, hy, hz), radius).Create().Get();
    };

    if (mech.shape == CollisionShape::Sphere) {
        shape = new JPH::SphereShape(static_cast<float>(std::max(mech.radius, 0.5) * kMmToM));
    } else if (mech.shape == CollisionShape::Capsule) {
        const float radius = static_cast<float>(std::max(mech.radius, 0.5) * kMmToM);
        const float cylinder = static_cast<float>(std::max(mech.height - 2.0 * mech.radius, 0.5) * kMmToM);
        auto capsule = JPH::CapsuleShapeSettings(cylinder * 0.5f, radius).Create().Get();
        const JPH::Quat yToZ = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * JPH::JPH_PI);
        shape = new JPH::RotatedTranslatedShape(JPH::Vec3::sZero(), yToZ, capsule);
    } else if (mech.shape == CollisionShape::ConvexHull) {
        JPH::Array<JPH::Vec3> points;
        if (scene != nullptr) {
            if (const auto mesh = scene->mesh(node.meshId)) {
                const int count = static_cast<int>(mesh->vertexCount());
                const int stride = std::max(1, count / 96);
                for (int i = 0; i < count; i += stride) {
                    points.push_back(JPH::Vec3(mesh->positions[static_cast<std::size_t>(i) * 3] * static_cast<float>(kMmToM),
                                               mesh->positions[static_cast<std::size_t>(i) * 3 + 1] * static_cast<float>(kMmToM),
                                               mesh->positions[static_cast<std::size_t>(i) * 3 + 2] * static_cast<float>(kMmToM)));
                }
            }
        }
        if (points.size() >= 4) {
            auto created = JPH::ConvexHullShapeSettings(points, 0.001f).Create();
            if (created.IsValid()) {
                shape = created.Get();
            }
        }
        if (shape == nullptr) {
            shape = boxHalf(mech.halfExtents);
        }
    } else {
        shape = boxHalf(mech.halfExtents);
    }

    double mass = 0;
    if (!kinematic && mech.kind != MechKind::Collision) {
        mass = mech.autoMass ? std::max(0.05, volumeCubicMeters(mech) * 900.0) : std::max(0.01, mech.mass);
    }
    const JPH::EMotionType motion = kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic;
    const JPH::ObjectLayer layer = motion == JPH::EMotionType::Dynamic || motion == JPH::EMotionType::Kinematic
                                       ? Layers::kMoving
                                       : Layers::kStatic;
    JPH::BodyCreationSettings settings(shape, toPos(pose.translation), toRot(pose.rotation), motion, layer);
    settings.mFriction = 0.85f;
    settings.mRestitution = 0.02f;
    settings.mLinearDamping = 0.04f;
    settings.mAngularDamping = 0.2f;
    settings.mAllowSleeping = false;
    if (mass > 0) {
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = static_cast<float>(mass);
    }
    JPH::BodyInterface& bodies = physics_->GetBodyInterface();
    const JPH::BodyID id = bodies.CreateAndAddBody(settings, JPH::EActivation::Activate);
    BodyRecord record;
    record.id = node.id;
    record.body = id;
    record.dynamic = mass > 0;
    record.conveyor = mech.kind == MechKind::Conveyor;
    if (record.conveyor) {
        Vec3 direction = mech.direction;
        if (length(direction) < 1e-8) {
            direction = {1, 0, 0};
        }
        direction = normalize(pose.rotation.rotate(normalize(direction)));
        record.beltMeters = direction * (mech.speed * kMmToM);
        belts_[id.GetIndexAndSequenceNumber()] = record.beltMeters;
    }
    bodies_.push_back(record);
}

void JoltPhysicsEngine::Impl::addJoint(SceneNode& node) {
    BodyRecord* self = findBody(node.id);
    if (self == nullptr) {
        return;
    }
    BodyRecord* other = nullptr;
    if (!node.mech.other.empty() && scene != nullptr) {
        if (const auto named = scene->findByName(node.mech.other)) {
            other = findBody(named->id);
        }
    }
    const JPH::BodyID heavy = other != nullptr && !other->dynamic ? other->body
                             : other == nullptr                   ? anchor_
                             : self->dynamic                      ? other->body
                                                                  : self->body;
    const JPH::BodyID light = heavy == self->body ? (other != nullptr ? other->body : anchor_) : self->body;
    const WorldPose pose = worldPose(node);
    const Vec3 axis = authoredAxis(node);
    const Vec3 normal = perpendicular(axis);
    const JPH::RVec3 pivot = toPos(pose.translation);

    JPH::Ref<JPH::TwoBodyConstraint> constraint;
    bool slider = false;
    bool hinge = false;
    JPH::BodyLockWrite lockHeavy(physics_->GetBodyLockInterface(), heavy);
    JPH::BodyLockWrite lockLight(physics_->GetBodyLockInterface(), light);
    if (!lockHeavy.Succeeded() || !lockLight.Succeeded()) {
        return;
    }
    if (node.mech.kind == MechKind::Slider) {
        JPH::SliderConstraintSettings settings;
        settings.mSpace = JPH::EConstraintSpace::WorldSpace;
        settings.mAutoDetectPoint = false;
        settings.mPoint1 = pivot;
        settings.mPoint2 = pivot;
        settings.mSliderAxis1 = toDir(axis);
        settings.mSliderAxis2 = toDir(axis);
        settings.mNormalAxis1 = toDir(normal);
        settings.mNormalAxis2 = toDir(normal);
        float lower = node.mech.limited ? static_cast<float>(std::min(node.mech.lower, node.mech.upper) * kMmToM) : -2.0f;
        float upper = node.mech.limited ? static_cast<float>(std::max(node.mech.lower, node.mech.upper) * kMmToM) : 2.0f;
        if (lower > 0.0f) {
            lower = 0.0f;
        }
        if (upper < 0.0f) {
            upper = 0.0f;
        }
        settings.mLimitsMin = lower;
        settings.mLimitsMax = upper;
        configureMotor(settings.mMotorSettings);
        constraint = settings.Create(lockHeavy.GetBody(), lockLight.GetBody());
        slider = true;
    } else if (node.mech.kind == MechKind::Hinge) {
        JPH::HingeConstraintSettings settings;
        settings.mSpace = JPH::EConstraintSpace::WorldSpace;
        settings.mPoint1 = pivot;
        settings.mPoint2 = pivot;
        settings.mHingeAxis1 = toDir(axis);
        settings.mHingeAxis2 = toDir(axis);
        settings.mNormalAxis1 = toDir(normal);
        settings.mNormalAxis2 = toDir(normal);
        float lower = node.mech.limited ? static_cast<float>(deg2rad(std::min(node.mech.lower, node.mech.upper)))
                                        : static_cast<float>(-kPi);
        float upper = node.mech.limited ? static_cast<float>(deg2rad(std::max(node.mech.lower, node.mech.upper)))
                                        : static_cast<float>(kPi);
        lower = std::clamp(lower, static_cast<float>(-kPi), 0.0f);
        upper = std::clamp(upper, 0.0f, static_cast<float>(kPi));
        settings.mLimitsMin = lower;
        settings.mLimitsMax = upper;
        configureMotor(settings.mMotorSettings);
        constraint = settings.Create(lockHeavy.GetBody(), lockLight.GetBody());
        hinge = true;
    } else {
        JPH::FixedConstraintSettings settings;
        settings.mAutoDetectPoint = true;
        constraint = settings.Create(lockHeavy.GetBody(), lockLight.GetBody());
    }
    physics_->AddConstraint(constraint);
    joints_.push_back(JointRecord{node.id, constraint, heavy, light, slider, hinge, 0});
}

void JoltPhysicsEngine::Impl::destroyBody(const std::string& id) {
    BodyRecord* record = findBody(id);
    if (record == nullptr || physics_ == nullptr) {
        return;
    }
    const JPH::BodyID body = record->body;
    for (auto it = joints_.begin(); it != joints_.end();) {
        if (it->heavy == body || it->light == body || it->id == id) {
            physics_->RemoveConstraint(it->constraint);
            it = joints_.erase(it);
        } else {
            ++it;
        }
    }
    belts_.erase(body.GetIndexAndSequenceNumber());
    activeBelts_.erase(body.GetIndexAndSequenceNumber());
    pendingBelts_.erase(body.GetIndexAndSequenceNumber());
    JPH::BodyInterface& bodies = physics_->GetBodyInterface();
    bodies.RemoveBody(body);
    bodies.DestroyBody(body);
    bodies_.erase(std::remove_if(bodies_.begin(), bodies_.end(), [&](const BodyRecord& item) { return item.id == id; }),
                  bodies_.end());
}

void JoltPhysicsEngine::Impl::applyDrives(double dt) {
    if (scene == nullptr || dt <= 0) {
        return;
    }
    for (JointRecord& joint : joints_) {
        auto node = scene->find(joint.id);
        if (!node || node->mech.drive == DriveMode::None || joint.constraint == nullptr) {
            continue;
        }
        const double maxDelta = std::max(0.0, node->mech.maxAccel) * dt;
        if (joint.slider) {
            auto* slider = static_cast<JPH::SliderConstraint*>(joint.constraint.GetPtr());
            double desired = 0;
            if (node->mech.drive == DriveMode::Speed) {
                desired = node->mech.target;
            } else {
                const double current = slider->GetCurrentPosition() / kMmToM;
                desired = std::clamp((node->mech.target - current) / dt, -std::abs(node->mech.maxSpeed),
                                     std::abs(node->mech.maxSpeed));
            }
            desired = std::clamp(desired, joint.lastSpeed - maxDelta, joint.lastSpeed + maxDelta);
            desired = std::clamp(desired, -std::abs(node->mech.maxSpeed), std::abs(node->mech.maxSpeed));
            joint.lastSpeed = desired;
            slider->SetMotorState(JPH::EMotorState::Velocity);
            slider->SetTargetVelocity(static_cast<float>(desired * kMmToM));
        } else if (joint.hinge) {
            auto* hinge = static_cast<JPH::HingeConstraint*>(joint.constraint.GetPtr());
            const double current = rad2deg(hinge->GetCurrentAngle());
            double speed = 0;
            if (node->mech.drive == DriveMode::Speed) {
                speed = std::clamp(node->mech.target, -std::abs(node->mech.maxSpeed), std::abs(node->mech.maxSpeed));
            } else {
                double step = std::clamp(node->mech.target - current, -std::abs(node->mech.maxSpeed) * dt,
                                         std::abs(node->mech.maxSpeed) * dt);
                const double accelStep = std::max(0.0, node->mech.maxAccel) * dt * dt;
                const double previousStep = joint.lastSpeed * dt;
                step = std::clamp(step, previousStep - accelStep, previousStep + accelStep);
                speed = dt > 0 ? step / dt : 0;
            }
            joint.lastSpeed = speed;
            hinge->SetMotorState(JPH::EMotorState::Velocity);
            hinge->SetTargetAngularVelocity(static_cast<float>(deg2rad(speed)));
        }
    }
}

void JoltPhysicsEngine::Impl::applyBelts() {
    JPH::BodyInterface& bodies = physics_->GetBodyInterface();
    for (const auto& [key, belt] : activeBelts_) {
        const JPH::BodyID id(key);
        if (!bodies.IsAdded(id)) {
            continue;
        }
        JPH::Vec3 velocity = bodies.GetLinearVelocity(id);
        velocity.SetX(static_cast<float>(belt.x));
        velocity.SetY(static_cast<float>(belt.y));
        if (std::abs(belt.z) > 1e-6) {
            velocity.SetZ(static_cast<float>(belt.z));
        }
        bodies.SetLinearVelocity(id, velocity);
        bodies.ActivateBody(id);
    }
}

void JoltPhysicsEngine::Impl::syncTransforms() {
    if (scene == nullptr || physics_ == nullptr) {
        return;
    }
    JPH::BodyInterface& bodies = physics_->GetBodyInterface();
    for (const BodyRecord& record : bodies_) {
        auto node = scene->find(record.id);
        if (!node) {
            continue;
        }
        if (!record.dynamic && !mechIsJoint(node->mech.kind)) {
            continue;
        }
        writePose(*node, bodies.GetPosition(record.body), bodies.GetRotation(record.body));
    }
}

void JoltPhysicsEngine::Impl::updateSensors() {
    if (scene == nullptr || physics_ == nullptr) {
        return;
    }
        const JPH::NarrowPhaseQuery& query = physics_->GetNarrowPhaseQuery();
    std::vector<SceneNode*> nodes;
    collect(scene->root(), nodes);
    IgnoreBodyFilter ignore(anchor_);
    for (SceneNode* node : nodes) {
        if (node->mech.kind != MechKind::CollisionSensor) {
            continue;
        }
        const WorldPose pose = worldPose(*node);
        const auto shape = JPH::BoxShapeSettings(JPH::Vec3(static_cast<float>(std::max(node->mech.halfExtents.x, 0.5) * kMmToM),
                                                          static_cast<float>(std::max(node->mech.halfExtents.y, 0.5) * kMmToM),
                                                          static_cast<float>(std::max(node->mech.halfExtents.z, 0.5) * kMmToM)),
                                                 0.001f)
                               .Create()
                               .Get();
        class AnyOther final : public JPH::CollideShapeCollector {
        public:
            bool hit = false;
            void AddHit(const ResultType&) override {
                hit = true;
                ForceEarlyOut();
            }
        } collector;
        JPH::CollideShapeSettings settings;
        settings.mMaxSeparationDistance = 0.0f;
        query.CollideShape(shape, JPH::Vec3::sReplicate(1.0f),
                           JPH::RMat44::sRotationTranslation(toRot(pose.rotation), toPos(pose.translation)), settings,
                           JPH::RVec3::sZero(), collector, {}, {}, ignore);
        const std::string signalName = node->mech.signal.empty() ? node->name : node->mech.signal;
        Signal& signal = scene->upsertSignal(signalName, SignalType::Bool);
        signal.flag = collector.hit;
    }
    for (SceneNode* node : nodes) {
        if (node->mech.kind != MechKind::DistanceSensor && node->mech.kind != MechKind::LightBarrier) {
            continue;
        }
        Vec3 axis = node->mech.axis;
        if (length(axis) < 1e-8) {
            axis = {0, 1, 0};
        }
        axis = normalize(node->local.rotation.rotate(normalize(axis)));
        const double range = std::max(node->mech.range, 1.0);
        const WorldPose pose = worldPose(*node);
        const JPH::RVec3 origin = toPos(pose.translation);
        const JPH::Vec3 delta = toDir(axis * (range * kMmToM));
        JPH::RRayCast ray(origin, delta);
        JPH::RayCastResult hit;
        const bool blocked = query.CastRay(ray, hit, {}, {}, ignore);
        const double distance = blocked ? static_cast<double>(hit.mFraction) * range : range;
        const std::string signalName = node->mech.signal.empty() ? node->name : node->mech.signal;
        Signal& signal = scene->upsertSignal(signalName, SignalType::Bool);
        signal.flag = blocked;
        if (!node->mech.valueSignal.empty()) {
            Signal& value = scene->upsertSignal(node->mech.valueSignal, SignalType::Real);
            value.real = distance;
        }
    }
}

void JoltPhysicsEngine::Impl::updateSources(double dt) {
    if (scene == nullptr) {
        return;
    }
    for (auto& [id, clock] : sourceClocks_) {
        auto source = scene->find(id);
        if (!source || source->mech.prototype.empty()) {
            continue;
        }
        clock += dt;
        const double interval = std::max(source->mech.interval, 0.05);
        if (clock < interval) {
            continue;
        }
        clock = 0;
        auto prototype = scene->findByName(source->mech.prototype);
        if (!prototype) {
            continue;
        }
        auto clone = scene->spawnClone(prototype, source->local);
        if (clone) {
            addBody(*clone);
        }
    }
}

void JoltPhysicsEngine::Impl::updateSinks() {
    if (scene == nullptr || physics_ == nullptr) {
        return;
    }
    std::vector<SceneNode*> nodes;
    collect(scene->root(), nodes);
    std::vector<std::string> doomed;
    JPH::BodyInterface& bodies = physics_->GetBodyInterface();
    for (SceneNode* sink : nodes) {
        if (sink->mech.kind != MechKind::Sink) {
            continue;
        }
        for (const BodyRecord& record : bodies_) {
            if (!record.dynamic) {
                continue;
            }
            auto node = scene->find(record.id);
            if (!node || !node->transient) {
                continue;
            }
            const JPH::RVec3 origin = bodies.GetPosition(record.body);
            const Vec3 point{origin.GetX() / kMmToM, origin.GetY() / kMmToM, origin.GetZ() / kMmToM};
            if (pointInside(point, *sink)) {
                doomed.push_back(record.id);
            }
        }
    }
    std::sort(doomed.begin(), doomed.end());
    doomed.erase(std::unique(doomed.begin(), doomed.end()), doomed.end());
    for (const std::string& id : doomed) {
        destroyBody(id);
        if (scene != nullptr) {
            scene->removeNode(id);
        }
    }
}

JoltPhysicsEngine::JoltPhysicsEngine() : impl_(std::make_unique<Impl>()) { ensureJolt(); }

JoltPhysicsEngine::~JoltPhysicsEngine() { shutdown(); }

bool JoltPhysicsEngine::build(Scene& scene, std::string& error) {
    shutdown();
    impl_ = std::make_unique<Impl>();
    ensureJolt();
    Impl& world = *impl_;
    world.scene = &scene;
    world.temp_ = std::make_unique<JPH::TempAllocatorImpl>(64 * 1024 * 1024);
    world.jobs_ = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    world.physics_ = std::make_unique<JPH::PhysicsSystem>();
    world.physics_->Init(8192, 0, 65536, 20480, world.broadMap_, world.objectBroad_, world.pairs_);
    world.physics_->SetGravity(JPH::Vec3(0, 0, -9.81f));
    world.physics_->SetContactListener(&world.contacts_);
    JPH::PhysicsSettings settings = world.physics_->GetPhysicsSettings();
    settings.mNumVelocitySteps = 16;
    settings.mNumPositionSteps = 3;
    world.physics_->SetPhysicsSettings(settings);

    JPH::BodyCreationSettings anchorSettings(new JPH::SphereShape(0.02f), JPH::RVec3::sZero(), JPH::Quat::sIdentity(),
                                             JPH::EMotionType::Static, Layers::kAnchor);
    world.anchor_ = world.physics_->GetBodyInterface().CreateAndAddBody(anchorSettings, JPH::EActivation::DontActivate);

    std::vector<SceneNode*> nodes;
    collect(scene.root(), nodes);
    for (SceneNode* node : nodes) {
        if (node->visible && mechCreatesBody(node->mech.kind)) {
            world.addBody(*node);
        }
    }
    for (SceneNode* node : nodes) {
        if (node->visible && mechIsJoint(node->mech.kind)) {
            world.addJoint(*node);
        }
    }
    for (SceneNode* node : nodes) {
        if (node->mech.kind == MechKind::Source) {
            world.sourceClocks_.emplace_back(node->id, 0);
        }
    }
    error.clear();
    return true;
}

void JoltPhysicsEngine::step(double dtSeconds) {
    Impl& world = *impl_;
    if (world.physics_ == nullptr || dtSeconds <= 0) {
        return;
    }
    world.applyDrives(dtSeconds);
    world.applyBelts();
    {
        std::lock_guard<std::mutex> lock(world.contactMutex_);
        world.pendingBelts_.clear();
    }
    world.physics_->Update(static_cast<float>(dtSeconds), 1, world.temp_.get(), world.jobs_.get());
    {
        std::lock_guard<std::mutex> lock(world.contactMutex_);
        world.activeBelts_ = world.pendingBelts_;
    }
    world.syncTransforms();
    world.updateSensors();
    world.updateSources(dtSeconds);
    world.updateSinks();
}

void JoltPhysicsEngine::adoptSpawned(const std::string& id) {
    if (impl_->physics_ == nullptr || impl_->scene == nullptr || impl_->findBody(id) != nullptr) {
        return;
    }
    if (auto node = impl_->scene->find(id)) {
        impl_->addBody(*node);
    }
}

void JoltPhysicsEngine::releaseBody(const std::string& id) { impl_->destroyBody(id); }

void JoltPhysicsEngine::shutdown() {
    if (!impl_ || impl_->physics_ == nullptr) {
        return;
    }
    Impl& world = *impl_;
    world.physics_->SetContactListener(nullptr);
    for (Impl::JointRecord& joint : world.joints_) {
        world.physics_->RemoveConstraint(joint.constraint);
    }
    world.joints_.clear();
    JPH::BodyInterface& bodies = world.physics_->GetBodyInterface();
    for (const Impl::BodyRecord& record : world.bodies_) {
        bodies.RemoveBody(record.body);
        bodies.DestroyBody(record.body);
    }
    world.bodies_.clear();
    if (!world.anchor_.IsInvalid()) {
        bodies.RemoveBody(world.anchor_);
        bodies.DestroyBody(world.anchor_);
        world.anchor_ = JPH::BodyID();
    }
    world.physics_.reset();
    world.jobs_.reset();
    world.temp_.reset();
    world.belts_.clear();
    world.activeBelts_.clear();
    world.pendingBelts_.clear();
    world.sourceClocks_.clear();
    world.scene = nullptr;
}

std::unique_ptr<IPhysicsEngine> createPhysicsEngine() { return std::make_unique<JoltPhysicsEngine>(); }

}  // namespace sim
