#pragma once

#include "sim/physics/PhysicsEngine.hpp"

class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btDiscreteDynamicsWorld;
class btGhostPairCallback;
class btCollisionShape;
class btRigidBody;
class btTypedConstraint;
class btGhostObject;

namespace sim {

class BulletPhysicsEngine final : public IPhysicsEngine {
public:
    BulletPhysicsEngine();
    ~BulletPhysicsEngine() override;

    const char* backendName() const override { return "bullet"; }
    bool build(Scene& scene, std::string& error) override;
    void step(double dtSeconds) override;
    void shutdown() override;
    void adoptSpawned(const std::string& id) override;
    void releaseBody(const std::string& id) override;

private:
    struct BodyRecord {
        std::string id;
        btRigidBody* body = nullptr;
        btCollisionShape* shape = nullptr;
        bool dynamic = false;
        bool conveyor = false;
        Vec3 beltMeters{0, 0, 0};
    };
    struct JointRecord {
        std::string id;
        btTypedConstraint* constraint = nullptr;
        bool slider = false;
        bool hinge = false;
        double lastSpeed = 0;
    };
    struct GhostRecord {
        std::string id;
        btGhostObject* ghost = nullptr;
        btCollisionShape* shape = nullptr;
    };

    void addBody(SceneNode& node);
    void addJoint(SceneNode& node);
    void addGhost(SceneNode& node);
    void applyDrives(double dt);
    void applyConveyors();
    void syncTransforms();
    void updateSensors();
    void updateSources(double dt);
    void updateSinks();
    void destroyBody(const std::string& id);

    Scene* scene_ = nullptr;
    btCollisionConfiguration* config_ = nullptr;
    btCollisionDispatcher* dispatcher_ = nullptr;
    btBroadphaseInterface* broadphase_ = nullptr;
    btConstraintSolver* solver_ = nullptr;
    btDiscreteDynamicsWorld* world_ = nullptr;
    btGhostPairCallback* ghosts_ = nullptr;
    btRigidBody* anchor_ = nullptr;
    btCollisionShape* anchorShape_ = nullptr;
    std::vector<BodyRecord> bodies_;
    std::vector<JointRecord> joints_;
    std::vector<GhostRecord> ghostRecords_;
    std::vector<std::pair<std::string, double>> sourceClocks_;
};

}  // namespace sim
