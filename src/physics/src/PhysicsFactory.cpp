#include "sim/physics/PhysicsEngine.hpp"

#ifndef SIM_PHYSICS_NO_JOLT
#include "sim/physics/JoltPhysicsEngine.hpp"
#endif

namespace sim {

const char* physicsBackendName(PhysicsBackend backend) {
    switch (backend) {
        case PhysicsBackend::Jolt: return "jolt";
        case PhysicsBackend::Builtin: return "builtin";
    }
    return "builtin";
}

std::unique_ptr<IPhysicsEngine> createBuiltinPhysicsEngine();

std::unique_ptr<IPhysicsEngine> createPhysicsEngine(PhysicsBackend backend) {
    if (backend == PhysicsBackend::Builtin) {
        return createBuiltinPhysicsEngine();
    }
#ifndef SIM_PHYSICS_NO_JOLT
    return std::make_unique<JoltPhysicsEngine>();
#else
    return createBuiltinPhysicsEngine();
#endif
}

std::unique_ptr<IPhysicsEngine> createPhysicsEngine() {
#ifdef SIM_PHYSICS_NO_JOLT
    return createPhysicsEngine(PhysicsBackend::Builtin);
#else
    return createPhysicsEngine(PhysicsBackend::Jolt);
#endif
}

}  // namespace sim
