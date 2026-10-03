#include "sim/physics/PhysicsEngine.hpp"

namespace sim {

std::unique_ptr<IPhysicsEngine> createPhysicsEngine() { return std::make_unique<NullPhysicsEngine>(); }

}  // namespace sim
