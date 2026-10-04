#pragma once

#include "sim/core/Math.hpp"

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <string>

namespace sim {

// One mechanism record per scene node. Unused fields stay at defaults so the
// project file can round-trip every MCD type with the same schema.
enum class MechKind {
    None,
    Rigid,
    Collision,
    FixedJoint,
    Hinge,
    Slider,
    Ball,
    Conveyor,
    CollisionSensor,
    DistanceSensor,
    LightBarrier,
    Source,
    Sink
};

enum class CollisionShape { Box, Sphere, Capsule, ConvexHull, Cylinder, Extrusion, Mesh };

// How a rigid body is integrated. Anchored never moves. Driven follows a motor
// or joint and can push free bodies. Free is full rigid-body motion. Surface
// stays on a conveyor until a handoff volume, then becomes Free.
enum class BodyMotion { Anchored, Driven, Free, Surface };

enum class DriveMode { None, Position, Speed };

enum class SignalType { Bool, Int, Real };

struct Mech {
    MechKind kind = MechKind::None;
    BodyMotion motion = BodyMotion::Free;
    // When true, a free body that is resting on a conveyor switches back to Surface.
    bool surfaceRide = false;
    double mass = 1;
    bool autoMass = true;
    // kg/m^3. Used when autoMass is set. Scene lengths stay millimeters.
    double density = 900;
    bool kinematic = false;
    double margin = 0.2;
    // Child shapes are solved on the nearest ancestor that creates a body.
    bool joinAncestor = false;
    std::string surface;
    double linearDamp = 0.04;
    double angularDamp = 0.2;
    bool continuous = false;
    bool allowSleep = true;
    bool asleep = false;
    CollisionShape shape = CollisionShape::Box;
    Vec3 halfExtents{25, 25, 25};
    double radius = 25;
    double height = 50;
    std::string other;
    Vec3 axis{1, 0, 0};
    // Initial angular velocity of a ball joint, degrees per second, parent frame.
    Vec3 spin{0, 0, 0};
    bool limited = true;
    double lower = 0;
    double upper = 0;
    DriveMode drive = DriveMode::None;
    double target = 0;
    double maxSpeed = 200;
    double maxAccel = 2000;
    // 0 means the joint limit is not force-limited.
    double maxTorque = 0;
    double maxForce = 0;
    Vec3 direction{1, 0, 0};
    double speed = 150;
    // Conveyor side-guide half-width. 0 disables the guide.
    double guide = 0;
    // Scales the friction pair when a load rides this surface.
    double grip = 1;
    std::string nextName;
    // Collision volume that forces a Surface body into Free.
    bool handoff = false;
    std::string motor;
    double range = 300;
    std::string signal;
    std::string valueSignal;
    double interval = 1.5;
    std::string prototype;
};

struct Signal {
    std::string name;
    SignalType type = SignalType::Bool;
    bool flag = false;
    std::int64_t integer = 0;
    double real = 0;
};

const char* mechKindName(MechKind kind);
bool mechKindFromName(const std::string& name, MechKind& out);
const char* collisionShapeName(CollisionShape shape);
bool collisionShapeFromName(const std::string& name, CollisionShape& out);
const char* bodyMotionName(BodyMotion motion);
bool bodyMotionFromName(const std::string& name, BodyMotion& out);
// Driven, Surface, and an explicit kinematic flag all count as non-free.
BodyMotion effectiveMotion(const Mech& mech);
const char* driveModeName(DriveMode mode);
bool driveModeFromName(const std::string& name, DriveMode& out);
const char* signalTypeName(SignalType type);
bool signalTypeFromName(const std::string& name, SignalType& out);

nlohmann::json mechToJson(const Mech& mech);
void mechFromJson(const nlohmann::json& json, Mech& mech);
nlohmann::json signalToJson(const Signal& signal);
void signalFromJson(const nlohmann::json& json, Signal& signal);

bool mechIsJoint(MechKind kind);
bool mechCreatesBody(MechKind kind);

}  // namespace sim
