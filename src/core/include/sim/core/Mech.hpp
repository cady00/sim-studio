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
    Conveyor,
    CollisionSensor,
    DistanceSensor,
    LightBarrier,
    Source,
    Sink
};

enum class CollisionShape { Box, Sphere, Capsule, ConvexHull };

enum class DriveMode { None, Position, Speed };

enum class SignalType { Bool, Int, Real };

struct Mech {
    MechKind kind = MechKind::None;
    double mass = 1;
    bool autoMass = true;
    bool kinematic = false;
    CollisionShape shape = CollisionShape::Box;
    Vec3 halfExtents{25, 25, 25};
    double radius = 25;
    double height = 50;
    std::string other;
    Vec3 axis{1, 0, 0};
    bool limited = true;
    double lower = 0;
    double upper = 0;
    DriveMode drive = DriveMode::None;
    double target = 0;
    double maxSpeed = 200;
    double maxAccel = 2000;
    Vec3 direction{1, 0, 0};
    double speed = 150;
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
