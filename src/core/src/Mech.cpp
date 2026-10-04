#include "sim/core/Mech.hpp"

#include <nlohmann/json.hpp>

namespace sim {
namespace {

template <typename Enum>
Enum pick(bool (*parse)(const std::string&, Enum&), const nlohmann::json& json, const char* key, Enum fallback) {
    if (!json.contains(key) || !json.at(key).is_string()) {
        return fallback;
    }
    Enum value = fallback;
    if (!parse(json.at(key).get<std::string>(), value)) {
        return fallback;
    }
    return value;
}

Vec3 vec3Or(const nlohmann::json& json, const char* key, Vec3 fallback) {
    if (!json.contains(key) || !json.at(key).is_array() || json.at(key).size() != 3) {
        return fallback;
    }
    const auto& value = json.at(key);
    return {value[0].get<double>(), value[1].get<double>(), value[2].get<double>()};
}

}  // namespace

const char* mechKindName(MechKind kind) {
    switch (kind) {
        case MechKind::None: return "none";
        case MechKind::Rigid: return "rigid";
        case MechKind::Collision: return "collision";
        case MechKind::FixedJoint: return "fixed";
        case MechKind::Hinge: return "hinge";
        case MechKind::Slider: return "slider";
        case MechKind::Ball: return "ball";
        case MechKind::Conveyor: return "conveyor";
        case MechKind::CollisionSensor: return "collisionSensor";
        case MechKind::DistanceSensor: return "distanceSensor";
        case MechKind::LightBarrier: return "lightBarrier";
        case MechKind::Source: return "source";
        case MechKind::Sink: return "sink";
    }
    return "none";
}

bool mechKindFromName(const std::string& name, MechKind& out) {
    const MechKind kinds[] = {MechKind::None,        MechKind::Rigid,           MechKind::Collision,
                              MechKind::FixedJoint,  MechKind::Hinge,           MechKind::Slider,
                              MechKind::Ball,        MechKind::Conveyor,        MechKind::CollisionSensor,
                              MechKind::DistanceSensor,
                              MechKind::LightBarrier, MechKind::Source,         MechKind::Sink};
    for (MechKind kind : kinds) {
        if (name == mechKindName(kind)) {
            out = kind;
            return true;
        }
    }
    return false;
}

const char* collisionShapeName(CollisionShape shape) {
    switch (shape) {
        case CollisionShape::Box: return "box";
        case CollisionShape::Sphere: return "sphere";
        case CollisionShape::Capsule: return "capsule";
        case CollisionShape::ConvexHull: return "convex";
        case CollisionShape::Cylinder: return "cylinder";
        case CollisionShape::Extrusion: return "extrusion";
        case CollisionShape::Mesh: return "mesh";
    }
    return "box";
}

bool collisionShapeFromName(const std::string& name, CollisionShape& out) {
    const CollisionShape shapes[] = {CollisionShape::Box,      CollisionShape::Sphere,    CollisionShape::Capsule,
                                     CollisionShape::ConvexHull, CollisionShape::Cylinder, CollisionShape::Extrusion,
                                     CollisionShape::Mesh};
    for (CollisionShape shape : shapes) {
        if (name == collisionShapeName(shape)) {
            out = shape;
            return true;
        }
    }
    return false;
}

const char* driveModeName(DriveMode mode) {
    switch (mode) {
        case DriveMode::None: return "none";
        case DriveMode::Position: return "position";
        case DriveMode::Speed: return "speed";
    }
    return "none";
}

bool driveModeFromName(const std::string& name, DriveMode& out) {
    const DriveMode modes[] = {DriveMode::None, DriveMode::Position, DriveMode::Speed};
    for (DriveMode mode : modes) {
        if (name == driveModeName(mode)) {
            out = mode;
            return true;
        }
    }
    return false;
}

const char* signalTypeName(SignalType type) {
    switch (type) {
        case SignalType::Bool: return "bool";
        case SignalType::Int: return "int";
        case SignalType::Real: return "real";
    }
    return "bool";
}

bool signalTypeFromName(const std::string& name, SignalType& out) {
    const SignalType types[] = {SignalType::Bool, SignalType::Int, SignalType::Real};
    for (SignalType type : types) {
        if (name == signalTypeName(type)) {
            out = type;
            return true;
        }
    }
    return false;
}

bool mechIsJoint(MechKind kind) {
    return kind == MechKind::FixedJoint || kind == MechKind::Hinge || kind == MechKind::Slider || kind == MechKind::Ball;
}

bool mechCreatesBody(MechKind kind) {
    return kind == MechKind::Rigid || kind == MechKind::Collision || mechIsJoint(kind) || kind == MechKind::Conveyor;
}

nlohmann::json mechToJson(const Mech& mech) {
    return nlohmann::json{
        {"kind", mechKindName(mech.kind)},
        {"motion", bodyMotionName(mech.motion)},
        {"surfaceRide", mech.surfaceRide},
        {"mass", mech.mass},
        {"autoMass", mech.autoMass},
        {"density", mech.density},
        {"kinematic", mech.kinematic},
        {"margin", mech.margin},
        {"joinAncestor", mech.joinAncestor},
        {"surface", mech.surface},
        {"linearDamp", mech.linearDamp},
        {"angularDamp", mech.angularDamp},
        {"continuous", mech.continuous},
        {"allowSleep", mech.allowSleep},
        {"shape", collisionShapeName(mech.shape)},
        {"halfExtents", {mech.halfExtents.x, mech.halfExtents.y, mech.halfExtents.z}},
        {"radius", mech.radius},
        {"height", mech.height},
        {"other", mech.other},
        {"axis", {mech.axis.x, mech.axis.y, mech.axis.z}},
        {"spin", {mech.spin.x, mech.spin.y, mech.spin.z}},
        {"limited", mech.limited},
        {"lower", mech.lower},
        {"upper", mech.upper},
        {"drive", driveModeName(mech.drive)},
        {"target", mech.target},
        {"maxSpeed", mech.maxSpeed},
        {"maxAccel", mech.maxAccel},
        {"maxTorque", mech.maxTorque},
        {"maxForce", mech.maxForce},
        {"direction", {mech.direction.x, mech.direction.y, mech.direction.z}},
        {"speed", mech.speed},
        {"guide", mech.guide},
        {"grip", mech.grip},
        {"next", mech.nextName},
        {"handoff", mech.handoff},
        {"motor", mech.motor},
        {"range", mech.range},
        {"signal", mech.signal},
        {"valueSignal", mech.valueSignal},
        {"interval", mech.interval},
        {"prototype", mech.prototype},
    };
}

void mechFromJson(const nlohmann::json& json, Mech& mech) {
    if (!json.is_object()) {
        return;
    }
    mech.kind = pick(mechKindFromName, json, "kind", mech.kind);
    mech.mass = json.value("mass", mech.mass);
    mech.autoMass = json.value("autoMass", mech.autoMass);
    mech.density = json.value("density", mech.density);
    mech.kinematic = json.value("kinematic", mech.kinematic);
    mech.margin = json.value("margin", mech.margin);
    mech.joinAncestor = json.value("joinAncestor", mech.joinAncestor);
    mech.surface = json.value("surface", mech.surface);
    mech.surfaceRide = json.value("surfaceRide", mech.surfaceRide);
    mech.linearDamp = json.value("linearDamp", mech.linearDamp);
    mech.angularDamp = json.value("angularDamp", mech.angularDamp);
    mech.continuous = json.value("continuous", mech.continuous);
    mech.allowSleep = json.value("allowSleep", mech.allowSleep);
    if (json.contains("motion")) {
        mech.motion = pick(bodyMotionFromName, json, "motion", mech.motion);
    } else if (mech.kinematic) {
        mech.motion = BodyMotion::Driven;
    } else if (mech.kind == MechKind::Rigid) {
        mech.motion = BodyMotion::Free;
    } else {
        mech.motion = BodyMotion::Anchored;
    }
    mech.shape = pick(collisionShapeFromName, json, "shape", mech.shape);
    mech.halfExtents = vec3Or(json, "halfExtents", mech.halfExtents);
    mech.radius = json.value("radius", mech.radius);
    mech.height = json.value("height", mech.height);
    mech.other = json.value("other", mech.other);
    mech.axis = vec3Or(json, "axis", mech.axis);
    mech.spin = vec3Or(json, "spin", mech.spin);
    mech.limited = json.value("limited", mech.limited);
    mech.lower = json.value("lower", mech.lower);
    mech.upper = json.value("upper", mech.upper);
    mech.drive = pick(driveModeFromName, json, "drive", mech.drive);
    mech.target = json.value("target", mech.target);
    mech.maxSpeed = json.value("maxSpeed", mech.maxSpeed);
    mech.maxAccel = json.value("maxAccel", mech.maxAccel);
    mech.maxTorque = json.value("maxTorque", mech.maxTorque);
    mech.maxForce = json.value("maxForce", mech.maxForce);
    mech.guide = json.value("guide", mech.guide);
    mech.grip = json.value("grip", mech.grip);
    mech.nextName = json.value("next", mech.nextName);
    mech.handoff = json.value("handoff", mech.handoff);
    mech.motor = json.value("motor", mech.motor);
    mech.direction = vec3Or(json, "direction", mech.direction);
    mech.speed = json.value("speed", mech.speed);
    mech.range = json.value("range", mech.range);
    mech.signal = json.value("signal", mech.signal);
    mech.valueSignal = json.value("valueSignal", mech.valueSignal);
    mech.interval = json.value("interval", mech.interval);
    mech.prototype = json.value("prototype", mech.prototype);
}

nlohmann::json signalToJson(const Signal& signal) {
    nlohmann::json json{{"name", signal.name}, {"type", signalTypeName(signal.type)}};
    if (signal.type == SignalType::Bool) {
        json["value"] = signal.flag;
    } else if (signal.type == SignalType::Int) {
        json["value"] = signal.integer;
    } else {
        json["value"] = signal.real;
    }
    return json;
}

void signalFromJson(const nlohmann::json& json, Signal& signal) {
    if (!json.is_object()) {
        return;
    }
    signal.name = json.value("name", std::string{});
    signal.type = pick(signalTypeFromName, json, "type", SignalType::Bool);
    if (!json.contains("value")) {
        return;
    }
    const auto& value = json.at("value");
    if (signal.type == SignalType::Bool) {
        signal.flag = value.is_boolean() ? value.get<bool>() : value.get<double>() != 0;
    } else if (signal.type == SignalType::Int) {
        signal.integer = value.is_number_integer() ? value.get<std::int64_t>() : static_cast<std::int64_t>(value.get<double>());
    } else {
        signal.real = value.get<double>();
    }
}

}  // namespace sim
