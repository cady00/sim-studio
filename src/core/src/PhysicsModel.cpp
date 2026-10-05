#include "sim/core/PhysicsModel.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

double clamp01(double value) { return std::clamp(value, 0.0, 1.0); }

double gradeFriction(const PhysicsDocument& document, const std::string& name, bool& found) {
    for (const SurfaceGrade& grade : document.grades) {
        if (grade.name == name) {
            found = true;
            return grade.friction;
        }
    }
    found = false;
    return document.defaultFriction;
}

double volumeCubicMillimeters(const Mech& mech) {
    const double hx = std::max(mech.halfExtents.x, 0.1);
    const double hy = std::max(mech.halfExtents.y, 0.1);
    const double hz = std::max(mech.halfExtents.z, 0.1);
    const double radius = std::max(mech.radius, 0.1);
    const double height = std::max(mech.height, 0.1);
    switch (mech.shape) {
        case CollisionShape::Sphere:
            return 4.0 / 3.0 * kPi * radius * radius * radius;
        case CollisionShape::Capsule: {
            const double cylinder = kPi * radius * radius * std::max(height - 2.0 * radius, 0.0);
            const double caps = 4.0 / 3.0 * kPi * radius * radius * radius;
            return cylinder + caps;
        }
        case CollisionShape::Cylinder:
        case CollisionShape::Extrusion:
            return kPi * radius * radius * height;
        case CollisionShape::Box:
        case CollisionShape::ConvexHull:
        case CollisionShape::Mesh:
            return 8.0 * hx * hy * hz;
    }
    return 8.0 * hx * hy * hz;
}

std::vector<double> projectBounds(std::vector<double> values, const std::vector<double>& lower, const std::vector<double>& upper) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        const double lo = i < lower.size() ? lower[i] : -1e9;
        const double hi = i < upper.size() ? upper[i] : 1e9;
        values[i] = std::clamp(values[i], std::min(lo, hi), std::max(lo, hi));
    }
    return values;
}

double residualNorm(const std::vector<double>& residual) {
    double sum = 0;
    for (double value : residual) {
        sum += value * value;
    }
    return std::sqrt(sum);
}

}  // namespace

const char* bodyMotionName(BodyMotion motion) {
    switch (motion) {
        case BodyMotion::Anchored: return "anchored";
        case BodyMotion::Driven: return "driven";
        case BodyMotion::Free: return "free";
        case BodyMotion::Surface: return "surface";
    }
    return "free";
}

bool bodyMotionFromName(const std::string& name, BodyMotion& out) {
    const BodyMotion values[] = {BodyMotion::Anchored, BodyMotion::Driven, BodyMotion::Free, BodyMotion::Surface};
    for (BodyMotion motion : values) {
        if (name == bodyMotionName(motion)) {
            out = motion;
            return true;
        }
    }
    return false;
}

BodyMotion effectiveMotion(const Mech& mech) {
    if (mech.motion == BodyMotion::Surface || mech.surfaceRide) {
        if (mech.motion == BodyMotion::Surface) {
            return BodyMotion::Surface;
        }
    }
    if (mech.motion == BodyMotion::Anchored) {
        return BodyMotion::Anchored;
    }
    if (mech.motion == BodyMotion::Driven || mech.kinematic) {
        return BodyMotion::Driven;
    }
    if (mech.motion == BodyMotion::Surface) {
        return BodyMotion::Surface;
    }
    return BodyMotion::Free;
}

void syncMotionFlags(Mech& mech) {
    mech.kinematic = mech.motion != BodyMotion::Free;
}

double stoppingDistance(double speed, double decel) {
    if (decel <= 1e-9) {
        return std::abs(speed) <= 1e-9 ? 0 : 1e12;
    }
    return (speed * speed) / (2.0 * decel);
}

double stoppingTime(double speed, double decel) {
    if (decel <= 1e-9) {
        return std::abs(speed) <= 1e-9 ? 0 : 1e12;
    }
    return std::abs(speed) / decel;
}

double timeToSpeed(double from, double to, double rate) {
    if (rate <= 1e-9) {
        return std::abs(to - from) <= 1e-9 ? 0 : 1e12;
    }
    return std::abs(to - from) / rate;
}

double pairFriction(const PhysicsDocument& document, const std::string& a, const std::string& b) {
    for (const GradePair& pair : document.pairs) {
        if ((pair.a == a && pair.b == b) || (pair.a == b && pair.b == a)) {
            return pair.friction;
        }
    }
    bool foundA = false;
    bool foundB = false;
    const double frictionA = gradeFriction(document, a, foundA);
    const double frictionB = gradeFriction(document, b, foundB);
    if (foundA && foundB) {
        return std::sqrt(std::max(frictionA, 0.0) * std::max(frictionB, 0.0));
    }
    return document.defaultFriction;
}

double pairRestitution(const PhysicsDocument& document, const std::string& a, const std::string& b) {
    for (const GradePair& pair : document.pairs) {
        if ((pair.a == a && pair.b == b) || (pair.a == b && pair.b == a)) {
            return pair.restitution;
        }
    }
    return document.defaultRestitution;
}

double conveyorScale(const PhysicsDocument& document, const std::string& load, const std::string& belt, double grip) {
    const double friction = pairFriction(document, load, belt);
    const double reference = std::max(document.referenceFriction, 1e-4);
    return clamp01(friction * std::max(grip, 0.0) / reference);
}

double massKilograms(const Mech& mech) {
    if (!mech.autoMass) {
        return std::max(mech.mass, 0.0);
    }
    return std::max(volumeCubicMillimeters(mech) * mech.density * 1e-9, 0.0);
}

EncoderSample sampleEncoder(const MotorAspect& motor) {
    EncoderSample sample;
    const double pitch = std::max(motor.pulseDistance, 1e-6);
    const double turns = motor.distance / pitch;
    std::int64_t count = static_cast<std::int64_t>(std::floor(turns));
    if (motor.maxCount > motor.minCount) {
        count = std::clamp(count, motor.minCount, motor.maxCount);
    }
    sample.count = count;
    const double fraction = turns - std::floor(turns);
    sample.pulseHigh = fraction >= 0 && fraction < clamp01(motor.highRatio);
    const std::int64_t gray = count ^ (count >> 1);
    sample.gray = motor.grayCode ? gray : count;
    return sample;
}

MotorEvent advanceMotor(MotorAspect& motor, double dt) {
    MotorEvent event;
    const double previous = motor.speed;
    const bool wasRunning = motor.running;
    if (motor.locked || dt <= 0) {
        motor.speed = 0;
    } else {
        const double goal = motor.running ? motor.targetSpeed * (motor.direction < 0 ? -1.0 : 1.0) : 0.0;
        const double rate = std::abs(goal) >= std::abs(motor.speed) ? std::max(motor.accel, 0.0) : std::max(motor.decel, 0.0);
        const double step = rate * dt;
        if (motor.speed < goal) {
            motor.speed = std::min(goal, motor.speed + step);
        } else {
            motor.speed = std::max(goal, motor.speed - step);
        }
    }
    motor.distance += motor.speed * dt;
    if (motor.limited) {
        const double lower = std::min(motor.lower, motor.upper);
        const double upper = std::max(motor.lower, motor.upper);
        if (motor.distance < lower) {
            motor.distance = lower;
            motor.speed = 0;
        } else if (motor.distance > upper) {
            motor.distance = upper;
            motor.speed = 0;
        }
    }
    event.started = motor.running && !wasRunning;
    event.stopped = !motor.running && wasRunning;
    event.speedChanged = std::abs(motor.speed - previous) > 1e-6;
    if (motor.notifyEvery > 1e-6 && std::abs(motor.distance - motor.lastNotified) >= motor.notifyEvery) {
        event.distanceNotified = true;
        motor.lastNotified = motor.distance;
    }
    motor.wasRunning = motor.running;
    return event;
}

IkNumberResult solveBoundedIk(const IkResidual& residual, int equations, const std::vector<double>& seed,
                              const std::vector<double>& lower, const std::vector<double>& upper, IkSolverKind kind,
                              const IkSolverSettings& settings) {
    IkNumberResult result;
    result.solver = kind == IkSolverKind::SequentialQuadratic ? "sqp" : "quasi-newton";
    if (!residual || equations <= 0 || seed.empty()) {
        return result;
    }
    std::vector<double> q = projectBounds(seed, lower, upper);
    const int n = static_cast<int>(q.size());
    const double step = 1e-4;
    for (int iteration = 0; iteration < std::max(settings.iterations, 1); ++iteration) {
        std::vector<double> r(static_cast<std::size_t>(equations));
        residual(q, r);
        const double error = residualNorm(r);
        result.iterations = iteration + 1;
        result.residual = error;
        result.values = q;
        if (error <= settings.epsilon) {
            result.ok = true;
            return result;
        }
        std::vector<std::vector<double>> jacobian(static_cast<std::size_t>(equations), std::vector<double>(static_cast<std::size_t>(n)));
        for (int column = 0; column < n; ++column) {
            std::vector<double> shifted = q;
            shifted[static_cast<std::size_t>(column)] += step;
            shifted = projectBounds(shifted, lower, upper);
            std::vector<double> shiftedResidual(static_cast<std::size_t>(equations));
            residual(shifted, shiftedResidual);
            for (int row = 0; row < equations; ++row) {
                jacobian[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)] =
                    (shiftedResidual[static_cast<std::size_t>(row)] - r[static_cast<std::size_t>(row)]) / step;
            }
        }
        std::vector<double> normal(static_cast<std::size_t>(n), settings.damping);
        std::vector<double> rhs(static_cast<std::size_t>(n), 0);
        for (int row = 0; row < equations; ++row) {
            for (int column = 0; column < n; ++column) {
                rhs[static_cast<std::size_t>(column)] += jacobian[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)] *
                                                         r[static_cast<std::size_t>(row)];
                for (int other = 0; other < n; ++other) {
                    normal[static_cast<std::size_t>(column)] += jacobian[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)] *
                                                                jacobian[static_cast<std::size_t>(row)][static_cast<std::size_t>(other)] *
                                                                (column == other ? 1.0 : 0.0);
                }
            }
        }
        if (kind == IkSolverKind::SequentialQuadratic) {
            for (int column = 0; column < n; ++column) {
                const double lo = column < static_cast<int>(lower.size()) ? lower[static_cast<std::size_t>(column)] : -1e9;
                const double hi = column < static_cast<int>(upper.size()) ? upper[static_cast<std::size_t>(column)] : 1e9;
                const double violation = std::max(0.0, lo - q[static_cast<std::size_t>(column)]) +
                                         std::max(0.0, q[static_cast<std::size_t>(column)] - hi);
                rhs[static_cast<std::size_t>(column)] += settings.penalty * violation;
            }
        }
        std::vector<double> next = q;
        for (int column = 0; column < n; ++column) {
            const double diag = std::max(normal[static_cast<std::size_t>(column)], 1e-8);
            next[static_cast<std::size_t>(column)] -= rhs[static_cast<std::size_t>(column)] / diag;
        }
        q = projectBounds(next, lower, upper);
    }
    std::vector<double> r(static_cast<std::size_t>(equations));
    residual(q, r);
    result.values = q;
    result.residual = residualNorm(r);
    result.ok = result.residual <= settings.epsilon;
    return result;
}

nlohmann::json physicsToJson(const PhysicsDocument& document) {
    nlohmann::json grades = nlohmann::json::array();
    for (const SurfaceGrade& grade : document.grades) {
        grades.push_back({{"name", grade.name}, {"friction", grade.friction}, {"restitution", grade.restitution}});
    }
    nlohmann::json pairs = nlohmann::json::array();
    for (const GradePair& pair : document.pairs) {
        pairs.push_back({{"a", pair.a}, {"b", pair.b}, {"friction", pair.friction}, {"restitution", pair.restitution}});
    }
    return {{"enabled", document.enabled},
            {"timestep", document.timestep},
            {"engine", document.engine},
            {"defaultFriction", document.defaultFriction},
            {"defaultRestitution", document.defaultRestitution},
            {"referenceFriction", document.referenceFriction},
            {"grades", std::move(grades)},
            {"pairs", std::move(pairs)}};
}

void physicsFromJson(const nlohmann::json& json, PhysicsDocument& document) {
    if (!json.is_object()) {
        return;
    }
    document.enabled = json.value("enabled", document.enabled);
    document.timestep = json.value("timestep", document.timestep);
    document.engine = json.value("engine", document.engine);
    document.defaultFriction = json.value("defaultFriction", document.defaultFriction);
    document.defaultRestitution = json.value("defaultRestitution", document.defaultRestitution);
    document.referenceFriction = json.value("referenceFriction", document.referenceFriction);
    document.grades.clear();
    if (json.contains("grades") && json.at("grades").is_array()) {
        for (const auto& item : json.at("grades")) {
            SurfaceGrade grade;
            grade.name = item.value("name", std::string{});
            grade.friction = item.value("friction", grade.friction);
            grade.restitution = item.value("restitution", grade.restitution);
            document.grades.push_back(std::move(grade));
        }
    }
    document.pairs.clear();
    if (json.contains("pairs") && json.at("pairs").is_array()) {
        for (const auto& item : json.at("pairs")) {
            GradePair pair;
            pair.a = item.value("a", std::string{});
            pair.b = item.value("b", std::string{});
            pair.friction = item.value("friction", pair.friction);
            pair.restitution = item.value("restitution", pair.restitution);
            document.pairs.push_back(std::move(pair));
        }
    }
}

nlohmann::json motorToJson(const MotorAspect& motor) {
    return {{"name", motor.name},
            {"owner", motor.owner},
            {"targetSpeed", motor.targetSpeed},
            {"accel", motor.accel},
            {"decel", motor.decel},
            {"speed", motor.speed},
            {"direction", motor.direction},
            {"running", motor.running},
            {"distance", motor.distance},
            {"limited", motor.limited},
            {"lower", motor.lower},
            {"upper", motor.upper},
            {"locked", motor.locked},
            {"pulseDistance", motor.pulseDistance},
            {"highRatio", motor.highRatio},
            {"grayCode", motor.grayCode},
            {"minCount", motor.minCount},
            {"maxCount", motor.maxCount},
            {"countSignal", motor.countSignal},
            {"pulseSignal", motor.pulseSignal},
            {"runningSignal", motor.runningSignal},
            {"speedSignal", motor.speedSignal},
            {"graySignal", motor.graySignal},
            {"distanceSignal", motor.distanceSignal},
            {"startedSignal", motor.startedSignal},
            {"stoppedSignal", motor.stoppedSignal},
            {"changedSignal", motor.changedSignal},
            {"notifyEvery", motor.notifyEvery}};
}

void motorFromJson(const nlohmann::json& json, MotorAspect& motor) {
    if (!json.is_object()) {
        return;
    }
    motor.name = json.value("name", motor.name);
    motor.owner = json.value("owner", motor.owner);
    motor.targetSpeed = json.value("targetSpeed", motor.targetSpeed);
    motor.accel = json.value("accel", motor.accel);
    motor.decel = json.value("decel", motor.decel);
    motor.speed = json.value("speed", motor.speed);
    motor.direction = json.value("direction", motor.direction);
    motor.running = json.value("running", motor.running);
    motor.distance = json.value("distance", motor.distance);
    motor.limited = json.value("limited", motor.limited);
    motor.lower = json.value("lower", motor.lower);
    motor.upper = json.value("upper", motor.upper);
    motor.locked = json.value("locked", motor.locked);
    motor.pulseDistance = json.value("pulseDistance", motor.pulseDistance);
    motor.highRatio = json.value("highRatio", motor.highRatio);
    motor.grayCode = json.value("grayCode", motor.grayCode);
    motor.minCount = json.value("minCount", motor.minCount);
    motor.maxCount = json.value("maxCount", motor.maxCount);
    motor.countSignal = json.value("countSignal", motor.countSignal);
    motor.pulseSignal = json.value("pulseSignal", motor.pulseSignal);
    motor.runningSignal = json.value("runningSignal", motor.runningSignal);
    motor.speedSignal = json.value("speedSignal", motor.speedSignal);
    motor.graySignal = json.value("graySignal", motor.graySignal);
    motor.distanceSignal = json.value("distanceSignal", motor.distanceSignal);
    motor.startedSignal = json.value("startedSignal", motor.startedSignal);
    motor.stoppedSignal = json.value("stoppedSignal", motor.stoppedSignal);
    motor.changedSignal = json.value("changedSignal", motor.changedSignal);
    motor.notifyEvery = json.value("notifyEvery", motor.notifyEvery);
}

}  // namespace sim
