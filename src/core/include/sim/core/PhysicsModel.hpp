#pragma once

#include "sim/core/Mech.hpp"

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace sim {

struct SurfaceGrade {
    std::string name;
    double friction = 0.4;
    double restitution = 0.05;
};

struct GradePair {
    std::string a;
    std::string b;
    double friction = 0.4;
    double restitution = 0.05;
};

// Reusable drive. Distance is millimeters. Speeds are mm/s or deg/s, matching the owner.
struct MotorAspect {
    std::string name;
    std::string owner;
    double targetSpeed = 0;
    double accel = 500;
    double decel = 800;
    double speed = 0;
    int direction = 1;
    bool running = false;
    double distance = 0;
    bool limited = false;
    double lower = 0;
    double upper = 0;
    bool locked = false;
    double pulseDistance = 10;
    double highRatio = 0.5;
    bool grayCode = false;
    std::int64_t minCount = 0;
    std::int64_t maxCount = 1000000;
    std::string countSignal;
    std::string pulseSignal;
    std::string runningSignal;
    std::string speedSignal;
    std::string graySignal;
    std::string distanceSignal;
    std::string startedSignal;
    std::string stoppedSignal;
    std::string changedSignal;
    double notifyEvery = 0;
    bool wasRunning = false;
    double lastNotified = 0;
};

struct PhysicsStats {
    int contacts = 0;
    int continuous = 0;
    double detectMs = 0;
    double solveMs = 0;
};

struct PhysicsDocument {
    bool enabled = true;
    double timestep = 1.0 / 60.0;
    // Stored preference. The running backend can still be overridden at launch.
    std::string engine = "jolt";
    double defaultFriction = 0.4;
    double defaultRestitution = 0.05;
    // Pair friction at or above this value commands the full conveyor speed.
    double referenceFriction = 0.5;
    std::vector<SurfaceGrade> grades;
    std::vector<GradePair> pairs;
    PhysicsStats stats;
};

struct EncoderSample {
    std::int64_t count = 0;
    bool pulseHigh = false;
    std::int64_t gray = 0;
};

struct MotorEvent {
    bool started = false;
    bool stopped = false;
    bool speedChanged = false;
    bool distanceNotified = false;
};

enum class IkSolverKind { QuasiNewton, SequentialQuadratic };

struct IkSolverSettings {
    double epsilon = 1e-4;
    int iterations = 40;
    double penalty = 8;
    double damping = 1e-3;
};

struct IkNumberResult {
    bool ok = false;
    std::vector<double> values;
    int iterations = 0;
    double residual = 0;
    std::string solver;
};

double stoppingDistance(double speed, double decel);
double stoppingTime(double speed, double decel);
double timeToSpeed(double from, double to, double rate);

double pairFriction(const PhysicsDocument& document, const std::string& a, const std::string& b);
double pairRestitution(const PhysicsDocument& document, const std::string& a, const std::string& b);
double conveyorScale(const PhysicsDocument& document, const std::string& load, const std::string& belt, double grip);

double massKilograms(const Mech& mech);
void syncMotionFlags(Mech& mech);
EncoderSample sampleEncoder(const MotorAspect& motor);
MotorEvent advanceMotor(MotorAspect& motor, double dt);

using IkResidual = std::function<void(const std::vector<double>& values, std::vector<double>& residual)>;

IkNumberResult solveBoundedIk(const IkResidual& residual, int equations, const std::vector<double>& seed,
                              const std::vector<double>& lower, const std::vector<double>& upper, IkSolverKind kind,
                              const IkSolverSettings& settings);

nlohmann::json physicsToJson(const PhysicsDocument& document);
void physicsFromJson(const nlohmann::json& json, PhysicsDocument& document);
nlohmann::json motorToJson(const MotorAspect& motor);
void motorFromJson(const nlohmann::json& json, MotorAspect& motor);

}  // namespace sim
