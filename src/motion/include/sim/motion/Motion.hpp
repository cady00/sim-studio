#pragma once

#include "sim/core/Kinematics.hpp"
#include "sim/core/Scene.hpp"
#include "sim/physics/PhysicsEngine.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace sim {

enum class ProfileKind { Trapezoid, SCurve };

struct CartesianPose {
    Vec3 position{0, 0, 0};
    Quat rotation = Quat::identity();
};

struct IkResult {
    bool ok = false;
    bool approximated = false;
    std::string warning;
    std::vector<double> joints;
};

struct ActiveMove {
    bool active = false;
    ProfileKind profile = ProfileKind::Trapezoid;
    double time = 0;
    double duration = 0;
    std::vector<int> index;
    std::vector<double> q0;
    std::vector<double> q1;
    std::vector<double> vmax;
    std::vector<double> amax;
    std::vector<double> jointDuration;
};

struct MotionPlan {
    bool ok = false;
    double seconds = 0;
    double dt = 1.0 / 60.0;
    std::vector<std::vector<double>> samples;
    std::vector<std::string> warnings;
    std::vector<std::pair<std::string, std::string>> collisions;
    // First contact of the continuous sweep. `sweepTime` is seconds along the samples.
    bool sweepHit = false;
    double sweepTime = 0;
    std::string sweepA;
    std::string sweepB;
};

struct PathSweepHit {
    bool hit = false;
    double time = 0;
    std::string a;
    std::string b;
    Vec3 point{0, 0, 0};
};

double profileDuration(ProfileKind kind, double q0, double q1, double vmax, double amax);
double profileSample(ProfileKind kind, double q0, double q1, double vmax, double amax, double time, bool* done);

double jointCommand(const KinematicResource& resource, const KinJoint& joint);
CartesianPose forwardModel(const KinematicResource& resource);
IkResult inverseModel(const KinematicResource& resource, const CartesianPose& target, const std::vector<double>& seed);
bool linkWorld(const KinematicResource& resource, const std::string& linkName, Mat4& world);
bool namedFrameWorld(const KinematicResource& resource, const std::string& frameName, CartesianPose& pose);
CartesianPose poseFromMatrix(const Mat4& matrix);
Mat4 matrixFromPose(const CartesianPose& pose);

// Names stored for a later OLP post. This does not write a robot program.
struct OlpAxisName {
    std::string joint;
    std::string e;
    std::string rapid;
    std::string fanuc;
};

std::vector<OlpAxisName> externalExportMap(const KinematicResource& resource);
std::vector<int> externalJointIndices(const KinematicResource& resource);

struct ReachReport {
    int samples = 0;
    int reachable = 0;
    double lower = 0;
    double upper = 0;
    std::string joint;
};

// Locks the first track joint across its range and counts inverse-kinematics successes.
ReachReport analyzeReach(const KinematicResource& resource, const CartesianPose& target, int samples);
// Copies a user-built carrier onto the robot, names the joints E1..E6, and mounts the robot root.
bool attachExternalAxes(Scene& scene, KinematicResource& robot, const std::string& carrierName,
                        const std::string& mountLink, const std::string& kind, std::string& error);

void applyResource(Scene& scene, const KinematicResource& resource);
std::vector<double> jointVector(const KinematicResource& resource);
void setJointVector(KinematicResource& resource, const std::vector<double>& values);

KinematicResource& ensureResource(Scene& scene, const std::string& name);
KinLink& addLink(Scene& scene, KinematicResource& resource, const std::string& name, const std::string& parentLink,
                 const Transform& local);
bool adoptParts(Scene& scene, KinematicResource& resource, const std::string& linkName,
                const std::vector<std::string>& partIds, std::string& error);
KinJoint& addJoint(KinematicResource& resource, Scene& scene, const std::string& name, const std::string& parentLink,
                   const std::string& childLink, JointMotion motion, Vec3 axis, Vec3 anchor);
void setAxisFromPoints(KinJoint& joint, Vec3 start, Vec3 end);
void capturePose(KinematicResource& resource, const std::string& poseName);
bool jogJoint(KinematicResource& resource, const std::string& jointName, double delta, std::string& error);

bool beginJointMove(const KinematicResource& resource, const std::vector<double>& target, ProfileKind profile,
                    ActiveMove& move, std::string& error);
bool beginPoseMove(const KinematicResource& resource, const std::string& poseName, ProfileKind profile, ActiveMove& move,
                   std::string& error);
bool advanceMove(KinematicResource& resource, ActiveMove& move, double dt);

class LogicMachine {
public:
    void reset();
    bool start(Scene& scene, const std::string& blockName, std::string& error);
    void step(Scene& scene, double dt);
    bool finished(const std::string& blockName) const;
    const std::string& lastError() const { return lastError_; }

private:
    struct Run {
        bool active = false;
        bool finished = false;
        bool sawLow = true;
        std::size_t step = 0;
        std::size_t action = 0;
        double waitLeft = 0;
        bool waiting = false;
        bool jogging = false;
        std::string jogJoint;
        double jogSpeed = 0;
        double jogLeft = 0;
        ActiveMove move;
    };
    std::vector<std::pair<std::string, Run>> runs_;
    std::string lastError_;
};

MotionPlan planRobotPath(Scene& scene, KinematicResource& resource, const RobotPath& path);
std::vector<std::pair<std::string, std::string>> collidingPairs(const Scene& scene, const KinematicResource& resource);
void highlightCollisions(Scene& scene, const KinematicResource& resource,
                         const std::vector<std::pair<std::string, std::string>>& pairs);
// Continuous check along joint-space samples. The bound is the summed joint travel of every point.
PathSweepHit sweepPlannedSamples(Scene& scene, KinematicResource& resource, const std::vector<std::vector<double>>& samples);

bool attachTool(Scene& scene, KinematicResource& resource, const std::string& nodeId, std::string& error);
bool detachTool(Scene& scene, const std::string& nodeId, std::string& error);

bool saveLibrary(const Scene& scene, const std::string& resourceName, const std::string& path, std::string& error);
bool loadLibrary(Scene& scene, const std::string& path, std::string& error);

void buildTurntableLiftDemo(Scene& scene);
void buildScaraDemo(Scene& scene);
void buildCollisionDemo(Scene& scene);

struct RevoluteLimit {
    double lower = -180;
    double upper = 180;
    double speed = 90;
};

struct LinkSuggestion {
    std::string name;
    std::vector<std::string> partIds;
    bool hasJoint = false;
    Vec3 jointAxis{0, 0, 1};
    Vec3 jointOrigin;
    int axisCount = 0;
    double axisRadius = 0;
};

struct SuggestResult {
    std::vector<LinkSuggestion> links;
    std::string note;
    SphericalWrist fitted;
    // Distance between the A1 axis and the A2 axis. The spherical model can carry it as `shoulder`.
    double a1ToA2Mm = 0;
};

// Groups a flat solid list by large coaxial cylinders. A weak axis parallel to a stronger
// one and closer than 220 mm is merged away. A1 is the strongest vertical axis; later axes
// follow nearest anchors outward. Touching boxes stay one component.
SuggestResult suggestLinkGroups(const Scene& scene);
bool axisFromCylindricalFace(const Scene& scene, const std::string& nodeId, Vec3 point, Vec3 normal, Vec3& origin,
                             Vec3& direction, double& radius);
// `limits` apply in chain order. Acceleration is not taken from a datasheet; it is set to 180 deg/s^2.
bool applyLinkSuggestions(Scene& scene, const std::string& resourceName, const SuggestResult& suggestion,
                          const std::vector<RevoluteLimit>& limits, std::string& error);
// Public KR 420 R3080 ranges and speeds. See docs/KR420.md. Six entries, A1 through A6.
std::vector<RevoluteLimit> kr420DatasheetLimits();
// Hides meshed parts whose names are outside the proximity sets. The caller shows them again.
std::vector<std::shared_ptr<SceneNode>> hidePartsOutsideProximity(Scene& scene);
// Obstacle, tint is already applied by applyLinkSuggestions. Adds a PTP/LIN path named "cycle".
void addKr420Demonstration(Scene& scene);
// Hides every meshed part. Used when a path has no collision sets, so planning does not voxelize them.
std::vector<std::shared_ptr<SceneNode>> hideAllMeshedParts(Scene& scene);
// Rail built with addLink/addJoint, then attached. Adds a sync LIN named "long". Speed and accel are placeholders.
bool addKr420TrackDemonstration(Scene& scene, std::string& error);

// In-house planner. IRcsAdapter is the unimplemented vendor seam.
class IRcsAdapter {
public:
    virtual ~IRcsAdapter() = default;
    virtual const char* name() const = 0;
    virtual bool interpolate(const KinematicResource& resource, double dt, std::vector<double>& joints,
                             std::string& error) = 0;
};

class BuiltinRobotSolver : public IRobotSolver {
public:
    const char* backendName() const override { return "builtin"; }
    bool build(Scene& scene, std::string& error) override;
    void step(double dtSeconds) override;
    void shutdown() override;

    bool arm(const std::string& pathName, std::string& error);
    const MotionPlan& plan() const { return plan_; }
    bool playing() const { return playing_; }
    void setHideAllDuringQuery(bool hide) { hideAllDuringQuery_ = hide; }

private:
    Scene* scene_ = nullptr;
    MotionPlan plan_;
    std::string resourceName_;
    double cursor_ = 0;
    bool playing_ = false;
    bool hideAllDuringQuery_ = false;
};

std::unique_ptr<IRobotSolver> createRobotSolver();

}  // namespace sim
