#pragma once

#include "sim/core/Transform.hpp"

#include <nlohmann/json_fwd.hpp>

#include <string>
#include <utility>
#include <vector>

namespace sim {

// Versioned kinematic resource. A turntable, lift, roller, or robot is this
// data plus ordinary scene nodes. Nothing here is a prebuilt machine.

enum class JointMotion { Revolute, Prismatic };

struct KinLink {
    std::string name;
    std::string nodeId;
    // Rigid mount onto another link, with no joint of its own. The robot base
    // sits on a carriage this way: world = parent * mountRest.
    std::string mountedOn;
    Transform mountRest{};
    // URDF inertial, kilograms and kilograms times square meters. Geometry stays millimeters.
    bool hasInertia = false;
    double mass = 0;
    Vec3 com{0, 0, 0};
    double ixx = 0;
    double iyy = 0;
    double izz = 0;
    double ixy = 0;
    double ixz = 0;
    double iyz = 0;
};

struct KinJoint {
    std::string name;
    std::string parentLink;
    std::string childLink;
    JointMotion motion = JointMotion::Revolute;
    Vec3 anchor{0, 0, 0};
    Vec3 axis{0, 0, 1};
    bool limited = true;
    double lower = -180;
    double upper = 180;
    double maxSpeed = 90;
    double maxAccel = 360;
    // Follower position is master * coupleRatio. Empty means independent.
    std::string coupledTo;
    double coupleRatio = 1;
    double position = 0;
    Transform restLocal{};
    // Empty, or "E1".."E6" once the joint is coupled to the robot controller.
    // externalKind is "" | "track" | "positioner". A track moves the robot base.
    // A positioner moves the workpiece and does not move the robot base.
    std::string externalName;
    std::string externalKind;
};

struct KinPose {
    std::string name;
    std::vector<std::pair<std::string, double>> joints;
};

struct NamedFrame {
    std::string name;
    Vec3 origin{0, 0, 0};
    Quat rotation = Quat::identity();
    // Empty means the chain tip (the usual TCP). A workpiece frame names the platter link.
    std::string link;
};

// How a redundant external axis is chosen during inverse kinematics.
// "fixed" holds `fixed` (or the seed, where `fixed` is short). The other modes
// sample the external axes and score travel, mid-range, and manipulability together.
// Weights are relative. A third external axis stays at the seed during the search.
struct RedundancyPreference {
    std::string mode = "fixed";
    double travel = 1;
    double midrange = 0.2;
    double manipulability = 0.2;
    std::vector<double> fixed;
};

// Generic 6-axis arm, millimeters and a spherical wrist. Joint zero is the arm
// stretched along +X, tool approach along +Z of the flange.
struct SphericalWrist {
    double d1 = 300;
    // Horizontal distance from A1 to the A2 axis. Zero keeps the older intersecting-axis model.
    double shoulder = 0;
    double a2 = 400;
    double a3 = 300;
    double d6 = 80;
    Vec3 tcp{0, 0, 0};
    Quat tcpRotation = Quat::identity();
};

// SCARA: revolute, revolute, prismatic along Z, revolute. Lengths in millimeters.
struct ScaraArm {
    double l1 = 350;
    double l2 = 250;
    double baseZ = 0;
    Vec3 tcp{0, 0, 0};
};

struct KinematicResource {
    std::string name;
    // none | scara | spherical_wrist | numerical
    std::string ik = "none";
    SphericalWrist wrist;
    ScaraArm scara;
    std::vector<KinLink> links;
    std::vector<KinJoint> joints;
    std::vector<KinPose> poses;
    std::vector<NamedFrame> frames;
    RedundancyPreference redundancy;
};

enum class PathMove { Ptp, Lin, Circ };

struct PathPoint {
    std::string name;
    PathMove move = PathMove::Ptp;
    bool cartesian = false;
    std::vector<double> joints;
    Vec3 position{0, 0, 0};
    Quat orientation = Quat::identity();
    Vec3 via{0, 0, 0};
    double speed = 100;
    double zone = 0;
    // Empty inherits the path. Otherwise "locked", "sync", or "positioner".
    std::string external;
    // Parallel to the resource's external joints, in joint-list order.
    std::vector<double> externalValues;
};

struct RobotPath {
    std::string name;
    std::string resource;
    std::vector<PathPoint> points;
    // "locked" holds external axes. "sync" moves track axes with LIN/CIRC.
    // "positioner" interprets the point in the workpiece frame.
    std::string external = "locked";
    std::string workpiece;
};

struct LogicCondition {
    std::string signal;
    // == != > < >= <=
    std::string op = "==";
    bool asBool = true;
    bool flag = false;
    double number = 0;
};

struct LogicAction {
    // move_pose | jog | set_output | reset_output | wait | wait_signal
    std::string kind = "wait";
    std::string resource;
    std::string pose;
    std::string joint;
    std::string signal;
    // trapezoid | scurve
    std::string profile = "trapezoid";
    double speed = 0;
    double seconds = 0;
};

struct LogicParam {
    std::string name;
    std::string type = "real";
    bool flag = false;
    std::int64_t integer = 0;
    double real = 0;
};

struct LogicStep {
    std::vector<LogicCondition> when;
    std::vector<LogicAction> actions;
};

// A block never reads another block. Inputs, outputs, and conditions are signal names.
struct LogicBlock {
    std::string name;
    std::string resource;
    std::string runSignal;
    std::vector<std::string> inputs;
    std::vector<std::string> outputs;
    std::vector<LogicParam> parameters;
    std::vector<LogicStep> steps;
    // Stored with the block. The runner does not execute it; the script dock does,
    // and that script still only sees the signal table.
    std::string python;
};

const char* jointMotionName(JointMotion motion);
bool jointMotionFromName(const std::string& name, JointMotion& out);
const char* pathMoveName(PathMove move);
bool pathMoveFromName(const std::string& name, PathMove& out);

nlohmann::json kinematicsToJson(const KinematicResource& resource);
bool kinematicsFromJson(const nlohmann::json& json, KinematicResource& resource, std::string& error);
nlohmann::json logicBlockToJson(const LogicBlock& block);
bool logicBlockFromJson(const nlohmann::json& json, LogicBlock& block, std::string& error);
nlohmann::json robotPathToJson(const RobotPath& path);
bool robotPathFromJson(const nlohmann::json& json, RobotPath& path, std::string& error);

KinematicResource* findResource(std::vector<KinematicResource>& resources, const std::string& name);
const KinematicResource* findResource(const std::vector<KinematicResource>& resources, const std::string& name);
KinJoint* findJoint(KinematicResource& resource, const std::string& name);
const KinJoint* findJoint(const KinematicResource& resource, const std::string& name);
KinLink* findLink(KinematicResource& resource, const std::string& name);
const KinPose* findPose(const KinematicResource& resource, const std::string& name);

double clampJoint(const KinJoint& joint, double position);

inline bool hasExternalAxis(const KinematicResource& resource) {
    for (const KinJoint& joint : resource.joints) {
        if (!joint.externalName.empty()) {
            return true;
        }
    }
    return false;
}

}  // namespace sim
