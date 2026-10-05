#pragma once

#include "sim/core/Scene.hpp"
#include "sim/io/AssetImport.hpp"

#include <string>
#include <utility>
#include <vector>

namespace sim {

// Process Simulate .cojt sidecars. The parsers are self-contained: kin_graph.xml is a
// Lassalle AddFlow document, motionparameters.e is a semicolon statement list, and
// $machine.dat is KRL. Nothing here links a Siemens or KUKA library.

struct CojtGraphLink {
    std::string name;
    std::string guid;
};

struct CojtGraphEdge {
    std::string name;
    int from = -1;
    int to = -1;
    std::string tooltip;
    bool prismatic = false;
    bool hasAxis = false;
    Vec3 axis{0, 0, 1};
    bool hasLimits = false;
    double lower = 0;
    double upper = 0;
    bool hasSpeed = false;
    double speed = 0;
    std::string coupledTo;
    double coupleRatio = 1;
};

struct CojtGraph {
    std::vector<CojtGraphLink> links;
    std::vector<CojtGraphEdge> edges;
};

struct MotionAxisRate {
    std::string name;
    bool hasSpeed = false;
    // joint_max_speed below 20 is radians/second in the FANUC files (1.7453293 = 100 deg/s).
    double speedDeg = 0;
    bool hasAccTime = false;
    double accTime = 0;
    double decTime = 0;
    double settle = 0;
    double delay = 0;
};

struct MotionZone {
    std::string name;
    std::string jointType;
    std::vector<double> jointParam;
    std::string cartType;
    bool hasCart = false;
    double cartParam = 0;
};

struct MotionParameters {
    std::vector<MotionAxisRate> joints;
    bool hasCartSpeed = false;
    double cartMaxLinSpeed = 0;
    bool hasCartAcc = false;
    double cartMaxLinAcc = 0;
    std::vector<MotionZone> zones;
    std::string configFamily;
};

struct MachineAxis {
    int index = 0;
    int type = 0;
    bool used = false;
    bool rotational = true;
    bool endless = false;
    double lower = 0;
    double upper = 0;
    double gearNum = 0;
    double gearDen = 1;
    double motorRpm = 0;
    // Rotational: deg/s = |rpm| * 6 / |gear|. Linear axes are not converted.
    double speed = 0;
    bool hasSpeed = false;
    double jerk = 0;
    int direction = 0;
    // Parallel couplings. follower is this axis, master is 1-based, ratio is N/D.
    std::vector<std::pair<int, double>> couples;
};

struct MachineFrame {
    double x = 0;
    double y = 0;
    double z = 0;
    double a = 0;
    double b = 0;
    double c = 0;
    bool present = false;
};

struct MachineData {
    bool ok = false;
    std::string trafoName;
    std::string kinClass;
    MachineFrame tiroro;
    MachineFrame tx3p3;
    MachineFrame tflwp;
    bool hasLengthA = false;
    bool hasLengthB = false;
    double lengthA = 0;
    double lengthB = 0;
    bool hasDh4 = false;
    bool hasDh5 = false;
    double dh4Alpha = 0;
    double dh5Alpha = 0;
    int activeMask = -1;
    std::vector<MachineAxis> axes;
};

struct InfoAxisLimit {
    int index = 0;
    double lower = 0;
    double upper = 0;
};

bool parseKinGraphXml(const std::string& text, CojtGraph& graph, std::string& error);
bool parseMotionParameters(const std::string& text, MotionParameters& out, std::string& error);
bool parseMachineDat(const std::string& text, MachineData& out, std::string& error);
bool parseComponentInfo(const std::string& text, std::vector<InfoAxisLimit>& axes, std::string& error);

// `path` may be a .cojt directory, the .jt inside it, or kin_graph.xml.
// A .jt import also calls this when its parent folder name ends in .cojt.
AssetImportResult importCojt(const std::string& path, Scene& scene);
void attachCojtFromJt(const std::string& jtPath, Scene& scene, std::string& note);

}  // namespace sim
