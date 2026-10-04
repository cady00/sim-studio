#pragma once

#include "sim/core/Weld.hpp"
#include "sim/motion/Motion.hpp"

#include <functional>

namespace sim {

struct SpinSearch {
    bool ok = false;
    double spinDeg = 0;
    double margin = 0;
    double manipulability = 0;
    bool singular = false;
    bool collision = false;
    std::vector<double> joints;
    CartesianPose tcp;
    std::string note;
};

// TCP origin sits on the spot. Tool +Z points into the sheet, opposite `normal`.
// `spinDeg` twists the tool around that normal.
CartesianPose weldTcpPose(const WeldSpot& spot, double spinDeg);

// Samples spins, solves inverse kinematics, and keeps the pose with the largest
// joint margin. A singular wrist or a proximity hit lowers the score.
SpinSearch searchWeldSpin(Scene* scene, KinematicResource& resource, const WeldSpot& spot, int samples, bool checkCollision);

// Example spots on a plate in front of kr420. The private STEP is not required.
void prepareExampleWelds(Scene& scene);

struct WeldAdvice {
    // reachable | gun | base | track | open
    std::string kind;
    std::string text;
    Vec3 shift{0, 0, 0};
};

// For an unreachable spot, try a gun-normal change, a base shift, then an external-axis shift.
WeldAdvice suggestWeldFix(KinematicResource& resource, const WeldSpot& spot);

struct SpotReach {
    std::string id;
    bool reachable = false;
    double margin = 0;
    double manipulability = 0;
    bool singular = false;
    bool collision = false;
    std::vector<double> joints;
};

// Writes the red/green result back onto each spot. Collision is tested once, at the chosen spin.
std::vector<SpotReach> checkWeldReach(Scene* scene, KinematicResource& resource, std::vector<WeldSpot>& spots, bool checkCollision);

struct BaseSample {
    double x = 0;
    double y = 0;
    int covered = 0;
    bool coversAll = false;
};

struct BaseRegion {
    std::vector<BaseSample> samples;
    bool found = false;
    double minX = 0;
    double maxX = 0;
    double minY = 0;
    double maxY = 0;
    int bestCovered = 0;
};

// Samples base offsets in the world XY plane. A cell covers a spot when the shifted target stays reachable.
BaseRegion optimizeRobotBase(KinematicResource& resource, const std::vector<WeldSpot>& spots, double step, int cells);

struct RrtOptions {
    int iterations = 400;
    double stepDeg = 8;
    double goalBias = 0.15;
    unsigned seed = 1;
};

struct RrtResult {
    bool ok = false;
    std::vector<std::vector<double>> joints;
    int nodes = 0;
    std::string note;
};

// Joint-space RRT-Connect. `blocked` is true when that configuration must be avoided.
RrtResult planRrtConnect(const KinematicResource& resource, const std::vector<double>& start, const std::vector<double>& goal,
                         const std::function<bool(const std::vector<double>&)>& blocked, const RrtOptions& options);
std::vector<std::vector<double>> smoothJointPath(const std::vector<std::vector<double>>& path,
                                                 const std::function<bool(const std::vector<double>&)>& blocked, int passes);

// Open-tour nearest neighbor, then 2-opt. Returns indexes into `points`.
std::vector<int> orderWeldSpots(const std::vector<Vec3>& points);

// Orders reachable spots, plans a collision-free joint tour, and stores it as path "weld_tour".
int planExampleWeldTour(Scene& scene);
// Records a weld station and a task whose path is weld_tour when the project has none.
void attachWeldLine(Scene& scene);

}  // namespace sim
