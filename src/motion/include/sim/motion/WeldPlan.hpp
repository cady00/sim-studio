#pragma once

#include "sim/core/Weld.hpp"
#include "sim/motion/Motion.hpp"

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

}  // namespace sim
