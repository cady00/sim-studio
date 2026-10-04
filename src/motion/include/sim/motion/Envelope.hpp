#pragma once

#include "sim/core/Envelope.hpp"
#include "sim/core/Scene.hpp"
#include "sim/motion/Motion.hpp"

#include <string>
#include <vector>

namespace sim {

// Capsules along the TCP samples, plus a collision set named after the envelope.
// Existing collision sets stay. A pair is added against a set named "fixture" when that set exists.
MotionEnvelope buildTcpEnvelope(Scene& scene, KinematicResource& resource, const std::vector<std::vector<double>>& joints,
                                const std::string& name, double radiusMm);

// Swept convex hull of each link's mesh bounds along the joint samples.
// Analytic arms without link meshes use the four DH segments of the spherical wrist.
MotionEnvelope buildLinkSweep(Scene& scene, KinematicResource& resource, const std::vector<std::vector<double>>& joints,
                              const std::string& name);

// Robots whose TCP lies in the same zone produce one hit per pair. The signal
// interlock_<zone> is raised while a pair is inside.
std::vector<InterlockHit> evaluateInterlocks(Scene& scene);

// When a pair is inside, add the zone's release time to the waiting station's operation.
// Returns the seconds added. A later call does not add them again.
double applyInterlockHold(Scene& scene);

}  // namespace sim
