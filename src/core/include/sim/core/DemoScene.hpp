#pragma once

#include "sim/core/Scene.hpp"

namespace sim {

// Small conveyor line used as the Phase 2 sample. Scripts are stored on the scene.
void buildConveyorLine(Scene& scene);
// Two belts, a handoff volume, and an encoder motor. Surface ride is the builtin solver.
void buildTransferLine(Scene& scene);
// A mesh plate with a gap in its bounding box. The builtin solver uses triangles, not the box.
void buildMeshGapDemo(Scene& scene);
// Two free boxes on one belt. Coulomb friction, not the surface-ride shortcut.
void buildSlipDemo(Scene& scene);
// Gear, cam, cylinder, gripper, sensors, and a source/sink. Logic writes the signals.
void buildDeviceDemo(Scene& scene);
// Four-station example line. layout.example and ppr.example are true. The KR420 is a named proxy
// unless a caller replaces the node with an imported model.
void buildExampleLine(Scene& scene);
// One shared mesh and `parts` instances. `triangles` is the mesh triangle count.
void buildLargeField(Scene& scene, int parts, int triangles);

const char* pusherScriptSource();
const char* sequenceScriptSource();
const char* counterScriptSource();

}  // namespace sim
