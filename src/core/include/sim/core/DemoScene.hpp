#pragma once

#include "sim/core/Scene.hpp"

namespace sim {

// Small conveyor line used as the Phase 2 sample. Scripts are stored on the scene.
void buildConveyorLine(Scene& scene);
// Two belts, a handoff volume, and an encoder motor. Surface ride is the builtin solver.
void buildTransferLine(Scene& scene);

const char* pusherScriptSource();
const char* sequenceScriptSource();
const char* counterScriptSource();

}  // namespace sim
