#pragma once

namespace sim {

// Points OCCT at its resource files when the process has no CSF_* variables.
// No-op if the variables are already set. Does not access the network.
void ensureOcctEnvironment();

}  // namespace sim
