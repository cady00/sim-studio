#include "sim/scripting/ScriptHost.hpp"

namespace sim {

std::unique_ptr<IScriptHost> createScriptHost() { return std::make_unique<NullScriptHost>(); }

}  // namespace sim
