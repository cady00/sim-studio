#include "sim/io/OcctEnvironment.hpp"

#include <cstdlib>
#include <filesystem>
#include <string>

namespace sim {
namespace {

void setEnvIfUnset(const char* key, const std::string& value) {
    const char* current = std::getenv(key);
    if (current != nullptr && current[0] != '\0') {
        return;
    }
#if defined(_WIN32)
    _putenv_s(key, value.c_str());
#else
    setenv(key, value.c_str(), 0);
#endif
}

}  // namespace

void ensureOcctEnvironment() {
#if defined(SIM_OCCT_RESOURCE_DIR)
    const std::filesystem::path root{SIM_OCCT_RESOURCE_DIR};
#else
    const std::filesystem::path root{"/usr/share/opencascade/resources"};
#endif
    if (!std::filesystem::exists(root)) {
        return;
    }
    const std::string standard = (root / "StdResource").string();
    const std::string step = (root / "XSTEPResource").string();
    const std::string xs = (root / "XSMessage").string();
    const std::string sh = (root / "SHMessage").string();
    setEnvIfUnset("CSF_StandardDefaults", standard);
    setEnvIfUnset("CSF_PluginDefaults", standard);
    setEnvIfUnset("CSF_XCAFDefaults", standard);
    setEnvIfUnset("CSF_STEPDefaults", step);
    setEnvIfUnset("CSF_IGESDefaults", step);
    setEnvIfUnset("CSF_XSMessage", xs);
    setEnvIfUnset("CSF_SHMessage", sh);
}

}  // namespace sim
