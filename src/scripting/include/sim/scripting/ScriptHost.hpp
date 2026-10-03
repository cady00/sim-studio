#pragma once

#include <memory>
#include <string>

namespace sim {

// Phase 2 embeds CPython through pybind11, in the style of MELSOFT Gemini / Visual Components.
class IScriptHost {
public:
    virtual ~IScriptHost() = default;
    virtual const char* backendName() const = 0;
    virtual bool available() const = 0;
    virtual bool execute(const std::string& code, std::string& output, std::string& error) = 0;
};

class NullScriptHost final : public IScriptHost {
public:
    const char* backendName() const override { return "null"; }
    bool available() const override { return false; }
    bool execute(const std::string&, std::string&, std::string& error) override {
        error = "Python scripting is not linked in Phase 1";
        return false;
    }
};

std::unique_ptr<IScriptHost> createScriptHost();

}  // namespace sim
