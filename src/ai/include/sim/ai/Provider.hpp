#pragma once

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace sim {

// Future model-backed features. None of them run in this revision.
enum class AiCapability {
    NaturalLanguageToLogic,
    LayoutSuggestion,
    CycleTimeOptimization,
    AnomalyExplanation
};

struct ProviderConfig {
    // Default is off. A configured endpoint still does not open a socket
    // unless a future build turns on the transport.
    bool enabled = false;
    std::string kind = "none";  // "none" or "openai-compatible"
    std::string endpoint;
    std::string model;
};

// Local OpenAI-compatible servers (Ollama, llama.cpp) are the intended first
// provider. Cloud providers would be another kind. This binary never calls them.
class IAiProvider {
public:
    virtual ~IAiProvider() = default;
    virtual const char* name() const = 0;
    virtual bool enabled() const = 0;
    virtual bool usesNetwork() const = 0;
    virtual std::vector<AiCapability> capabilities() const = 0;
    virtual nlohmann::json complete(const nlohmann::json& request, std::string& error) = 0;
};

std::unique_ptr<IAiProvider> createAiProvider(const ProviderConfig& config = {});

}  // namespace sim
