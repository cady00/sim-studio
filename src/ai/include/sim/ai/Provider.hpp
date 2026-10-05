#pragma once

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace sim {

// The four hooks run offline rules by default. A localhost model is optional.
enum class AiCapability {
    NaturalLanguageToLogic,
    LayoutSuggestion,
    CycleTimeOptimization,
    AnomalyExplanation
};

struct ProviderConfig {
    // Default is off. An enabled endpoint may open a socket only to localhost.
    bool enabled = false;
    std::string kind = "none";  // "none" or "openai-compatible"
    std::string endpoint;
    std::string model;
};

// Local OpenAI-compatible servers (llama.cpp) can answer on localhost.
// Any other host is rejected before a socket is opened.
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
