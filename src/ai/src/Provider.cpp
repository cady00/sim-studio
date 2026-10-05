#include "sim/ai/Provider.hpp"

namespace sim {
namespace {

class OfflineProvider final : public IAiProvider {
public:
    explicit OfflineProvider(ProviderConfig config) : config_(std::move(config)) {}

    const char* name() const override {
        return config_.kind == "openai-compatible" ? "openai-compatible" : "none";
    }
    bool enabled() const override { return config_.enabled && config_.kind != "none"; }
    bool usesNetwork() const override { return false; }
    std::vector<AiCapability> capabilities() const override { return {}; }

    nlohmann::json complete(const nlohmann::json&, std::string& error) override {
        if (!config_.enabled || config_.kind == "none") {
            error = "AI provider is off. Sim Studio does not contact a model unless a provider is enabled in a future build.";
        } else if (config_.endpoint.empty()) {
            error = "AI provider is enabled but has no endpoint. No request was sent.";
        } else {
            error = "OpenAI-compatible transport is not compiled. Endpoint '" + config_.endpoint +
                    "' was not contacted. A later build can POST /v1/chat/completions only after the user turns the transport on.";
        }
        return nlohmann::json::object();
    }

private:
    ProviderConfig config_;
};

}  // namespace

std::unique_ptr<IAiProvider> createAiProvider(const ProviderConfig& config) {
    return std::make_unique<OfflineProvider>(config);
}

}  // namespace sim
