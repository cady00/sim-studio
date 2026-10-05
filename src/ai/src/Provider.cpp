#include "sim/ai/Provider.hpp"

#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>

namespace sim {
namespace {

bool parseLocalHttp(const std::string& endpoint, int& port, std::string& path, std::string& error) {
    const std::string prefix = "http://";
    if (endpoint.rfind(prefix, 0) != 0) {
        error = "AI provider only connects to localhost over http";
        return false;
    }
    const std::string rest = endpoint.substr(prefix.size());
    const auto slash = rest.find('/');
    const std::string authority = slash == std::string::npos ? rest : rest.substr(0, slash);
    path = slash == std::string::npos ? "/v1/chat/completions" : rest.substr(slash);
    const auto colon = authority.rfind(':');
    const std::string host = colon == std::string::npos ? authority : authority.substr(0, colon);
    if (host != "127.0.0.1" && host != "localhost") {
        error = "AI provider only connects to localhost";
        return false;
    }
    port = 80;
    if (colon != std::string::npos) {
        try {
            port = std::stoi(authority.substr(colon + 1));
        } catch (const std::exception&) {
            error = "AI provider port is not a number";
            return false;
        }
    }
    if (port <= 0 || port > 65535) {
        error = "AI provider port is out of range";
        return false;
    }
    error.clear();
    return true;
}

std::string postLocalhost(int port, const std::string& path, const std::string& body, std::string& error) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        error = "localhost socket was not opened";
        return {};
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1) {
        ::close(fd);
        error = "localhost address was not parsed";
        return {};
    }
    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    const int connected = ::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    if (connected != 0 && errno != EINPROGRESS) {
        ::close(fd);
        error = "localhost model was not reached";
        return {};
    }
    pollfd wait{};
    wait.fd = fd;
    wait.events = POLLOUT;
    if (poll(&wait, 1, 200) <= 0 || (wait.revents & (POLLERR | POLLHUP)) != 0) {
        ::close(fd);
        error = "localhost model was not reached";
        return {};
    }
    int soError = 0;
    socklen_t length = sizeof(soError);
    getsockopt(fd, SOL_SOCKET, SO_ERROR, &soError, &length);
    if (soError != 0) {
        ::close(fd);
        error = "localhost model was not reached";
        return {};
    }
    fcntl(fd, F_SETFL, flags);
    std::ostringstream request;
    request << "POST " << path << " HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: "
            << body.size() << "\r\n\r\n"
            << body;
    const std::string bytes = request.str();
    std::size_t sent = 0;
    while (sent < bytes.size()) {
        const ssize_t wrote = ::send(fd, bytes.data() + sent, bytes.size() - sent, 0);
        if (wrote <= 0) {
            ::close(fd);
            error = "localhost model was not reached";
            return {};
        }
        sent += static_cast<std::size_t>(wrote);
    }
    std::string response;
    char buffer[4096];
    for (;;) {
        pollfd incoming{};
        incoming.fd = fd;
        incoming.events = POLLIN;
        if (poll(&incoming, 1, 200) <= 0) {
            break;
        }
        const ssize_t got = ::recv(fd, buffer, sizeof(buffer), 0);
        if (got <= 0) {
            break;
        }
        response.append(buffer, buffer + got);
        if (response.size() > 65536) {
            break;
        }
    }
    ::close(fd);
    const auto split = response.find("\r\n\r\n");
    if (split == std::string::npos) {
        error = "localhost model returned no body";
        return {};
    }
    error.clear();
    return response.substr(split + 4);
}

class OfflineProvider final : public IAiProvider {
public:
    explicit OfflineProvider(ProviderConfig config) : config_(std::move(config)) {}

    const char* name() const override {
        return config_.kind == "openai-compatible" ? "openai-compatible" : "none";
    }
    bool enabled() const override { return config_.enabled && config_.kind != "none"; }
    bool usesNetwork() const override { return false; }
    std::vector<AiCapability> capabilities() const override {
        if (!enabled()) {
            return {};
        }
        return {AiCapability::NaturalLanguageToLogic, AiCapability::LayoutSuggestion, AiCapability::CycleTimeOptimization,
                AiCapability::AnomalyExplanation};
    }

    nlohmann::json complete(const nlohmann::json& request, std::string& error) override {
        if (!config_.enabled || config_.kind == "none") {
            error = "AI provider is off. Sim Studio does not contact a model unless a provider is enabled.";
            return nlohmann::json::object();
        }
        if (config_.endpoint.empty()) {
            error = "AI provider is enabled but has no endpoint. No request was sent.";
            return nlohmann::json::object();
        }
        int port = 0;
        std::string path;
        if (!parseLocalHttp(config_.endpoint, port, path, error)) {
            return nlohmann::json::object();
        }
        nlohmann::json body = {{"model", config_.model.empty() ? "local" : config_.model}, {"messages", request.value("messages", nlohmann::json::array())}};
        const std::string payload = postLocalhost(port, path, body.dump(), error);
        if (!error.empty()) {
            return nlohmann::json::object();
        }
        const auto parsed = nlohmann::json::parse(payload, nullptr, false);
        if (parsed.is_discarded()) {
            error = "localhost model returned unreadable JSON";
            return nlohmann::json::object();
        }
        if (parsed.contains("choices") && parsed.at("choices").is_array() && !parsed.at("choices").empty()) {
            const auto& message = parsed.at("choices").at(0).value("message", nlohmann::json::object());
            const std::string content = message.value("content", "");
            const auto inner = nlohmann::json::parse(content, nullptr, false);
            if (inner.is_object()) {
                error.clear();
                return inner;
            }
        }
        error.clear();
        return parsed;
    }

private:
    ProviderConfig config_;
};

}  // namespace

std::unique_ptr<IAiProvider> createAiProvider(const ProviderConfig& config) {
    return std::make_unique<OfflineProvider>(config);
}

}  // namespace sim
