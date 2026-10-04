#pragma once

#include "sim/core/Scene.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

// Binary OPC UA over TCP, SecurityPolicy None only.
// The message layout follows the published Part 6 chunk headers and the numeric
// encoding node ids. No third-party stack is linked.
struct OpcUaReference {
    std::uint32_t nodeId = 0;
    std::string browseName;
};

class OpcUaServer {
public:
    OpcUaServer();
    ~OpcUaServer();
    OpcUaServer(const OpcUaServer&) = delete;
    OpcUaServer& operator=(const OpcUaServer&) = delete;

    bool listen(int port, std::string& error);
    int port() const { return port_; }
    // Accepts one client and answers until that socket closes.
    bool serve(std::string& error);
    void close();

private:
    int listenFd_ = -1;
    int port_ = 0;
    bool partSeen_ = false;
    std::uint32_t monitoredHandle_ = 7;
};

class OpcUaClient {
public:
    OpcUaClient();
    ~OpcUaClient();
    OpcUaClient(const OpcUaClient&) = delete;
    OpcUaClient& operator=(const OpcUaClient&) = delete;

    bool connect(const std::string& host, int port, std::string& error);
    bool hello(const std::string& endpointUrl, std::string& error);
    bool openSecureChannel(std::string& error);
    bool createSession(std::string& error);
    bool activateSession(std::string& error);
    bool browse(std::uint32_t nodeId, std::vector<OpcUaReference>& out, std::string& error);
    bool readBool(std::uint32_t nodeId, bool& value, std::string& error);
    bool writeBool(std::uint32_t nodeId, bool value, std::string& error);
    bool createSubscription(std::uint32_t& subscriptionId, std::string& error);
    bool monitorBool(std::uint32_t subscriptionId, std::uint32_t nodeId, std::uint32_t clientHandle, std::string& error);
    bool publish(std::uint32_t& clientHandle, bool& value, std::string& error);
    void close();

private:
    int fd_ = -1;
    std::uint32_t channelId_ = 0;
    std::uint32_t tokenId_ = 0;
    std::uint32_t sequence_ = 1;
    std::uint32_t requestId_ = 1;
    std::uint32_t authToken_ = 0;
};

// Copies a monitored value onto a scene signal the logic machine already waits on.
void publishPlcBool(Scene& scene, const std::string& signalName, bool value);

}  // namespace sim
