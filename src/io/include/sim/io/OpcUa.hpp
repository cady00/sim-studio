#pragma once

#include "sim/core/Scene.hpp"
#include "sim/core/Socket.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

// Binary OPC UA over TCP. SecurityPolicy None, and Basic256Sha256 Sign / SignAndEncrypt.
// The message layout follows the published Part 6 chunk headers and the numeric
// encoding node ids. Crypto uses OpenSSL 3. No OPC UA stack is linked.
enum class OpcUaSecurity { None, Sign, SignAndEncrypt };

struct OpcUaReference {
    std::uint32_t nodeId = 0;
    std::string browseName;
};

struct OpcUaCertificate {
    std::vector<std::uint8_t> der;
    std::string keyPem;
    bool createSelfSigned(const std::string& commonName, std::string& error);
    bool savePem(const std::string& certificatePath, const std::string& keyPath, std::string& error) const;
    bool loadPem(const std::string& certificatePath, const std::string& keyPath, std::string& error);
};

struct ImportedSignal {
    std::string name;
    SignalType type = SignalType::Bool;
    bool flag = false;
    std::int64_t integer = 0;
    double real = 0;
    bool hasValue = false;
};

class OpcUaServer {
public:
    OpcUaServer();
    ~OpcUaServer();
    OpcUaServer(const OpcUaServer&) = delete;
    OpcUaServer& operator=(const OpcUaServer&) = delete;

    bool listen(int port, std::string& error);
    int port() const { return port_; }
    void useSecurity(OpcUaSecurity mode, const OpcUaCertificate& own);
    void setUser(const std::string& name, const std::string& password);
    // Accepts one client and answers until that socket closes.
    bool serve(std::string& error);
    void close();

private:
    SocketFd listenFd_ = kInvalidSocket;
    int port_ = 0;
    bool partSeen_ = false;
    std::uint32_t monitoredHandle_ = 7;
    OpcUaSecurity security_ = OpcUaSecurity::None;
    OpcUaCertificate own_;
    std::string user_;
    std::string password_;
    std::vector<std::uint8_t> clientNonce_;
    std::vector<std::uint8_t> serverNonce_;
    std::vector<std::uint8_t> sessionNonce_;
    std::vector<std::uint8_t> clientSign_;
    std::vector<std::uint8_t> clientEnc_;
    std::vector<std::uint8_t> serverSign_;
    std::vector<std::uint8_t> serverEnc_;
    std::vector<std::uint8_t> peerDer_;
};

class OpcUaClient {
public:
    OpcUaClient();
    ~OpcUaClient();
    OpcUaClient(const OpcUaClient&) = delete;
    OpcUaClient& operator=(const OpcUaClient&) = delete;

    bool connect(const std::string& host, int port, std::string& error);
    bool hello(const std::string& endpointUrl, std::string& error);
    void useSecurity(OpcUaSecurity mode, const OpcUaCertificate& own, const OpcUaCertificate& peer);
    void setUser(const std::string& name, const std::string& password);
    bool openSecureChannel(std::string& error);
    bool createSession(std::string& error);
    bool activateSession(std::string& error);
    // Repeats connect, hello, open, create, and activate after the socket drops.
    bool reconnect(std::string& error);
    bool browse(std::uint32_t nodeId, std::vector<OpcUaReference>& out, std::string& error);
    bool readBool(std::uint32_t nodeId, bool& value, std::string& error);
    bool writeBool(std::uint32_t nodeId, bool value, std::string& error);
    bool createSubscription(std::uint32_t& subscriptionId, std::string& error);
    bool monitorBool(std::uint32_t subscriptionId, std::uint32_t nodeId, std::uint32_t clientHandle, std::string& error);
    bool publish(std::uint32_t& clientHandle, bool& value, std::string& error);
    void close();

private:
    SocketFd fd_ = kInvalidSocket;
    std::uint32_t channelId_ = 0;
    std::uint32_t tokenId_ = 0;
    std::uint32_t sequence_ = 1;
    std::uint32_t requestId_ = 1;
    std::uint32_t authToken_ = 0;
    OpcUaSecurity security_ = OpcUaSecurity::None;
    OpcUaCertificate own_;
    OpcUaCertificate peer_;
    std::string host_;
    int portNumber_ = 0;
    std::string endpoint_;
    std::string user_;
    std::string password_;
    std::vector<std::uint8_t> sessionNonce_;
    std::vector<std::uint8_t> clientSign_;
    std::vector<std::uint8_t> clientEnc_;
    std::vector<std::uint8_t> serverSign_;
    std::vector<std::uint8_t> serverEnc_;
    bool transact(const std::uint8_t* body, std::size_t size, std::vector<std::uint8_t>& reply, std::string& error);
};

// Copies a monitored value onto a scene signal the logic machine already waits on.
void publishPlcBool(Scene& scene, const std::string& signalName, bool value);
void publishPlcNumber(Scene& scene, const std::string& signalName, double value);
// CSV (name,type,value) or a TIA Portal tag-table text export (semicolon, quoted).
bool importSignalTable(const std::string& text, std::vector<ImportedSignal>& rows, std::string& error);
void applyImportedSignals(Scene& scene, const std::vector<ImportedSignal>& rows);

}  // namespace sim
