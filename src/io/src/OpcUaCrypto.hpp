#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

struct UaSymKeys {
    std::vector<std::uint8_t> sign;
    std::vector<std::uint8_t> enc;
    std::vector<std::uint8_t> iv;
};

struct UaOpened {
    std::uint32_t channelId = 0;
    std::uint32_t tokenId = 0;
    std::uint32_t sequence = 0;
    std::uint32_t requestId = 0;
    std::vector<std::uint8_t> senderDer;
    std::vector<std::uint8_t> body;
};

bool uaSelfSigned(const std::string& commonName, std::vector<std::uint8_t>& der, std::string& keyPem, std::string& error);
bool uaSavePem(const std::vector<std::uint8_t>& der, const std::string& keyPem, const std::string& certificatePath,
               const std::string& keyPath, std::string& error);
bool uaLoadPem(const std::string& certificatePath, const std::string& keyPath, std::vector<std::uint8_t>& der, std::string& keyPem,
               std::string& error);
std::vector<std::uint8_t> uaSha1(const std::vector<std::uint8_t>& data);
std::vector<std::uint8_t> uaRandom(std::size_t count);
bool uaDeriveKeys(const std::vector<std::uint8_t>& secretNonce, const std::vector<std::uint8_t>& seedNonce, UaSymKeys& out);

// mode 2 signs. mode 3 signs and encrypts. The message includes the 8-byte chunk header.
bool uaSealAsymmetric(int mode, const std::vector<std::uint8_t>& senderDer, const std::string& senderKeyPem,
                      const std::vector<std::uint8_t>& receiverDer, std::uint32_t channelId, std::uint32_t sequence,
                      std::uint32_t requestId, const std::uint8_t* body, std::size_t bodySize, std::vector<std::uint8_t>& message,
                      std::string& error);
bool uaOpenAsymmetric(int mode, const std::vector<std::uint8_t>& receiverDer, const std::string& receiverKeyPem,
                      const std::uint8_t* message, std::size_t messageSize, UaOpened& out, std::string& error);
bool uaSealSymmetric(int mode, const UaSymKeys& keys, std::uint32_t channelId, std::uint32_t tokenId, std::uint32_t sequence,
                     std::uint32_t requestId, const std::uint8_t* body, std::size_t bodySize, std::vector<std::uint8_t>& message,
                     std::string& error);
bool uaOpenSymmetric(int mode, const UaSymKeys& keys, const std::uint8_t* message, std::size_t messageSize, UaOpened& out,
                     std::string& error);
bool uaEncryptSecret(const std::vector<std::uint8_t>& certificateDer, const std::string& password,
                     const std::vector<std::uint8_t>& nonce, std::vector<std::uint8_t>& out, std::string& error);
bool uaDecryptSecret(const std::string& keyPem, const std::vector<std::uint8_t>& secret, const std::vector<std::uint8_t>& nonce,
                     std::string& password, std::string& error);

}  // namespace sim
