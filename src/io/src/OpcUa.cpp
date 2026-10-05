#include "sim/io/OpcUa.hpp"

#include "OpcUaCrypto.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cstring>
#include <vector>

namespace sim {
namespace {

constexpr const char* kPolicyNone = "http://opcfoundation.org/UA/SecurityPolicy#None";
constexpr std::uint32_t kObjectsFolder = 85;
constexpr std::uint32_t kPartSeen = 10001;
constexpr std::uint32_t kOpenRequest = 446;
constexpr std::uint32_t kOpenResponse = 449;
constexpr std::uint32_t kCreateSessionRequest = 461;
constexpr std::uint32_t kCreateSessionResponse = 464;
constexpr std::uint32_t kActivateSessionRequest = 467;
constexpr std::uint32_t kActivateSessionResponse = 470;
constexpr std::uint32_t kBrowseRequest = 527;
constexpr std::uint32_t kBrowseResponse = 530;
constexpr std::uint32_t kReadRequest = 631;
constexpr std::uint32_t kReadResponse = 634;
constexpr std::uint32_t kWriteRequest = 673;
constexpr std::uint32_t kWriteResponse = 676;
constexpr std::uint32_t kMonitorRequest = 751;
constexpr std::uint32_t kMonitorResponse = 754;
constexpr std::uint32_t kSubscriptionRequest = 787;
constexpr std::uint32_t kSubscriptionResponse = 790;
constexpr std::uint32_t kPublishRequest = 826;
constexpr std::uint32_t kPublishResponse = 829;
constexpr std::uint32_t kDataChange = 811;
constexpr std::uint32_t kAnonymousToken = 321;
constexpr std::uint32_t kUserToken = 324;

class Writer {
public:
    void u8(std::uint8_t value) { bytes_.push_back(value); }
    void u16(std::uint16_t value) { put(value); }
    void u32(std::uint32_t value) { put(value); }
    void i32(std::int32_t value) { put(static_cast<std::uint32_t>(value)); }
    void i64(std::int64_t value) { put(static_cast<std::uint64_t>(value)); }
    void f64(double value) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        put(bits);
    }
    void boolean(bool value) { u8(value ? 1 : 0); }
    void string(const char* text) {
        if (text == nullptr) {
            i32(-1);
            return;
        }
        const int length = static_cast<int>(std::strlen(text));
        i32(length);
        bytes_.insert(bytes_.end(), text, text + length);
    }
    void byteString(const std::uint8_t* data, int count) {
        i32(count);
        if (count > 0 && data != nullptr) {
            bytes_.insert(bytes_.end(), data, data + count);
        }
    }
    void nodeId(std::uint32_t id, std::uint16_t ns = 0) {
        if (ns == 0 && id < 256) {
            u8(0x00);
            u8(static_cast<std::uint8_t>(id));
            return;
        }
        if (ns < 256 && id <= 0xffffu) {
            u8(0x01);
            u8(static_cast<std::uint8_t>(ns));
            u16(static_cast<std::uint16_t>(id));
            return;
        }
        u8(0x02);
        u16(ns);
        u32(id);
    }
    void localized(const char* text) {
        u8(0x02);
        string(text);
    }
    void qualified(std::uint16_t ns, const char* name) {
        u16(ns);
        string(name);
    }
    void nullExtension() {
        nodeId(0);
        u8(0);
    }
    void requestHeader(std::uint32_t authToken, std::uint32_t handle) {
        nodeId(authToken);
        i64(0);
        u32(handle);
        u32(0);
        string(nullptr);
        u32(0);
        nullExtension();
    }
    void responseHeader(std::uint32_t handle, std::uint32_t status = 0) {
        i64(0);
        u32(handle);
        u32(status);
        u8(0);
        i32(0);
        nullExtension();
    }
    void dataValueBool(bool value) {
        u8(0x03);
        u8(0x01);
        boolean(value);
        u32(0);
    }
    void append(const Writer& other) { bytes_.insert(bytes_.end(), other.bytes_.begin(), other.bytes_.end()); }
    void appendRaw(const std::uint8_t* data, std::size_t size) {
        if (data != nullptr && size > 0) {
            bytes_.insert(bytes_.end(), data, data + size);
        }
    }
    void patchSize() {
        const auto size = static_cast<std::uint32_t>(bytes_.size());
        bytes_[4] = static_cast<std::uint8_t>(size & 0xffu);
        bytes_[5] = static_cast<std::uint8_t>((size >> 8) & 0xffu);
        bytes_[6] = static_cast<std::uint8_t>((size >> 16) & 0xffu);
        bytes_[7] = static_cast<std::uint8_t>((size >> 24) & 0xffu);
    }
    const std::uint8_t* data() const { return bytes_.data(); }
    std::size_t size() const { return bytes_.size(); }

private:
    template <typename T>
    void put(T value) {
        for (int i = 0; i < static_cast<int>(sizeof(T)); ++i) {
            bytes_.push_back(static_cast<std::uint8_t>((static_cast<std::uint64_t>(value) >> (8 * i)) & 0xffu));
        }
    }
    std::vector<std::uint8_t> bytes_;
};

class Reader {
public:
    Reader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}
    bool ok() const { return ok_; }
    std::size_t cursor() const { return cursor_; }
    std::uint8_t u8() { return static_cast<std::uint8_t>(take(1)); }
    std::uint16_t u16() { return static_cast<std::uint16_t>(take(2)); }
    std::uint32_t u32() { return static_cast<std::uint32_t>(take(4)); }
    std::int32_t i32() { return static_cast<std::int32_t>(take(4)); }
    std::int64_t i64() { return static_cast<std::int64_t>(take(8)); }
    double f64() {
        const std::uint64_t bits = take(8);
        double value = 0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }
    bool boolean() { return u8() != 0; }
    std::string string() {
        const std::int32_t count = i32();
        if (!ok_ || count < 0) {
            return {};
        }
        if (cursor_ + static_cast<std::size_t>(count) > size_) {
            ok_ = false;
            return {};
        }
        std::string text(reinterpret_cast<const char*>(data_ + cursor_), static_cast<std::size_t>(count));
        cursor_ += static_cast<std::size_t>(count);
        return text;
    }
    std::vector<std::uint8_t> bytes() {
        const std::int32_t count = i32();
        if (!ok_ || count < 0) {
            return {};
        }
        if (cursor_ + static_cast<std::size_t>(count) > size_) {
            ok_ = false;
            return {};
        }
        std::vector<std::uint8_t> out(data_ + cursor_, data_ + cursor_ + static_cast<std::size_t>(count));
        cursor_ += static_cast<std::size_t>(count);
        return out;
    }
    void skipBytes() { bytes(); }
    std::uint32_t nodeId() {
        const std::uint8_t encoding = u8();
        const std::uint8_t kind = static_cast<std::uint8_t>(encoding & 0x3f);
        std::uint32_t id = 0;
        if (kind == 0x00) {
            id = u8();
        } else if (kind == 0x01) {
            u8();
            id = u16();
        } else if (kind == 0x02) {
            u16();
            id = u32();
        } else if (kind == 0x03 || kind == 0x05) {
            u16();
            string();
        } else if (kind == 0x04) {
            u16();
            take(16);
        } else {
            ok_ = false;
        }
        if (encoding & 0x80) {
            u32();
        }
        if (encoding & 0x40) {
            string();
        }
        return id;
    }
    void skipExtension() {
        nodeId();
        const std::uint8_t encoding = u8();
        if (encoding == 0x01) {
            skipBytes();
        }
    }
    std::uint32_t requestHeader() {
        nodeId();
        i64();
        const std::uint32_t handle = u32();
        u32();
        string();
        u32();
        skipExtension();
        return handle;
    }
    void responseHeader() {
        i64();
        u32();
        const std::uint32_t status = u32();
        u8();
        const std::int32_t strings = i32();
        for (std::int32_t i = 0; i < strings && ok_; ++i) {
            string();
        }
        skipExtension();
        if (status != 0) {
            ok_ = false;
        }
    }
    bool dataValueBool(bool& value) {
        const std::uint8_t mask = u8();
        value = false;
        std::uint32_t status = 0;
        if (mask & 0x01) {
            const std::uint8_t variant = u8();
            if ((variant & 0x3f) != 0x01 || (variant & 0x80) != 0) {
                ok_ = false;
                return false;
            }
            value = boolean();
        }
        if (mask & 0x02) {
            status = u32();
        }
        if (mask & 0x04) {
            i64();
        }
        if (mask & 0x08) {
            i64();
        }
        if (mask & 0x10 || mask & 0x20) {
            if (mask & 0x10) {
                u16();
            }
            if (mask & 0x20) {
                u16();
            }
        }
        if (status != 0) {
            ok_ = false;
        }
        return ok_;
    }

private:
    std::uint64_t take(int width) {
        if (!ok_ || cursor_ + static_cast<std::size_t>(width) > size_) {
            ok_ = false;
            return 0;
        }
        std::uint64_t value = 0;
        for (int i = 0; i < width; ++i) {
            value |= static_cast<std::uint64_t>(data_[cursor_ + static_cast<std::size_t>(i)]) << (8 * i);
        }
        cursor_ += static_cast<std::size_t>(width);
        return value;
    }
    const std::uint8_t* data_ = nullptr;
    std::size_t size_ = 0;
    std::size_t cursor_ = 0;
    bool ok_ = true;
};

bool writeAll(int fd, const std::uint8_t* data, std::size_t size) {
    std::size_t sent = 0;
    while (sent < size) {
        const ssize_t wrote = ::send(fd, data + sent, size - sent, MSG_NOSIGNAL);
        if (wrote <= 0) {
            return false;
        }
        sent += static_cast<std::size_t>(wrote);
    }
    return true;
}

bool readAll(int fd, std::uint8_t* data, std::size_t size) {
    std::size_t got = 0;
    while (got < size) {
        const ssize_t read = ::recv(fd, data + got, size - got, 0);
        if (read <= 0) {
            return false;
        }
        got += static_cast<std::size_t>(read);
    }
    return true;
}

struct Chunk {
    char type[3] = {};
    std::uint32_t channelId = 0;
    std::uint32_t tokenId = 0;
    std::uint32_t sequence = 0;
    std::uint32_t requestId = 0;
    std::vector<std::uint8_t> body;
};

bool recvChunk(int fd, Chunk& chunk) {
    std::uint8_t header[8];
    if (!readAll(fd, header, sizeof(header))) {
        return false;
    }
    std::memcpy(chunk.type, header, 3);
    const std::uint32_t size = static_cast<std::uint32_t>(header[4]) | (static_cast<std::uint32_t>(header[5]) << 8) |
                               (static_cast<std::uint32_t>(header[6]) << 16) | (static_cast<std::uint32_t>(header[7]) << 24);
    if (size < 8 || size > 8u * 1024u * 1024u) {
        return false;
    }
    std::vector<std::uint8_t> rest(size - 8);
    if (size > 8 && !readAll(fd, rest.data(), rest.size())) {
        return false;
    }
    if (chunk.type[0] == 'H' || chunk.type[0] == 'A') {
        chunk.body = std::move(rest);
        return true;
    }
    Reader reader(rest.data(), rest.size());
    chunk.channelId = reader.u32();
    if (chunk.type[0] == 'M') {
        chunk.tokenId = reader.u32();
    } else if (chunk.type[0] == 'O') {
        reader.string();
        reader.skipBytes();
        reader.skipBytes();
    } else {
        return false;
    }
    chunk.sequence = reader.u32();
    chunk.requestId = reader.u32();
    if (!reader.ok() || reader.cursor() > rest.size()) {
        return false;
    }
    chunk.body.assign(rest.begin() + static_cast<std::ptrdiff_t>(reader.cursor()), rest.end());
    return true;
}

bool sendHello(int fd, const char* kind, const Writer& body) {
    Writer message;
    message.u8(static_cast<std::uint8_t>(kind[0]));
    message.u8(static_cast<std::uint8_t>(kind[1]));
    message.u8(static_cast<std::uint8_t>(kind[2]));
    message.u8('F');
    message.u32(0);
    message.append(body);
    message.patchSize();
    return writeAll(fd, message.data(), message.size());
}

bool sendSecure(int fd, const char* kind, std::uint32_t channelId, bool messageToken, std::uint32_t tokenId,
                const Writer* openPrefix, std::uint32_t sequence, std::uint32_t requestId, const Writer& body) {
    Writer message;
    message.u8(static_cast<std::uint8_t>(kind[0]));
    message.u8(static_cast<std::uint8_t>(kind[1]));
    message.u8(static_cast<std::uint8_t>(kind[2]));
    message.u8('F');
    message.u32(0);
    message.u32(channelId);
    if (messageToken) {
        message.u32(tokenId);
    } else if (openPrefix != nullptr) {
        message.append(*openPrefix);
    }
    message.u32(sequence);
    message.u32(requestId);
    message.append(body);
    message.patchSize();
    return writeAll(fd, message.data(), message.size());
}

void writeApplication(Writer& writer, const char* uri, const char* name, std::int32_t kind) {
    writer.string(uri);
    writer.string(uri);
    writer.localized(name);
    writer.i32(kind);
    writer.string(nullptr);
    writer.string(nullptr);
    writer.i32(0);
}

void readApplication(Reader& reader) {
    reader.string();
    reader.string();
    reader.u8();
    reader.string();
    reader.i32();
    reader.string();
    reader.string();
    const std::int32_t urls = reader.i32();
    for (std::int32_t i = 0; i < urls && reader.ok(); ++i) {
        reader.string();
    }
}

void writeEndpoint(Writer& writer) {
    writer.string("opc.tcp://127.0.0.1:4840");
    writeApplication(writer, "urn:simstudio:server", "Sim Studio", 0);
    writer.byteString(nullptr, -1);
    writer.i32(1);
    writer.string(kPolicyNone);
    writer.i32(1);
    writer.string("anonymous");
    writer.i32(0);
    writer.string(nullptr);
    writer.string(nullptr);
    writer.string(kPolicyNone);
    writer.string("http://opcfoundation.org/UA-Profile/Transport/uatcp-uasc-uabinary");
    writer.u8(0);
}

void readEndpoint(Reader& reader) {
    reader.string();
    readApplication(reader);
    reader.skipBytes();
    reader.i32();
    reader.string();
    const std::int32_t tokens = reader.i32();
    for (std::int32_t i = 0; i < tokens && reader.ok(); ++i) {
        reader.string();
        reader.i32();
        reader.string();
        reader.string();
        reader.string();
    }
    reader.string();
    reader.u8();
}

bool readBoolValue(Reader& reader, bool& value) { return reader.dataValueBool(value) && reader.ok(); }

bool readFrame(int fd, std::vector<std::uint8_t>& raw) {
    std::uint8_t header[8];
    if (!readAll(fd, header, sizeof(header))) {
        return false;
    }
    const std::uint32_t size = static_cast<std::uint32_t>(header[4]) | (static_cast<std::uint32_t>(header[5]) << 8) |
                               (static_cast<std::uint32_t>(header[6]) << 16) | (static_cast<std::uint32_t>(header[7]) << 24);
    if (size < 8 || size > 8u * 1024u * 1024u) {
        return false;
    }
    raw.assign(header, header + 8);
    raw.resize(size);
    return size == 8 || readAll(fd, raw.data() + 8, size - 8);
}

int securityMode(OpcUaSecurity security) { return security == OpcUaSecurity::SignAndEncrypt ? 3 : 2; }

UaSymKeys symKeys(const std::vector<std::uint8_t>& sign, const std::vector<std::uint8_t>& enc) {
    UaSymKeys keys;
    keys.sign = sign;
    keys.enc = enc;
    return keys;
}

}  // namespace

OpcUaServer::OpcUaServer() = default;

OpcUaServer::~OpcUaServer() { close(); }

bool OpcUaServer::listen(int port, std::string& error) {
    close();
    listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd_ < 0) {
        error = "socket failed";
        return false;
    }
    const int reuse = 1;
    ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(listenFd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        error = "bind failed";
        close();
        return false;
    }
    if (::listen(listenFd_, 1) != 0) {
        error = "listen failed";
        close();
        return false;
    }
    socklen_t length = sizeof(address);
    if (::getsockname(listenFd_, reinterpret_cast<sockaddr*>(&address), &length) != 0) {
        error = "getsockname failed";
        close();
        return false;
    }
    port_ = ntohs(address.sin_port);
    error.clear();
    return true;
}

void OpcUaServer::close() {
    if (listenFd_ >= 0) {
        ::close(listenFd_);
        listenFd_ = -1;
    }
}

void OpcUaServer::useSecurity(OpcUaSecurity mode, const OpcUaCertificate& own) {
    security_ = mode;
    own_ = own;
}

void OpcUaServer::setUser(const std::string& name, const std::string& password) {
    user_ = name;
    password_ = password;
}

bool OpcUaServer::serve(std::string& error) {
    if (listenFd_ < 0) {
        error = "server is not listening";
        return false;
    }
    const int client = ::accept(listenFd_, nullptr, nullptr);
    if (client < 0) {
        error = "accept failed";
        return false;
    }
    std::uint32_t sequence = 1;
    bool ok = true;
    while (ok) {
        Chunk chunk;
        if (security_ == OpcUaSecurity::None) {
            if (!recvChunk(client, chunk)) {
                break;
            }
        } else {
            std::vector<std::uint8_t> raw;
            if (!readFrame(client, raw)) {
                break;
            }
            chunk.type[0] = static_cast<char>(raw[0]);
            if (raw[0] == 'H' || raw[0] == 'A') {
                chunk.body.assign(raw.begin() + 8, raw.end());
            } else if (raw[0] == 'O') {
                UaOpened opened;
                std::string cryptoError;
                if (!uaOpenAsymmetric(securityMode(security_), own_.der, own_.keyPem, raw.data(), raw.size(), opened, cryptoError)) {
                    error = cryptoError;
                    ok = false;
                    break;
                }
                chunk.channelId = opened.channelId;
                chunk.sequence = opened.sequence;
                chunk.requestId = opened.requestId;
                chunk.body = std::move(opened.body);
                peerDer_ = std::move(opened.senderDer);
            } else if (raw[0] == 'M') {
                UaOpened opened;
                std::string cryptoError;
                if (!uaOpenSymmetric(securityMode(security_), symKeys(clientSign_, clientEnc_), raw.data(), raw.size(), opened,
                                     cryptoError)) {
                    error = cryptoError;
                    ok = false;
                    break;
                }
                chunk.channelId = opened.channelId;
                chunk.tokenId = opened.tokenId;
                chunk.sequence = opened.sequence;
                chunk.requestId = opened.requestId;
                chunk.body = std::move(opened.body);
            } else {
                ok = false;
                break;
            }
        }
        if (chunk.type[0] == 'H') {
            Reader reader(chunk.body.data(), chunk.body.size());
            reader.u32();
            reader.u32();
            reader.u32();
            reader.u32();
            reader.u32();
            reader.string();
            Writer body;
            body.u32(0);
            body.u32(65536);
            body.u32(65536);
            body.u32(0);
            body.u32(0);
            ok = reader.ok() && sendHello(client, "ACK", body);
            continue;
        }
        if (chunk.type[0] == 'O') {
            Reader reader(chunk.body.data(), chunk.body.size());
            const std::uint32_t typeId = reader.nodeId();
            const std::uint32_t handle = reader.requestHeader();
            reader.u32();
            reader.i32();
            const std::int32_t requestedMode = reader.i32();
            clientNonce_ = reader.bytes();
            reader.u32();
            Writer body;
            body.nodeId(kOpenResponse);
            body.responseHeader(handle);
            body.u32(0);
            body.u32(1);
            body.u32(1);
            body.i64(0);
            body.u32(600000);
            if (security_ == OpcUaSecurity::None) {
                const std::uint8_t nonce = 1;
                body.byteString(&nonce, 1);
                Writer prefix;
                prefix.string(kPolicyNone);
                prefix.byteString(nullptr, -1);
                prefix.byteString(nullptr, -1);
                ok = reader.ok() && typeId == kOpenRequest &&
                     sendSecure(client, "OPN", 1, false, 0, &prefix, sequence++, chunk.requestId, body);
            } else {
                serverNonce_ = uaRandom(32);
                UaSymKeys clientKeys;
                UaSymKeys serverKeys;
                const bool derived = uaDeriveKeys(serverNonce_, clientNonce_, clientKeys) &&
                                     uaDeriveKeys(clientNonce_, serverNonce_, serverKeys);
                clientSign_ = clientKeys.sign;
                clientEnc_ = clientKeys.enc;
                serverSign_ = serverKeys.sign;
                serverEnc_ = serverKeys.enc;
                body.byteString(serverNonce_.data(), static_cast<int>(serverNonce_.size()));
                std::vector<std::uint8_t> message;
                std::string cryptoError;
                ok = reader.ok() && derived && typeId == kOpenRequest && requestedMode == securityMode(security_) &&
                     uaSealAsymmetric(securityMode(security_), own_.der, own_.keyPem, peerDer_, 1, sequence++, chunk.requestId,
                                      body.data(), body.size(), message, cryptoError) &&
                     writeAll(client, message.data(), message.size());
                if (!ok && error.empty()) {
                    error = cryptoError.empty() ? "open rejected" : cryptoError;
                }
            }
            continue;
        }
        if (chunk.type[0] != 'M') {
            ok = false;
            break;
        }
        Reader reader(chunk.body.data(), chunk.body.size());
        const std::uint32_t typeId = reader.nodeId();
        const std::uint32_t handle = reader.requestHeader();
        Writer body;
        if (typeId == kCreateSessionRequest) {
            readApplication(reader);
            reader.string();
            reader.string();
            reader.string();
            reader.skipBytes();
            reader.skipBytes();
            reader.f64();
            reader.u32();
            body.nodeId(kCreateSessionResponse);
            body.responseHeader(handle);
            body.nodeId(1);
            body.nodeId(2);
            body.f64(60000);
            if (security_ == OpcUaSecurity::None) {
                const std::uint8_t nonce = 2;
                body.byteString(&nonce, 1);
                body.byteString(nullptr, -1);
            } else {
                sessionNonce_ = uaRandom(32);
                body.byteString(sessionNonce_.data(), static_cast<int>(sessionNonce_.size()));
                body.byteString(own_.der.data(), static_cast<int>(own_.der.size()));
            }
            body.i32(1);
            writeEndpoint(body);
            body.i32(0);
            body.string(nullptr);
            body.byteString(nullptr, -1);
            body.u32(0);
        } else if (typeId == kActivateSessionRequest) {
            reader.string();
            reader.skipBytes();
            reader.i32();
            const std::int32_t locales = reader.i32();
            for (std::int32_t i = 0; i < locales && reader.ok(); ++i) {
                reader.string();
            }
            std::uint32_t activateStatus = 0;
            if (security_ != OpcUaSecurity::None && !user_.empty()) {
                const std::uint32_t tokenType = reader.nodeId();
                const std::uint8_t encoding = reader.u8();
                const std::vector<std::uint8_t> token = reader.bytes();
                Reader identity(token.data(), token.size());
                identity.string();
                const std::string name = identity.string();
                const std::vector<std::uint8_t> secret = identity.bytes();
                std::string given;
                std::string cryptoError;
                if (tokenType != kUserToken || encoding != 0x01 ||
                    !uaDecryptSecret(own_.keyPem, secret, sessionNonce_, given, cryptoError) || name != user_ || given != password_) {
                    activateStatus = 0x801F0000u;
                }
            } else {
                reader.skipExtension();
            }
            reader.string();
            reader.skipBytes();
            body.nodeId(kActivateSessionResponse);
            body.responseHeader(handle, activateStatus);
            const std::uint8_t nonce = 3;
            body.byteString(&nonce, 1);
            body.i32(0);
            body.i32(0);
        } else if (typeId == kBrowseRequest) {
            reader.nodeId();
            reader.i64();
            reader.u32();
            reader.u32();
            const std::int32_t count = reader.i32();
            std::uint32_t start = 0;
            for (std::int32_t i = 0; i < count && reader.ok(); ++i) {
                start = reader.nodeId();
                reader.i32();
                reader.nodeId();
                reader.boolean();
                reader.u32();
                reader.u32();
            }
            body.nodeId(kBrowseResponse);
            body.responseHeader(handle);
            body.i32(1);
            body.u32(0);
            body.byteString(nullptr, -1);
            if (start == kObjectsFolder) {
                body.i32(1);
                body.nodeId(35);
                body.boolean(true);
                body.nodeId(kPartSeen);
                body.qualified(0, "PartSeen");
                body.localized("PartSeen");
                body.i32(2);
                body.nodeId(63);
            } else {
                body.i32(0);
            }
            body.i32(0);
        } else if (typeId == kReadRequest) {
            reader.f64();
            reader.i32();
            const std::int32_t count = reader.i32();
            std::uint32_t id = 0;
            for (std::int32_t i = 0; i < count && reader.ok(); ++i) {
                id = reader.nodeId();
                reader.u32();
                reader.string();
                reader.u16();
                reader.string();
            }
            body.nodeId(kReadResponse);
            body.responseHeader(handle);
            body.i32(1);
            if (id == kPartSeen) {
                body.dataValueBool(partSeen_);
            } else {
                body.u8(0x02);
                body.u32(0x80340000u);
            }
            body.i32(0);
        } else if (typeId == kWriteRequest) {
            const std::int32_t count = reader.i32();
            std::uint32_t id = 0;
            bool value = false;
            for (std::int32_t i = 0; i < count && reader.ok(); ++i) {
                id = reader.nodeId();
                reader.u32();
                reader.string();
                readBoolValue(reader, value);
            }
            if (id == kPartSeen && reader.ok()) {
                partSeen_ = value;
            }
            body.nodeId(kWriteResponse);
            body.responseHeader(handle);
            body.i32(1);
            body.u32(id == kPartSeen ? 0u : 0x80340000u);
            body.i32(0);
        } else if (typeId == kSubscriptionRequest) {
            reader.f64();
            reader.u32();
            reader.u32();
            reader.u32();
            reader.boolean();
            reader.u8();
            body.nodeId(kSubscriptionResponse);
            body.responseHeader(handle);
            body.u32(1);
            body.f64(100);
            body.u32(60);
            body.u32(10);
        } else if (typeId == kMonitorRequest) {
            reader.u32();
            reader.i32();
            const std::int32_t count = reader.i32();
            for (std::int32_t i = 0; i < count && reader.ok(); ++i) {
                reader.nodeId();
                reader.u32();
                reader.string();
                reader.u16();
                reader.string();
                reader.i32();
                monitoredHandle_ = reader.u32();
                reader.f64();
                reader.skipExtension();
                reader.u32();
                reader.boolean();
            }
            body.nodeId(kMonitorResponse);
            body.responseHeader(handle);
            body.i32(1);
            body.u32(0);
            body.u32(1);
            body.f64(100);
            body.u32(1);
            body.nullExtension();
        } else if (typeId == kPublishRequest) {
            const std::int32_t acks = reader.i32();
            for (std::int32_t i = 0; i < acks && reader.ok(); ++i) {
                reader.u32();
                reader.u32();
            }
            body.nodeId(kPublishResponse);
            body.responseHeader(handle);
            body.u32(1);
            body.i32(1);
            body.u32(1);
            body.boolean(false);
            body.u32(1);
            body.i64(0);
            Writer change;
            change.i32(1);
            change.u32(monitoredHandle_);
            change.dataValueBool(partSeen_);
            change.i32(0);
            body.i32(1);
            body.nodeId(kDataChange);
            body.u8(0x01);
            body.byteString(change.data(), static_cast<int>(change.size()));
            body.i32(0);
            body.i32(0);
        } else {
            ok = false;
        }
        if (security_ == OpcUaSecurity::None) {
            ok = ok && reader.ok() && sendSecure(client, "MSG", chunk.channelId, true, chunk.tokenId, nullptr, sequence++,
                                                 chunk.requestId, body);
        } else {
            std::vector<std::uint8_t> message;
            std::string cryptoError;
            ok = ok && reader.ok() &&
                 uaSealSymmetric(securityMode(security_), symKeys(serverSign_, serverEnc_), chunk.channelId, chunk.tokenId, sequence++,
                                 chunk.requestId, body.data(), body.size(), message, cryptoError) &&
                 writeAll(client, message.data(), message.size());
            if (!ok && error.empty()) {
                error = cryptoError.empty() ? "message rejected" : cryptoError;
            }
        }
    }
    ::close(client);
    error.clear();
    return true;
}

OpcUaClient::OpcUaClient() = default;

OpcUaClient::~OpcUaClient() { close(); }

void OpcUaClient::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool OpcUaClient::connect(const std::string& host, int port, std::string& error) {
    close();
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) {
        error = "socket failed";
        return false;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
        error = "bad host";
        close();
        return false;
    }
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        error = "connect failed";
        close();
        return false;
    }
    host_ = host;
    portNumber_ = port;
    error.clear();
    return true;
}

void OpcUaClient::useSecurity(OpcUaSecurity mode, const OpcUaCertificate& own, const OpcUaCertificate& peer) {
    security_ = mode;
    own_ = own;
    peer_ = peer;
}

void OpcUaClient::setUser(const std::string& name, const std::string& password) {
    user_ = name;
    password_ = password;
}

bool OpcUaClient::transact(const std::uint8_t* body, std::size_t size, std::vector<std::uint8_t>& reply, std::string& error) {
    if (security_ == OpcUaSecurity::None) {
        Writer wrapped;
        wrapped.appendRaw(body, size);
        if (!sendSecure(fd_, "MSG", channelId_, true, tokenId_, nullptr, sequence_++, requestId_, wrapped)) {
            error = "send failed";
            return false;
        }
        Chunk chunk;
        if (!recvChunk(fd_, chunk)) {
            error = "response missing";
            return false;
        }
        reply = std::move(chunk.body);
        error.clear();
        return true;
    }
    std::vector<std::uint8_t> message;
    if (!uaSealSymmetric(securityMode(security_), symKeys(clientSign_, clientEnc_), channelId_, tokenId_, sequence_++, requestId_, body,
                         size, message, error)) {
        return false;
    }
    if (!writeAll(fd_, message.data(), message.size())) {
        error = "send failed";
        return false;
    }
    std::vector<std::uint8_t> raw;
    if (!readFrame(fd_, raw)) {
        error = "response missing";
        return false;
    }
    UaOpened opened;
    if (!uaOpenSymmetric(securityMode(security_), symKeys(serverSign_, serverEnc_), raw.data(), raw.size(), opened, error)) {
        return false;
    }
    reply = std::move(opened.body);
    error.clear();
    return true;
}

bool OpcUaClient::reconnect(std::string& error) {
    close();
    sequence_ = 1;
    requestId_ = 1;
    authToken_ = 0;
    channelId_ = 0;
    tokenId_ = 0;
    if (host_.empty() || endpoint_.empty()) {
        error = "reconnect has no endpoint";
        return false;
    }
    return connect(host_, portNumber_, error) && hello(endpoint_, error) && openSecureChannel(error) && createSession(error) &&
           activateSession(error);
}

bool OpcUaClient::hello(const std::string& endpointUrl, std::string& error) {
    Writer body;
    body.u32(0);
    body.u32(65536);
    body.u32(65536);
    body.u32(0);
    body.u32(0);
    endpoint_ = endpointUrl;
    body.string(endpointUrl.c_str());
    if (!sendHello(fd_, "HEL", body)) {
        error = "hello send failed";
        return false;
    }
    Chunk chunk;
    if (!recvChunk(fd_, chunk) || chunk.type[0] != 'A') {
        error = "acknowledge missing";
        return false;
    }
    error.clear();
    return true;
}

bool OpcUaClient::openSecureChannel(std::string& error) {
    Writer body;
    body.nodeId(kOpenRequest);
    body.requestHeader(0, requestId_);
    body.u32(0);
    body.i32(0);
    std::vector<std::uint8_t> clientNonce{1};
    Chunk chunk;
    if (security_ == OpcUaSecurity::None) {
        body.i32(1);
        body.byteString(clientNonce.data(), 1);
        body.u32(600000);
        Writer prefix;
        prefix.string(kPolicyNone);
        prefix.byteString(nullptr, -1);
        prefix.byteString(nullptr, -1);
        if (!sendSecure(fd_, "OPN", 0, false, 0, &prefix, sequence_++, requestId_, body)) {
            error = "open send failed";
            return false;
        }
        if (!recvChunk(fd_, chunk) || chunk.type[0] != 'O') {
            error = "open response missing";
            return false;
        }
    } else {
        clientNonce = uaRandom(32);
        body.i32(securityMode(security_));
        body.byteString(clientNonce.data(), static_cast<int>(clientNonce.size()));
        body.u32(600000);
        std::vector<std::uint8_t> message;
        if (!uaSealAsymmetric(securityMode(security_), own_.der, own_.keyPem, peer_.der, 0, sequence_++, requestId_, body.data(),
                              body.size(), message, error) ||
            !writeAll(fd_, message.data(), message.size())) {
            if (error.empty()) {
                error = "open send failed";
            }
            return false;
        }
        std::vector<std::uint8_t> raw;
        UaOpened opened;
        if (!readFrame(fd_, raw) || !uaOpenAsymmetric(securityMode(security_), own_.der, own_.keyPem, raw.data(), raw.size(), opened, error)) {
            if (error.empty()) {
                error = "open response missing";
            }
            return false;
        }
        chunk.body = std::move(opened.body);
        chunk.type[0] = 'O';
    }
    Reader reader(chunk.body.data(), chunk.body.size());
    if (reader.nodeId() != kOpenResponse) {
        error = "open type mismatch";
        return false;
    }
    reader.responseHeader();
    reader.u32();
    channelId_ = reader.u32();
    tokenId_ = reader.u32();
    reader.i64();
    reader.u32();
    const std::vector<std::uint8_t> serverNonce = reader.bytes();
    if (security_ != OpcUaSecurity::None) {
        UaSymKeys clientKeys;
        UaSymKeys serverKeys;
        if (!uaDeriveKeys(serverNonce, clientNonce, clientKeys) || !uaDeriveKeys(clientNonce, serverNonce, serverKeys)) {
            error = "channel keys rejected";
            return false;
        }
        clientSign_ = clientKeys.sign;
        clientEnc_ = clientKeys.enc;
        serverSign_ = serverKeys.sign;
        serverEnc_ = serverKeys.enc;
    }
    if (!reader.ok() || channelId_ == 0 || tokenId_ == 0) {
        error = "open body rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

bool OpcUaClient::createSession(std::string& error) {
    Writer body;
    body.nodeId(kCreateSessionRequest);
    body.requestHeader(0, requestId_);
    writeApplication(body, "urn:simstudio:client", "Sim Studio", 1);
    body.string(nullptr);
    body.string("opc.tcp://127.0.0.1:4840");
    body.string("simstudio");
    if (security_ == OpcUaSecurity::None) {
        const std::uint8_t nonce = 4;
        body.byteString(&nonce, 1);
        body.byteString(nullptr, -1);
    } else {
        const std::vector<std::uint8_t> nonce = uaRandom(32);
        body.byteString(nonce.data(), static_cast<int>(nonce.size()));
        body.byteString(own_.der.data(), static_cast<int>(own_.der.size()));
    }
    body.f64(60000);
    body.u32(0);
    std::vector<std::uint8_t> reply;
    if (!transact(body.data(), body.size(), reply, error)) {
        return false;
    }
    Reader reader(reply.data(), reply.size());
    if (reader.nodeId() != kCreateSessionResponse) {
        error = "create session type mismatch";
        return false;
    }
    reader.responseHeader();
    reader.nodeId();
    authToken_ = reader.nodeId();
    reader.f64();
    sessionNonce_ = reader.bytes();
    reader.skipBytes();
    const std::int32_t endpoints = reader.i32();
    for (std::int32_t i = 0; i < endpoints && reader.ok(); ++i) {
        readEndpoint(reader);
    }
    const std::int32_t certificates = reader.i32();
    for (std::int32_t i = 0; i < certificates && reader.ok(); ++i) {
        reader.skipBytes();
        reader.skipBytes();
    }
    reader.string();
    reader.skipBytes();
    reader.u32();
    if (!reader.ok() || authToken_ == 0) {
        error = "create session body rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

bool OpcUaClient::activateSession(std::string& error) {
    Writer token;
    std::uint32_t tokenId = kAnonymousToken;
    if (user_.empty()) {
        token.string("anonymous");
    } else {
        std::vector<std::uint8_t> secret;
        if (!uaEncryptSecret(peer_.der, password_, sessionNonce_, secret, error)) {
            return false;
        }
        tokenId = kUserToken;
        token.string("username");
        token.string(user_.c_str());
        token.byteString(secret.data(), static_cast<int>(secret.size()));
        token.string("http://www.w3.org/2001/04/xmlenc#rsa-oaep");
    }
    Writer body;
    body.nodeId(kActivateSessionRequest);
    body.requestHeader(authToken_, requestId_);
    body.string(nullptr);
    body.byteString(nullptr, -1);
    body.i32(0);
    body.i32(1);
    body.string("en");
    body.nodeId(tokenId);
    body.u8(0x01);
    body.byteString(token.data(), static_cast<int>(token.size()));
    body.string(nullptr);
    body.byteString(nullptr, -1);
    std::vector<std::uint8_t> reply;
    if (!transact(body.data(), body.size(), reply, error)) {
        return false;
    }
    Reader reader(reply.data(), reply.size());
    if (reader.nodeId() != kActivateSessionResponse) {
        error = "activate type mismatch";
        return false;
    }
    reader.responseHeader();
    reader.skipBytes();
    reader.i32();
    reader.i32();
    if (!reader.ok()) {
        error = "activate body rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

bool OpcUaClient::browse(std::uint32_t nodeId, std::vector<OpcUaReference>& out, std::string& error) {
    out.clear();
    Writer body;
    body.nodeId(kBrowseRequest);
    body.requestHeader(authToken_, requestId_);
    body.nodeId(0);
    body.i64(0);
    body.u32(0);
    body.u32(0);
    body.i32(1);
    body.nodeId(nodeId);
    body.i32(0);
    body.nodeId(33);
    body.boolean(true);
    body.u32(0);
    body.u32(63);
    std::vector<std::uint8_t> reply;
    if (!transact(body.data(), body.size(), reply, error)) {
        return false;
    }
    Reader reader(reply.data(), reply.size());
    if (reader.nodeId() != kBrowseResponse) {
        error = "browse type mismatch";
        return false;
    }
    reader.responseHeader();
    const std::int32_t results = reader.i32();
    for (std::int32_t i = 0; i < results && reader.ok(); ++i) {
        reader.u32();
        reader.skipBytes();
        const std::int32_t refs = reader.i32();
        for (std::int32_t ref = 0; ref < refs && reader.ok(); ++ref) {
            reader.nodeId();
            reader.boolean();
            OpcUaReference item;
            item.nodeId = reader.nodeId();
            reader.u16();
            item.browseName = reader.string();
            reader.u8();
            reader.string();
            reader.i32();
            reader.nodeId();
            out.push_back(std::move(item));
        }
    }
    reader.i32();
    if (!reader.ok()) {
        error = "browse body rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

bool OpcUaClient::readBool(std::uint32_t nodeId, bool& value, std::string& error) {
    Writer body;
    body.nodeId(kReadRequest);
    body.requestHeader(authToken_, requestId_);
    body.f64(0);
    body.i32(3);
    body.i32(1);
    body.nodeId(nodeId);
    body.u32(13);
    body.string(nullptr);
    body.qualified(0, nullptr);
    std::vector<std::uint8_t> reply;
    if (!transact(body.data(), body.size(), reply, error)) {
        return false;
    }
    Reader reader(reply.data(), reply.size());
    if (reader.nodeId() != kReadResponse) {
        error = "read type mismatch";
        return false;
    }
    reader.responseHeader();
    const std::int32_t count = reader.i32();
    if (count < 1 || !readBoolValue(reader, value)) {
        error = "read body rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

bool OpcUaClient::writeBool(std::uint32_t nodeId, bool value, std::string& error) {
    Writer body;
    body.nodeId(kWriteRequest);
    body.requestHeader(authToken_, requestId_);
    body.i32(1);
    body.nodeId(nodeId);
    body.u32(13);
    body.string(nullptr);
    body.dataValueBool(value);
    std::vector<std::uint8_t> reply;
    if (!transact(body.data(), body.size(), reply, error)) {
        return false;
    }
    Reader reader(reply.data(), reply.size());
    if (reader.nodeId() != kWriteResponse) {
        error = "write type mismatch";
        return false;
    }
    reader.responseHeader();
    const std::int32_t count = reader.i32();
    const std::uint32_t status = count > 0 ? reader.u32() : 1u;
    if (!reader.ok() || status != 0) {
        error = "write status rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

bool OpcUaClient::createSubscription(std::uint32_t& subscriptionId, std::string& error) {
    Writer body;
    body.nodeId(kSubscriptionRequest);
    body.requestHeader(authToken_, requestId_);
    body.f64(100);
    body.u32(60);
    body.u32(10);
    body.u32(0);
    body.boolean(true);
    body.u8(0);
    std::vector<std::uint8_t> reply;
    if (!transact(body.data(), body.size(), reply, error)) {
        return false;
    }
    Reader reader(reply.data(), reply.size());
    if (reader.nodeId() != kSubscriptionResponse) {
        error = "subscription type mismatch";
        return false;
    }
    reader.responseHeader();
    subscriptionId = reader.u32();
    reader.f64();
    reader.u32();
    reader.u32();
    if (!reader.ok() || subscriptionId == 0) {
        error = "subscription body rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

bool OpcUaClient::monitorBool(std::uint32_t subscriptionId, std::uint32_t nodeId, std::uint32_t clientHandle,
                              std::string& error) {
    Writer body;
    body.nodeId(kMonitorRequest);
    body.requestHeader(authToken_, requestId_);
    body.u32(subscriptionId);
    body.i32(3);
    body.i32(1);
    body.nodeId(nodeId);
    body.u32(13);
    body.string(nullptr);
    body.qualified(0, nullptr);
    body.i32(2);
    body.u32(clientHandle);
    body.f64(100);
    body.nullExtension();
    body.u32(1);
    body.boolean(true);
    std::vector<std::uint8_t> reply;
    if (!transact(body.data(), body.size(), reply, error)) {
        return false;
    }
    Reader reader(reply.data(), reply.size());
    if (reader.nodeId() != kMonitorResponse) {
        error = "monitor type mismatch";
        return false;
    }
    reader.responseHeader();
    const std::int32_t count = reader.i32();
    const std::uint32_t status = count > 0 ? reader.u32() : 1u;
    if (!reader.ok() || status != 0) {
        error = "monitor status rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

bool OpcUaClient::publish(std::uint32_t& clientHandle, bool& value, std::string& error) {
    Writer body;
    body.nodeId(kPublishRequest);
    body.requestHeader(authToken_, requestId_);
    body.i32(0);
    std::vector<std::uint8_t> reply;
    if (!transact(body.data(), body.size(), reply, error)) {
        return false;
    }
    Reader reader(reply.data(), reply.size());
    if (reader.nodeId() != kPublishResponse) {
        error = "publish type mismatch";
        return false;
    }
    reader.responseHeader();
    reader.u32();
    const std::int32_t available = reader.i32();
    for (std::int32_t i = 0; i < available && reader.ok(); ++i) {
        reader.u32();
    }
    reader.boolean();
    reader.u32();
    reader.i64();
    const std::int32_t notes = reader.i32();
    if (notes < 1) {
        error = "publish had no notification";
        return false;
    }
    if (reader.nodeId() != kDataChange || reader.u8() != 0x01) {
        error = "publish notification type mismatch";
        return false;
    }
    const std::string payload = reader.string();
    Reader change(reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size());
    const std::int32_t items = change.i32();
    if (items < 1) {
        error = "publish item missing";
        return false;
    }
    clientHandle = change.u32();
    if (!readBoolValue(change, value) || !reader.ok()) {
        error = "publish value rejected";
        return false;
    }
    ++requestId_;
    error.clear();
    return true;
}

void publishPlcBool(Scene& scene, const std::string& signalName, bool value) {
    scene.upsertSignal(signalName, SignalType::Bool).flag = value;
}

void publishPlcNumber(Scene& scene, const std::string& signalName, double value) {
    scene.upsertSignal(signalName, SignalType::Real).real = value;
}

namespace {

std::string trimCopy(std::string text) {
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) {
        text.erase(text.begin());
    }
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) {
        text.pop_back();
    }
    return text;
}

std::string unquote(std::string text) {
    text = trimCopy(std::move(text));
    if (text.size() >= 2 && text.front() == '"' && text.back() == '"') {
        text = text.substr(1, text.size() - 2);
    }
    return text;
}

std::vector<std::string> splitDelimited(const std::string& line, char delimiter) {
    std::vector<std::string> fields;
    std::string current;
    bool quoted = false;
    for (char letter : line) {
        if (letter == '"') {
            quoted = !quoted;
            current.push_back(letter);
            continue;
        }
        if (letter == delimiter && !quoted) {
            fields.push_back(unquote(current));
            current.clear();
            continue;
        }
        current.push_back(letter);
    }
    fields.push_back(unquote(current));
    return fields;
}

bool signalTypeFromLabel(std::string label, SignalType& type) {
    for (char& letter : label) {
        letter = static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
    }
    if (label == "bool" || label == "boolean") {
        type = SignalType::Bool;
        return true;
    }
    if (label == "int" || label == "dint" || label == "word" || label == "dword" || label == "sint" || label == "uint") {
        type = SignalType::Int;
        return true;
    }
    if (label == "real" || label == "lreal" || label == "float" || label == "double") {
        type = SignalType::Real;
        return true;
    }
    return false;
}

bool headerName(const std::string& text) {
    std::string lower = text;
    for (char& letter : lower) {
        letter = static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
    }
    return lower == "name" || lower == "tag" || lower == "symbol";
}

}  // namespace

bool importSignalTable(const std::string& text, std::vector<ImportedSignal>& rows, std::string& error) {
    rows.clear();
    std::size_t cursor = 0;
    while (cursor <= text.size()) {
        const std::size_t end = text.find('\n', cursor);
        std::string line = text.substr(cursor, end == std::string::npos ? std::string::npos : end - cursor);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        cursor = end == std::string::npos ? text.size() + 1 : end + 1;
        line = trimCopy(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const char delimiter = line.find(';') != std::string::npos ? ';' : ',';
        const std::vector<std::string> fields = splitDelimited(line, delimiter);
        if (fields.empty() || fields[0].empty() || headerName(fields[0])) {
            continue;
        }
        const bool tia = delimiter == ';' && fields.size() >= 3;
        const std::size_t typeIndex = tia ? 2 : 1;
        if (fields.size() <= typeIndex) {
            error = "signal row needs a type: " + fields[0];
            return false;
        }
        ImportedSignal row;
        row.name = fields[0];
        if (!signalTypeFromLabel(fields[typeIndex], row.type)) {
            error = "unknown signal type: " + fields[typeIndex];
            return false;
        }
        const std::size_t valueIndex = tia ? 3 : 2;
        if (fields.size() > valueIndex && !fields[valueIndex].empty() && fields[valueIndex][0] != '%') {
            row.hasValue = true;
            if (row.type == SignalType::Bool) {
                const std::string value = fields[valueIndex];
                row.flag = value == "1" || value == "true" || value == "TRUE" || value == "True";
            } else if (row.type == SignalType::Int) {
                row.integer = std::stoll(fields[valueIndex]);
            } else {
                row.real = std::stod(fields[valueIndex]);
            }
        }
        rows.push_back(std::move(row));
    }
    if (rows.empty()) {
        error = "signal table was empty";
        return false;
    }
    error.clear();
    return true;
}

void applyImportedSignals(Scene& scene, const std::vector<ImportedSignal>& rows) {
    for (const ImportedSignal& row : rows) {
        Signal& signal = scene.upsertSignal(row.name, row.type);
        if (!row.hasValue) {
            continue;
        }
        if (row.type == SignalType::Bool) {
            signal.flag = row.flag;
        } else if (row.type == SignalType::Int) {
            signal.integer = row.integer;
        } else {
            signal.real = row.real;
        }
    }
}

}  // namespace sim
