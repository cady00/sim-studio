#include "OpcUaCrypto.hpp"

#include "sim/io/OpcUa.hpp"

#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/pem.h>
#include <openssl/rand.h>
#include <openssl/x509.h>

#include <cstring>
#include <fstream>

namespace sim {
namespace {

constexpr const char* kPolicy = "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256";

struct Pkey {
    EVP_PKEY* key = nullptr;
    ~Pkey() { EVP_PKEY_free(key); }
};

struct Bio {
    BIO* bio = nullptr;
    ~Bio() { BIO_free(bio); }
};

struct Md {
    EVP_MD_CTX* ctx = nullptr;
    ~Md() { EVP_MD_CTX_free(ctx); }
};

struct Cipher {
    EVP_CIPHER_CTX* ctx = nullptr;
    ~Cipher() { EVP_CIPHER_CTX_free(ctx); }
};

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xffu));
    }
}

std::uint32_t readU32(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) | (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16) | (static_cast<std::uint32_t>(data[3]) << 24);
}

void appendString(std::vector<std::uint8_t>& out, const char* text) {
    const auto length = static_cast<std::uint32_t>(std::strlen(text));
    appendU32(out, length);
    out.insert(out.end(), text, text + length);
}

void appendBytes(std::vector<std::uint8_t>& out, const std::vector<std::uint8_t>& bytes) {
    appendU32(out, static_cast<std::uint32_t>(bytes.size()));
    out.insert(out.end(), bytes.begin(), bytes.end());
}

bool readString(const std::uint8_t* data, std::size_t size, std::size_t& cursor, std::vector<std::uint8_t>& out) {
    if (cursor + 4 > size) {
        return false;
    }
    const auto length = static_cast<std::int32_t>(readU32(data + cursor));
    cursor += 4;
    if (length < 0) {
        out.clear();
        return true;
    }
    if (cursor + static_cast<std::size_t>(length) > size) {
        return false;
    }
    out.assign(data + cursor, data + cursor + static_cast<std::size_t>(length));
    cursor += static_cast<std::size_t>(length);
    return true;
}

Pkey privateKey(const std::string& pem, std::string& error) {
    Pkey out;
    Bio bio;
    bio.bio = BIO_new_mem_buf(pem.data(), static_cast<int>(pem.size()));
    if (bio.bio == nullptr) {
        error = "key buffer failed";
        return out;
    }
    out.key = PEM_read_bio_PrivateKey(bio.bio, nullptr, nullptr, nullptr);
    if (out.key == nullptr) {
        error = "private key rejected";
    }
    return out;
}

Pkey publicKey(const std::vector<std::uint8_t>& der, std::string& error) {
    Pkey out;
    const std::uint8_t* cursor = der.data();
    X509* cert = d2i_X509(nullptr, &cursor, static_cast<long>(der.size()));
    if (cert == nullptr) {
        error = "certificate rejected";
        return out;
    }
    out.key = X509_get_pubkey(cert);
    X509_free(cert);
    if (out.key == nullptr) {
        error = "certificate has no key";
    }
    return out;
}

int rsaBytes(EVP_PKEY* key) { return EVP_PKEY_get_size(key); }

std::vector<std::uint8_t> hmacSha256(const std::vector<std::uint8_t>& key, const std::uint8_t* data, std::size_t size) {
    std::vector<std::uint8_t> out(32);
    unsigned int length = 32;
    HMAC(EVP_sha256(), key.data(), static_cast<int>(key.size()), data, size, out.data(), &length);
    out.resize(length);
    return out;
}

bool rsaSign(EVP_PKEY* key, const std::uint8_t* data, std::size_t size, std::vector<std::uint8_t>& signature, std::string& error) {
    Md md;
    md.ctx = EVP_MD_CTX_new();
    if (md.ctx == nullptr || EVP_DigestSignInit(md.ctx, nullptr, EVP_sha256(), nullptr, key) != 1) {
        error = "sign init failed";
        return false;
    }
    if (EVP_DigestSignUpdate(md.ctx, data, size) != 1) {
        error = "sign update failed";
        return false;
    }
    std::size_t length = 0;
    if (EVP_DigestSignFinal(md.ctx, nullptr, &length) != 1) {
        error = "sign length failed";
        return false;
    }
    signature.resize(length);
    if (EVP_DigestSignFinal(md.ctx, signature.data(), &length) != 1) {
        error = "sign failed";
        return false;
    }
    signature.resize(length);
    return true;
}

bool rsaVerify(EVP_PKEY* key, const std::uint8_t* data, std::size_t size, const std::uint8_t* signature, std::size_t signatureSize,
               std::string& error) {
    Md md;
    md.ctx = EVP_MD_CTX_new();
    if (md.ctx == nullptr || EVP_DigestVerifyInit(md.ctx, nullptr, EVP_sha256(), nullptr, key) != 1 ||
        EVP_DigestVerifyUpdate(md.ctx, data, size) != 1 ||
        EVP_DigestVerifyFinal(md.ctx, signature, signatureSize) != 1) {
        error = "signature rejected";
        return false;
    }
    return true;
}

bool rsaEncrypt(EVP_PKEY* key, const std::uint8_t* data, std::size_t size, std::vector<std::uint8_t>& out, std::string& error) {
    EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new(key, nullptr);
    if (ctx == nullptr || EVP_PKEY_encrypt_init(ctx) != 1 || EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_OAEP_PADDING) != 1 ||
        EVP_PKEY_CTX_set_rsa_oaep_md(ctx, EVP_sha1()) != 1 || EVP_PKEY_CTX_set_rsa_mgf1_md(ctx, EVP_sha1()) != 1) {
        EVP_PKEY_CTX_free(ctx);
        error = "encrypt init failed";
        return false;
    }
    const int block = rsaBytes(key) - 42;
    if (block <= 0 || size % static_cast<std::size_t>(block) != 0) {
        EVP_PKEY_CTX_free(ctx);
        error = "encrypt size rejected";
        return false;
    }
    out.clear();
    for (std::size_t offset = 0; offset < size; offset += static_cast<std::size_t>(block)) {
        std::size_t length = 0;
        if (EVP_PKEY_encrypt(ctx, nullptr, &length, data + offset, static_cast<std::size_t>(block)) != 1) {
            EVP_PKEY_CTX_free(ctx);
            error = "encrypt length failed";
            return false;
        }
        const std::size_t at = out.size();
        out.resize(at + length);
        if (EVP_PKEY_encrypt(ctx, out.data() + at, &length, data + offset, static_cast<std::size_t>(block)) != 1) {
            EVP_PKEY_CTX_free(ctx);
            error = "encrypt failed";
            return false;
        }
        out.resize(at + length);
    }
    EVP_PKEY_CTX_free(ctx);
    return true;
}

bool rsaDecrypt(EVP_PKEY* key, const std::uint8_t* data, std::size_t size, std::vector<std::uint8_t>& out, std::string& error) {
    EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new(key, nullptr);
    if (ctx == nullptr || EVP_PKEY_decrypt_init(ctx) != 1 || EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_OAEP_PADDING) != 1 ||
        EVP_PKEY_CTX_set_rsa_oaep_md(ctx, EVP_sha1()) != 1 || EVP_PKEY_CTX_set_rsa_mgf1_md(ctx, EVP_sha1()) != 1) {
        EVP_PKEY_CTX_free(ctx);
        error = "decrypt init failed";
        return false;
    }
    const int block = rsaBytes(key);
    if (block <= 0 || size % static_cast<std::size_t>(block) != 0) {
        EVP_PKEY_CTX_free(ctx);
        error = "decrypt size rejected";
        return false;
    }
    out.clear();
    for (std::size_t offset = 0; offset < size; offset += static_cast<std::size_t>(block)) {
        std::size_t length = 0;
        if (EVP_PKEY_decrypt(ctx, nullptr, &length, data + offset, static_cast<std::size_t>(block)) != 1) {
            EVP_PKEY_CTX_free(ctx);
            error = "decrypt length failed";
            return false;
        }
        const std::size_t at = out.size();
        out.resize(at + length);
        if (EVP_PKEY_decrypt(ctx, out.data() + at, &length, data + offset, static_cast<std::size_t>(block)) != 1) {
            EVP_PKEY_CTX_free(ctx);
            error = "decrypt failed";
            return false;
        }
        out.resize(at + length);
    }
    EVP_PKEY_CTX_free(ctx);
    return true;
}

bool aesCrypt(bool encrypt, const std::vector<std::uint8_t>& key, const std::uint8_t* iv, const std::uint8_t* data, std::size_t size,
              std::vector<std::uint8_t>& out, std::string& error) {
    Cipher cipher;
    cipher.ctx = EVP_CIPHER_CTX_new();
    if (cipher.ctx == nullptr) {
        error = "cipher alloc failed";
        return false;
    }
    const int ready = encrypt ? EVP_EncryptInit_ex(cipher.ctx, EVP_aes_256_cbc(), nullptr, key.data(), iv)
                              : EVP_DecryptInit_ex(cipher.ctx, EVP_aes_256_cbc(), nullptr, key.data(), iv);
    if (ready != 1) {
        error = "cipher init failed";
        return false;
    }
    EVP_CIPHER_CTX_set_padding(cipher.ctx, 0);
    out.resize(size + 16);
    int written = 0;
    const int updated = encrypt ? EVP_EncryptUpdate(cipher.ctx, out.data(), &written, data, static_cast<int>(size))
                                : EVP_DecryptUpdate(cipher.ctx, out.data(), &written, data, static_cast<int>(size));
    if (updated != 1) {
        error = "cipher update failed";
        return false;
    }
    int finalSize = 0;
    const int finished = encrypt ? EVP_EncryptFinal_ex(cipher.ctx, out.data() + written, &finalSize)
                                 : EVP_DecryptFinal_ex(cipher.ctx, out.data() + written, &finalSize);
    if (finished != 1) {
        error = "cipher final failed";
        return false;
    }
    out.resize(static_cast<std::size_t>(written + finalSize));
    return true;
}

bool unpad(std::vector<std::uint8_t>& signedRegion, std::string& error) {
    if (signedRegion.size() < 9) {
        error = "padding missing";
        return false;
    }
    const unsigned padValue = signedRegion.back();
    const std::size_t padCount = static_cast<std::size_t>(padValue) + 1;
    if (padCount > signedRegion.size() - 8) {
        error = "padding rejected";
        return false;
    }
    for (std::size_t i = 0; i < padCount; ++i) {
        if (signedRegion[signedRegion.size() - 1 - i] != padValue) {
            error = "padding mismatch";
            return false;
        }
    }
    signedRegion.resize(signedRegion.size() - padCount);
    return true;
}

void addPadding(std::vector<std::uint8_t>& plain, std::size_t signatureSize, std::size_t block) {
    const std::size_t withSignature = plain.size() + signatureSize;
    const std::size_t padCount = block - (withSignature % block);
    const auto padValue = static_cast<std::uint8_t>(padCount - 1);
    plain.insert(plain.end(), padCount, padValue);
}

bool splitSigned(const std::vector<std::uint8_t>& plain, std::size_t signatureSize, bool encrypted, UaOpened& out, std::string& error) {
    if (plain.size() < signatureSize + 8) {
        error = "secure body short";
        return false;
    }
    const std::size_t signedSize = plain.size() - signatureSize;
    std::vector<std::uint8_t> signedRegion(plain.begin(), plain.begin() + static_cast<std::ptrdiff_t>(signedSize));
    if (encrypted && !unpad(signedRegion, error)) {
        return false;
    }
    if (signedRegion.size() < 8) {
        error = "sequence missing";
        return false;
    }
    out.sequence = readU32(signedRegion.data());
    out.requestId = readU32(signedRegion.data() + 4);
    out.body.assign(signedRegion.begin() + 8, signedRegion.end());
    return true;
}

void finishHeader(std::vector<std::uint8_t>& message, char a, char b, char c) {
    message[0] = static_cast<std::uint8_t>(a);
    message[1] = static_cast<std::uint8_t>(b);
    message[2] = static_cast<std::uint8_t>(c);
    message[3] = 'F';
    const auto size = static_cast<std::uint32_t>(message.size());
    message[4] = static_cast<std::uint8_t>(size & 0xffu);
    message[5] = static_cast<std::uint8_t>((size >> 8) & 0xffu);
    message[6] = static_cast<std::uint8_t>((size >> 16) & 0xffu);
    message[7] = static_cast<std::uint8_t>((size >> 24) & 0xffu);
}

}  // namespace

bool uaSelfSigned(const std::string& commonName, std::vector<std::uint8_t>& der, std::string& keyPem, std::string& error) {
    EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, nullptr);
    if (ctx == nullptr || EVP_PKEY_keygen_init(ctx) != 1 || EVP_PKEY_CTX_set_rsa_keygen_bits(ctx, 2048) != 1) {
        EVP_PKEY_CTX_free(ctx);
        error = "keygen init failed";
        return false;
    }
    EVP_PKEY* pkey = nullptr;
    if (EVP_PKEY_keygen(ctx, &pkey) != 1) {
        EVP_PKEY_CTX_free(ctx);
        error = "keygen failed";
        return false;
    }
    EVP_PKEY_CTX_free(ctx);
    X509* cert = X509_new();
    if (cert == nullptr) {
        EVP_PKEY_free(pkey);
        error = "certificate alloc failed";
        return false;
    }
    ASN1_INTEGER_set(X509_get_serialNumber(cert), 1);
    X509_gmtime_adj(X509_getm_notBefore(cert), 0);
    X509_gmtime_adj(X509_getm_notAfter(cert), 60 * 60 * 24 * 365);
    X509_set_pubkey(cert, pkey);
    X509_NAME* name = X509_get_subject_name(cert);
    X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC, reinterpret_cast<const unsigned char*>(commonName.c_str()), -1, -1, 0);
    X509_set_issuer_name(cert, name);
    if (X509_sign(cert, pkey, EVP_sha256()) == 0) {
        X509_free(cert);
        EVP_PKEY_free(pkey);
        error = "certificate sign failed";
        return false;
    }
    unsigned char* encoded = nullptr;
    const int length = i2d_X509(cert, &encoded);
    X509_free(cert);
    if (length <= 0 || encoded == nullptr) {
        EVP_PKEY_free(pkey);
        error = "certificate encode failed";
        return false;
    }
    der.assign(encoded, encoded + length);
    OPENSSL_free(encoded);
    Bio bio;
    bio.bio = BIO_new(BIO_s_mem());
    if (bio.bio == nullptr || PEM_write_bio_PrivateKey(bio.bio, pkey, nullptr, nullptr, 0, nullptr, nullptr) != 1) {
        EVP_PKEY_free(pkey);
        error = "key encode failed";
        return false;
    }
    EVP_PKEY_free(pkey);
    BUF_MEM* memory = nullptr;
    BIO_get_mem_ptr(bio.bio, &memory);
    if (memory == nullptr || memory->data == nullptr) {
        error = "key buffer empty";
        return false;
    }
    keyPem.assign(memory->data, memory->length);
    error.clear();
    return true;
}

bool uaSavePem(const std::vector<std::uint8_t>& der, const std::string& keyPem, const std::string& certificatePath,
               const std::string& keyPath, std::string& error) {
    const std::uint8_t* cursor = der.data();
    X509* cert = d2i_X509(nullptr, &cursor, static_cast<long>(der.size()));
    if (cert == nullptr) {
        error = "certificate rejected";
        return false;
    }
    Bio bio;
    bio.bio = BIO_new_file(certificatePath.c_str(), "w");
    if (bio.bio == nullptr || PEM_write_bio_X509(bio.bio, cert) != 1) {
        X509_free(cert);
        error = "certificate write failed";
        return false;
    }
    X509_free(cert);
    std::ofstream key(keyPath, std::ios::binary);
    if (!key) {
        error = "key write failed";
        return false;
    }
    key << keyPem;
    error.clear();
    return true;
}

bool uaLoadPem(const std::string& certificatePath, const std::string& keyPath, std::vector<std::uint8_t>& der, std::string& keyPem,
               std::string& error) {
    Bio bio;
    bio.bio = BIO_new_file(certificatePath.c_str(), "r");
    if (bio.bio == nullptr) {
        error = "certificate read failed";
        return false;
    }
    X509* cert = PEM_read_bio_X509(bio.bio, nullptr, nullptr, nullptr);
    if (cert == nullptr) {
        error = "certificate parse failed";
        return false;
    }
    unsigned char* encoded = nullptr;
    const int length = i2d_X509(cert, &encoded);
    X509_free(cert);
    if (length <= 0 || encoded == nullptr) {
        error = "certificate encode failed";
        return false;
    }
    der.assign(encoded, encoded + length);
    OPENSSL_free(encoded);
    std::ifstream key(keyPath, std::ios::binary);
    if (!key) {
        error = "key read failed";
        return false;
    }
    keyPem.assign(std::istreambuf_iterator<char>(key), std::istreambuf_iterator<char>());
    if (keyPem.empty()) {
        error = "key empty";
        return false;
    }
    error.clear();
    return true;
}

std::vector<std::uint8_t> uaSha1(const std::vector<std::uint8_t>& data) {
    std::vector<std::uint8_t> out(20);
    unsigned int length = 20;
    EVP_Digest(data.data(), data.size(), out.data(), &length, EVP_sha1(), nullptr);
    out.resize(length);
    return out;
}

std::vector<std::uint8_t> uaRandom(std::size_t count) {
    std::vector<std::uint8_t> out(count);
    if (count > 0) {
        RAND_bytes(out.data(), static_cast<int>(count));
    }
    return out;
}

bool uaDeriveKeys(const std::vector<std::uint8_t>& secretNonce, const std::vector<std::uint8_t>& seedNonce, UaSymKeys& out) {
    if (secretNonce.empty() || seedNonce.empty()) {
        return false;
    }
    std::vector<std::uint8_t> material;
    std::vector<std::uint8_t> mixed = hmacSha256(secretNonce, seedNonce.data(), seedNonce.size());
    while (material.size() < 80) {
        std::vector<std::uint8_t> block = mixed;
        block.insert(block.end(), seedNonce.begin(), seedNonce.end());
        const std::vector<std::uint8_t> piece = hmacSha256(secretNonce, block.data(), block.size());
        material.insert(material.end(), piece.begin(), piece.end());
        mixed = hmacSha256(secretNonce, mixed.data(), mixed.size());
    }
    out.sign.assign(material.begin(), material.begin() + 32);
    out.enc.assign(material.begin() + 32, material.begin() + 64);
    out.iv.assign(material.begin() + 64, material.begin() + 80);
    return out.sign.size() == 32 && out.enc.size() == 32 && out.iv.size() == 16;
}

bool uaSealAsymmetric(int mode, const std::vector<std::uint8_t>& senderDer, const std::string& senderKeyPem,
                      const std::vector<std::uint8_t>& receiverDer, std::uint32_t channelId, std::uint32_t sequence,
                      std::uint32_t requestId, const std::uint8_t* body, std::size_t bodySize, std::vector<std::uint8_t>& message,
                      std::string& error) {
    Pkey signer = privateKey(senderKeyPem, error);
    Pkey encryptTo = publicKey(receiverDer, error);
    if (signer.key == nullptr || encryptTo.key == nullptr) {
        return false;
    }
    const int rsaSize = rsaBytes(encryptTo.key);
    const int plainBlock = rsaSize - 42;
    if (rsaSize <= 42 || plainBlock <= 0) {
        error = "rsa size rejected";
        return false;
    }
    std::vector<std::uint8_t> plain;
    appendU32(plain, sequence);
    appendU32(plain, requestId);
    plain.insert(plain.end(), body, body + bodySize);
    const bool encrypt = mode == 3;
    if (encrypt) {
        addPadding(plain, static_cast<std::size_t>(rsaSize), static_cast<std::size_t>(plainBlock));
    }
    std::vector<std::uint8_t> signature;
    if (!rsaSign(signer.key, plain.data(), plain.size(), signature, error)) {
        return false;
    }
    plain.insert(plain.end(), signature.begin(), signature.end());
    std::vector<std::uint8_t> secured = plain;
    if (encrypt && !rsaEncrypt(encryptTo.key, plain.data(), plain.size(), secured, error)) {
        return false;
    }
    message.assign(8, 0);
    appendU32(message, channelId);
    appendString(message, kPolicy);
    appendBytes(message, senderDer);
    appendBytes(message, uaSha1(receiverDer));
    message.insert(message.end(), secured.begin(), secured.end());
    finishHeader(message, 'O', 'P', 'N');
    error.clear();
    return true;
}

bool uaOpenAsymmetric(int mode, const std::vector<std::uint8_t>& receiverDer, const std::string& receiverKeyPem,
                      const std::uint8_t* message, std::size_t messageSize, UaOpened& out, std::string& error) {
    if (messageSize < 16 || message[0] != 'O') {
        error = "open header rejected";
        return false;
    }
    const std::uint32_t size = readU32(message + 4);
    if (size != messageSize) {
        error = "open size rejected";
        return false;
    }
    std::size_t cursor = 8;
    if (cursor + 4 > messageSize) {
        error = "open channel missing";
        return false;
    }
    out.channelId = readU32(message + cursor);
    cursor += 4;
    std::vector<std::uint8_t> policy;
    std::vector<std::uint8_t> thumb;
    if (!readString(message, messageSize, cursor, policy) || !readString(message, messageSize, cursor, out.senderDer) ||
        !readString(message, messageSize, cursor, thumb)) {
        error = "open security header rejected";
        return false;
    }
    const std::string policyText(policy.begin(), policy.end());
    if (policyText.find("Basic256Sha256") == std::string::npos) {
        error = "security policy rejected";
        return false;
    }
    if (thumb != uaSha1(receiverDer)) {
        error = "receiver thumbprint rejected";
        return false;
    }
    Pkey receiver = privateKey(receiverKeyPem, error);
    Pkey sender = publicKey(out.senderDer, error);
    if (receiver.key == nullptr || sender.key == nullptr) {
        return false;
    }
    const int rsaSize = rsaBytes(receiver.key);
    std::vector<std::uint8_t> plain;
    const bool encrypt = mode == 3;
    if (encrypt) {
        if (!rsaDecrypt(receiver.key, message + cursor, messageSize - cursor, plain, error)) {
            return false;
        }
    } else {
        plain.assign(message + cursor, message + messageSize);
    }
    if (plain.size() < static_cast<std::size_t>(rsaSize) + 8) {
        error = "open body short";
        return false;
    }
    const std::size_t signedSize = plain.size() - static_cast<std::size_t>(rsaSize);
    if (!rsaVerify(sender.key, plain.data(), signedSize, plain.data() + signedSize, static_cast<std::size_t>(rsaSize), error)) {
        return false;
    }
    if (!splitSigned(plain, static_cast<std::size_t>(rsaSize), encrypt, out, error)) {
        return false;
    }
    error.clear();
    return true;
}

bool uaSealSymmetric(int mode, const UaSymKeys& keys, std::uint32_t channelId, std::uint32_t tokenId, std::uint32_t sequence,
                     std::uint32_t requestId, const std::uint8_t* body, std::size_t bodySize, std::vector<std::uint8_t>& message,
                     std::string& error) {
    if (keys.sign.size() != 32 || keys.enc.size() != 32) {
        error = "symmetric key missing";
        return false;
    }
    std::vector<std::uint8_t> plain;
    appendU32(plain, sequence);
    appendU32(plain, requestId);
    plain.insert(plain.end(), body, body + bodySize);
    const bool encrypt = mode == 3;
    if (encrypt) {
        addPadding(plain, 32, 16);
    }
    const std::vector<std::uint8_t> signature = hmacSha256(keys.sign, plain.data(), plain.size());
    plain.insert(plain.end(), signature.begin(), signature.end());
    std::vector<std::uint8_t> iv;
    std::vector<std::uint8_t> secured = plain;
    if (encrypt) {
        iv = uaRandom(16);
        if (!aesCrypt(true, keys.enc, iv.data(), plain.data(), plain.size(), secured, error)) {
            return false;
        }
    }
    message.assign(8, 0);
    appendU32(message, channelId);
    appendU32(message, tokenId);
    if (encrypt) {
        message.insert(message.end(), iv.begin(), iv.end());
    }
    message.insert(message.end(), secured.begin(), secured.end());
    finishHeader(message, 'M', 'S', 'G');
    error.clear();
    return true;
}

bool uaOpenSymmetric(int mode, const UaSymKeys& keys, const std::uint8_t* message, std::size_t messageSize, UaOpened& out,
                     std::string& error) {
    if (messageSize < 16 || message[0] != 'M') {
        error = "message header rejected";
        return false;
    }
    if (readU32(message + 4) != messageSize || keys.sign.size() != 32) {
        error = "message size rejected";
        return false;
    }
    out.channelId = readU32(message + 8);
    out.tokenId = readU32(message + 12);
    const bool encrypt = mode == 3;
    std::size_t cursor = 16;
    std::vector<std::uint8_t> plain;
    if (encrypt) {
        if (cursor + 16 > messageSize || keys.enc.size() != 32) {
            error = "iv missing";
            return false;
        }
        const std::uint8_t* iv = message + cursor;
        cursor += 16;
        if (!aesCrypt(false, keys.enc, iv, message + cursor, messageSize - cursor, plain, error)) {
            return false;
        }
    } else {
        plain.assign(message + cursor, message + messageSize);
    }
    if (plain.size() < 32 + 8) {
        error = "message body short";
        return false;
    }
    const std::size_t signedSize = plain.size() - 32;
    const std::vector<std::uint8_t> expect = hmacSha256(keys.sign, plain.data(), signedSize);
    if (expect.size() != 32 || std::memcmp(expect.data(), plain.data() + signedSize, 32) != 0) {
        error = "hmac rejected";
        return false;
    }
    if (!splitSigned(plain, 32, encrypt, out, error)) {
        return false;
    }
    error.clear();
    return true;
}

bool uaEncryptSecret(const std::vector<std::uint8_t>& certificateDer, const std::string& password,
                     const std::vector<std::uint8_t>& nonce, std::vector<std::uint8_t>& out, std::string& error) {
    Pkey key = publicKey(certificateDer, error);
    if (key.key == nullptr) {
        return false;
    }
    std::vector<std::uint8_t> plain;
    appendU32(plain, static_cast<std::uint32_t>(password.size()));
    plain.insert(plain.end(), password.begin(), password.end());
    plain.insert(plain.end(), nonce.begin(), nonce.end());
    const int block = rsaBytes(key.key) - 42;
    if (block <= 0 || plain.size() > static_cast<std::size_t>(block)) {
        error = "secret too long";
        return false;
    }
    plain.resize(static_cast<std::size_t>(block), 0);
    return rsaEncrypt(key.key, plain.data(), plain.size(), out, error);
}

bool OpcUaCertificate::createSelfSigned(const std::string& commonName, std::string& error) {
    return uaSelfSigned(commonName, der, keyPem, error);
}

bool OpcUaCertificate::savePem(const std::string& certificatePath, const std::string& keyPath, std::string& error) const {
    return uaSavePem(der, keyPem, certificatePath, keyPath, error);
}

bool OpcUaCertificate::loadPem(const std::string& certificatePath, const std::string& keyPath, std::string& error) {
    return uaLoadPem(certificatePath, keyPath, der, keyPem, error);
}

bool uaDecryptSecret(const std::string& keyPem, const std::vector<std::uint8_t>& secret, const std::vector<std::uint8_t>& nonce,
                     std::string& password, std::string& error) {
    Pkey key = privateKey(keyPem, error);
    if (key.key == nullptr) {
        return false;
    }
    std::vector<std::uint8_t> plain;
    if (!rsaDecrypt(key.key, secret.data(), secret.size(), plain, error)) {
        return false;
    }
    if (plain.size() < 4) {
        error = "secret short";
        return false;
    }
    const auto length = readU32(plain.data());
    if (4 + static_cast<std::size_t>(length) + nonce.size() > plain.size()) {
        error = "secret layout rejected";
        return false;
    }
    if (std::memcmp(plain.data() + 4 + length, nonce.data(), nonce.size()) != 0) {
        error = "secret nonce rejected";
        return false;
    }
    password.assign(reinterpret_cast<const char*>(plain.data() + 4), length);
    error.clear();
    return true;
}

}  // namespace sim
