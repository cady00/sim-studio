#include "sim/io/ZipArchive.hpp"

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace sim {
namespace {

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

void appendU16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

std::uint16_t readU16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t readU32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

}  // namespace

bool ZipArchive::write(const std::string& path, const std::vector<ZipEntry>& entries, std::string& error) {
    std::vector<std::uint8_t> blob;
    struct Central {
        std::string name;
        std::uint32_t crc = 0;
        std::uint32_t size = 0;
        std::uint32_t offset = 0;
    };
    std::vector<Central> centrals;
    centrals.reserve(entries.size());

    for (const ZipEntry& entry : entries) {
        if (entry.name.empty() || entry.name.size() > 60000) {
            error = "invalid zip entry name";
            return false;
        }
        if (entry.data.size() > 0xFFFFFFFFu) {
            error = "zip entry is larger than 4 GiB";
            return false;
        }
        Central central;
        central.name = entry.name;
        central.crc = crc32(entry.data.data(), entry.data.size());
        central.size = static_cast<std::uint32_t>(entry.data.size());
        central.offset = static_cast<std::uint32_t>(blob.size());
        constexpr std::uint16_t kUtf8 = 1u << 11;
        appendU32(blob, 0x04034b50u);
        appendU16(blob, 20);
        appendU16(blob, kUtf8);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, central.crc);
        appendU32(blob, central.size);
        appendU32(blob, central.size);
        appendU16(blob, static_cast<std::uint16_t>(entry.name.size()));
        appendU16(blob, 0);
        blob.insert(blob.end(), entry.name.begin(), entry.name.end());
        blob.insert(blob.end(), entry.data.begin(), entry.data.end());
        centrals.push_back(std::move(central));
    }

    const auto centralOffset = static_cast<std::uint32_t>(blob.size());
    for (const Central& central : centrals) {
        constexpr std::uint16_t kUtf8 = 1u << 11;
        appendU32(blob, 0x02014b50u);
        appendU16(blob, 20);
        appendU16(blob, 20);
        appendU16(blob, kUtf8);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, central.crc);
        appendU32(blob, central.size);
        appendU32(blob, central.size);
        appendU16(blob, static_cast<std::uint16_t>(central.name.size()));
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, 0);
        appendU32(blob, central.offset);
        blob.insert(blob.end(), central.name.begin(), central.name.end());
    }
    const auto centralSize = static_cast<std::uint32_t>(blob.size() - centralOffset);
    appendU32(blob, 0x06054b50u);
    appendU16(blob, 0);
    appendU16(blob, 0);
    appendU16(blob, static_cast<std::uint16_t>(centrals.size()));
    appendU16(blob, static_cast<std::uint16_t>(centrals.size()));
    appendU32(blob, centralSize);
    appendU32(blob, centralOffset);
    appendU16(blob, 0);

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        error = "cannot write " + path;
        return false;
    }
    out.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
    if (!out) {
        error = "failed while writing " + path;
        return false;
    }
    error.clear();
    return true;
}

bool ZipArchive::read(const std::string& path, std::vector<ZipEntry>& entries, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    in.seekg(0, std::ios::end);
    const auto length = static_cast<std::uint64_t>(in.tellg());
    if (length < 22) {
        error = "file is too small to be a zip archive";
        return false;
    }
    const std::uint64_t scan = std::min<std::uint64_t>(length, 22u + 65535u);
    std::vector<std::uint8_t> tail(static_cast<std::size_t>(scan));
    in.seekg(static_cast<std::streamoff>(length - scan), std::ios::beg);
    in.read(reinterpret_cast<char*>(tail.data()), static_cast<std::streamsize>(tail.size()));
    int eocd = -1;
    for (int i = static_cast<int>(tail.size()) - 22; i >= 0; --i) {
        if (readU32(tail.data() + i) == 0x06054b50u) {
            eocd = i;
            break;
        }
    }
    if (eocd < 0) {
        error = "zip end-of-central-directory record not found";
        return false;
    }
    const std::uint8_t* e = tail.data() + eocd;
    const std::uint16_t count = readU16(e + 10);
    const std::uint32_t centralSize = readU32(e + 12);
    const std::uint32_t centralOffset = readU32(e + 16);
    if (static_cast<std::uint64_t>(centralOffset) + centralSize > length) {
        error = "zip central directory is out of range";
        return false;
    }
    std::vector<std::uint8_t> central(centralSize);
    in.seekg(static_cast<std::streamoff>(centralOffset), std::ios::beg);
    in.read(reinterpret_cast<char*>(central.data()), static_cast<std::streamsize>(central.size()));
    entries.clear();
    std::size_t cursor = 0;
    for (std::uint16_t n = 0; n < count; ++n) {
        if (cursor + 46 > central.size() || readU32(central.data() + cursor) != 0x02014b50u) {
            error = "truncated zip central directory";
            return false;
        }
        const std::uint16_t method = readU16(central.data() + cursor + 10);
        const std::uint32_t crc = readU32(central.data() + cursor + 16);
        const std::uint32_t compressed = readU32(central.data() + cursor + 20);
        const std::uint32_t uncompressed = readU32(central.data() + cursor + 24);
        const std::uint16_t nameLen = readU16(central.data() + cursor + 28);
        const std::uint16_t extraLen = readU16(central.data() + cursor + 30);
        const std::uint16_t commentLen = readU16(central.data() + cursor + 32);
        const std::uint32_t localOffset = readU32(central.data() + cursor + 42);
        if (cursor + 46 + nameLen > central.size()) {
            error = "truncated zip entry name";
            return false;
        }
        ZipEntry entry;
        entry.name.assign(reinterpret_cast<const char*>(central.data() + cursor + 46), nameLen);
        cursor += 46 + nameLen + extraLen + commentLen;
        if (method != 0) {
            error = "compressed zip entries are not supported (" + entry.name + ")";
            return false;
        }
        if (compressed != uncompressed) {
            error = "stored zip entry has inconsistent sizes (" + entry.name + ")";
            return false;
        }
        std::vector<std::uint8_t> localHeader(30);
        in.seekg(static_cast<std::streamoff>(localOffset), std::ios::beg);
        in.read(reinterpret_cast<char*>(localHeader.data()), 30);
        if (!in || readU32(localHeader.data()) != 0x04034b50u) {
            error = "missing local file header for " + entry.name;
            return false;
        }
        const std::uint16_t localName = readU16(localHeader.data() + 26);
        const std::uint16_t localExtra = readU16(localHeader.data() + 28);
        const auto dataOffset = static_cast<std::uint64_t>(localOffset) + 30u + localName + localExtra;
        if (dataOffset + uncompressed > length) {
            error = "zip entry data is out of range (" + entry.name + ")";
            return false;
        }
        entry.data.resize(uncompressed);
        in.seekg(static_cast<std::streamoff>(dataOffset), std::ios::beg);
        if (uncompressed > 0) {
            in.read(reinterpret_cast<char*>(entry.data.data()), static_cast<std::streamsize>(uncompressed));
        }
        if (!in) {
            error = "failed reading " + entry.name;
            return false;
        }
        if (crc32(entry.data.data(), entry.data.size()) != crc) {
            error = "crc mismatch for " + entry.name;
            return false;
        }
        entries.push_back(std::move(entry));
    }
    error.clear();
    return true;
}

}  // namespace sim
