#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

struct ZipEntry {
    std::string name;
    std::vector<std::uint8_t> data;
};

// One central-directory record. Data is not loaded until readEntry.
struct ZipIndexEntry {
    std::string name;
    std::uint16_t method = 0;
    std::uint32_t crc = 0;
    std::uint32_t compressed = 0;
    std::uint32_t uncompressed = 0;
    std::uint32_t localOffset = 0;
};

// Writer stores entries uncompressed. The reader accepts stored entries and
// deflate (method 8). Encrypted entries are refused.
class ZipArchive {
public:
    static bool write(const std::string& path, const std::vector<ZipEntry>& entries, std::string& error);
    static bool read(const std::string& path, std::vector<ZipEntry>& entries, std::string& error);
    static bool index(const std::string& path, std::vector<ZipIndexEntry>& entries, std::string& error);
    static bool readEntry(const std::string& path, const ZipIndexEntry& entry, std::vector<std::uint8_t>& data,
                          std::string& error);
};

}  // namespace sim
