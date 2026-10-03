#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

struct ZipEntry {
    std::string name;
    std::vector<std::uint8_t> data;
};

// Store-only (method 0) ZIP reader and writer. Enough for .simproj packages.
class ZipArchive {
public:
    static bool write(const std::string& path, const std::vector<ZipEntry>& entries, std::string& error);
    static bool read(const std::string& path, std::vector<ZipEntry>& entries, std::string& error);
};

}  // namespace sim
