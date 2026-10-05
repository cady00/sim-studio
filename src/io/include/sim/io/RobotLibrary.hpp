#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

// One Process Simulate component found under a local library directory.
// `path` is either a .cojt directory or a .cojt.zip. The library root itself
// is not stored here and is not written into the project.
struct RobotLibraryEntry {
    std::string name;
    std::string brand;
    std::string model;
    std::string path;
    // directory, zip, or nested-zip (a .cojt.zip stored inside another zip).
    std::string kind;
    std::string relative;
    bool preview = false;
    // Zip-internal name of preview.bmp. Empty when the bitmap is preview.bmp
    // in a .cojt directory, or when kind is nested-zip (resolved on read).
    std::string previewEntry;
    // Set for nested-zip: the outer zip, the inner zip entry, and the .cojt prefix inside it.
    std::string innerZipEntry;
    std::string innerCojtPrefix;
};

// Walk `root` for .cojt directories and .cojt.zip files, including a .cojt.zip
// nested one level inside another zip. Does not read JT payloads.
std::vector<RobotLibraryEntry> scanRobotLibrary(const std::string& root, std::string& error);

// preview.bmp bytes when the component has one. Empty and ok when it does not.
std::vector<std::uint8_t> robotLibraryPreview(const RobotLibraryEntry& entry, std::string& error);

// Copy or inflate the component into `scratch` and return the .cojt folder.
// A directory entry returns its own path and leaves `scratchUsed` empty.
bool materializeRobotComponent(const RobotLibraryEntry& entry, const std::string& scratch, std::string& folder,
                               bool& scratchUsed, std::string& error);

}  // namespace sim
