#include "sim/io/RobotLibrary.hpp"

#include "sim/io/ZipArchive.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace sim {
namespace {

std::string lowerCopy(std::string text) {
    for (char& letter : text) {
        letter = static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
    }
    return text;
}

bool endsWith(const std::string& text, const std::string& suffix) {
    return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string fileNameOf(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

std::vector<std::string> splitZipPath(const std::string& name) {
    std::vector<std::string> parts;
    std::string current;
    for (char letter : name) {
        if (letter == '/' || letter == '\\') {
            if (!current.empty()) {
                parts.push_back(current);
                current.clear();
            }
        } else {
            current.push_back(letter);
        }
    }
    if (!current.empty()) {
        parts.push_back(current);
    }
    return parts;
}

std::string stripCojt(std::string name) {
    const std::string lower = lowerCopy(name);
    if (endsWith(lower, ".cojt.zip")) {
        return name.substr(0, name.size() - 9);
    }
    if (endsWith(lower, ".cojt")) {
        return name.substr(0, name.size() - 5);
    }
    return name;
}

std::string canonicalBrand(const std::string& token) {
    const std::string lower = lowerCopy(token);
    if (lower == "kuka" || lower.rfind("kuka", 0) == 0) {
        return "KUKA";
    }
    if (lower == "abb" || lower.rfind("abb", 0) == 0) {
        return "ABB";
    }
    if (lower == "fanuc" || lower.rfind("fanuc", 0) == 0) {
        return "FANUC";
    }
    if (lower == "yaskawa" || lower == "motoman" || lower.rfind("yaskawa", 0) == 0) {
        return "YASKAWA";
    }
    if (lower == "kawasaki") {
        return "KAWASAKI";
    }
    if (lower == "nachi") {
        return "NACHI";
    }
    if (lower == "comau") {
        return "COMAU";
    }
    if (lower == "staubli" || lower == "stäubli") {
        return "STAUBLI";
    }
    if (lower == "epson") {
        return "EPSON";
    }
    if (lower == "hyundai") {
        return "HYUNDAI";
    }
    if (lower == "panasonic") {
        return "PANASONIC";
    }
    if (lower == "universal" || lower == "ur") {
        return "UNIVERSAL";
    }
    return {};
}

std::string brandFromName(const std::string& name) {
    const std::string lower = lowerCopy(name);
    if (lower.rfind("kr", 0) == 0) {
        return "KUKA";
    }
    if (lower.rfind("irb", 0) == 0) {
        return "ABB";
    }
    if (lower.rfind("r2000", 0) == 0 || lower.rfind("r-2000", 0) == 0 || lower.rfind("lrmate", 0) == 0 ||
        lower.rfind("lr-mate", 0) == 0 || lower.rfind("m-", 0) == 0) {
        return "FANUC";
    }
    if (lower.rfind("gp", 0) == 0 || lower.rfind("mh", 0) == 0 || lower.rfind("motoman", 0) == 0) {
        return "YASKAWA";
    }
    return {};
}

std::string chooseBrand(const std::vector<std::string>& folders, const std::string& name) {
    for (const std::string& folder : folders) {
        if (endsWith(lowerCopy(folder), ".cojt") || endsWith(lowerCopy(folder), ".zip")) {
            continue;
        }
        if (const std::string known = canonicalBrand(folder); !known.empty()) {
            return known;
        }
    }
    if (const std::string fromName = brandFromName(name); !fromName.empty()) {
        return fromName;
    }
    for (const std::string& folder : folders) {
        if (!folder.empty() && !endsWith(lowerCopy(folder), ".cojt") && !endsWith(lowerCopy(folder), ".zip")) {
            return folder;
        }
    }
    return "其他";
}

bool isPreviewName(const std::string& name) { return lowerCopy(fileNameOf(name)) == "preview.bmp"; }

bool safeRelative(const std::filesystem::path& root, const std::filesystem::path& path, std::string& relative) {
    std::error_code ec;
    const std::filesystem::path from = std::filesystem::relative(path, root, ec);
    if (ec) {
        relative = path.filename().string();
        return false;
    }
    relative = from.generic_string();
    return relative.find("..") == std::string::npos;
}

void addDirectory(std::vector<RobotLibraryEntry>& out, const std::filesystem::path& root, const std::filesystem::path& folder) {
    RobotLibraryEntry entry;
    entry.name = stripCojt(folder.filename().string());
    entry.model = entry.name;
    entry.path = folder.string();
    entry.kind = "directory";
    std::string relative;
    safeRelative(root, folder, relative);
    entry.relative = relative;
    const auto parts = splitZipPath(relative);
    std::vector<std::string> folders = parts;
    if (!folders.empty()) {
        folders.pop_back();
    }
    entry.brand = chooseBrand(folders, entry.name);
    std::error_code ec;
    const std::filesystem::path preview = folder / "preview.bmp";
    if (std::filesystem::is_regular_file(preview, ec)) {
        entry.preview = true;
    }
    out.push_back(std::move(entry));
}

bool entryUnderPrefix(const std::string& name, const std::string& prefix) {
    return name == prefix || name.rfind(prefix, 0) == 0;
}

void addZipComponents(std::vector<RobotLibraryEntry>& out, const std::filesystem::path& root, const std::filesystem::path& zip,
                      const std::vector<ZipIndexEntry>& indexed, const std::string& kind, const std::string& innerZipEntry) {
    std::vector<std::string> prefixes;
    std::vector<std::string> nestedZips;
    for (const ZipIndexEntry& item : indexed) {
        const std::string lower = lowerCopy(item.name);
        if (endsWith(lower, ".cojt.zip") && !item.name.empty() && item.name.back() != '/') {
            nestedZips.push_back(item.name);
        }
        const auto parts = splitZipPath(item.name);
        for (std::size_t i = 0; i < parts.size(); ++i) {
            if (!endsWith(lowerCopy(parts[i]), ".cojt")) {
                continue;
            }
            std::string prefix;
            for (std::size_t j = 0; j <= i; ++j) {
                if (!prefix.empty()) {
                    prefix += '/';
                }
                prefix += parts[j];
            }
            prefix += '/';
            if (std::find(prefixes.begin(), prefixes.end(), prefix) == prefixes.end()) {
                prefixes.push_back(prefix);
            }
        }
    }
    std::string zipRelative;
    safeRelative(root, zip, zipRelative);
    const auto zipFolders = splitZipPath(zipRelative);
    for (const std::string& prefix : prefixes) {
        RobotLibraryEntry entry;
        const auto parts = splitZipPath(prefix);
        entry.name = stripCojt(parts.back());
        entry.model = entry.name;
        entry.path = zip.string();
        entry.kind = kind;
        entry.innerZipEntry = innerZipEntry;
        entry.innerCojtPrefix = prefix;
        entry.relative = zipRelative;
        std::vector<std::string> folders = zipFolders;
        if (!folders.empty()) {
            folders.pop_back();
        }
        for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
            folders.push_back(parts[i]);
        }
        entry.brand = chooseBrand(folders, entry.name);
        for (const ZipIndexEntry& item : indexed) {
            if (entryUnderPrefix(item.name, prefix) && isPreviewName(item.name)) {
                entry.preview = true;
                entry.previewEntry = item.name;
                break;
            }
        }
        out.push_back(std::move(entry));
    }
    if (kind != "zip") {
        return;
    }
    for (const std::string& nested : nestedZips) {
        std::string error;
        std::vector<std::uint8_t> bytes;
        ZipIndexEntry found;
        bool have = false;
        for (const ZipIndexEntry& item : indexed) {
            if (item.name == nested) {
                found = item;
                have = true;
                break;
            }
        }
        if (!have || !ZipArchive::readEntry(zip.string(), found, bytes, error) || bytes.empty()) {
            continue;
        }
        const std::filesystem::path temp = std::filesystem::temp_directory_path() / "ondot-cojt-index.zip";
        {
            std::ofstream outFile(temp, std::ios::binary);
            outFile.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            if (!outFile) {
                continue;
            }
        }
        std::vector<ZipIndexEntry> inner;
        if (!ZipArchive::index(temp.string(), inner, error)) {
            std::error_code ec;
            std::filesystem::remove(temp, ec);
            continue;
        }
        const std::size_t before = out.size();
        addZipComponents(out, root, zip, inner, "nested-zip", nested);
        for (std::size_t i = before; i < out.size(); ++i) {
            out[i].path = zip.string();
            out[i].kind = "nested-zip";
            out[i].innerZipEntry = nested;
            if (out[i].relative.empty()) {
                out[i].relative = zipRelative;
            }
            auto folders = splitZipPath(zipRelative);
            if (!folders.empty()) {
                folders.pop_back();
            }
            const std::vector<std::string> nestedParts = splitZipPath(nested);
            folders.insert(folders.end(), nestedParts.begin(), nestedParts.end());
            if (!folders.empty()) {
                folders.pop_back();
            }
            const auto innerParts = splitZipPath(out[i].innerCojtPrefix);
            for (std::size_t p = 0; p + 1 < innerParts.size(); ++p) {
                folders.push_back(innerParts[p]);
            }
            out[i].brand = chooseBrand(folders, out[i].name);
        }
        std::error_code ec;
        std::filesystem::remove(temp, ec);
    }
}

bool writeFile(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes, std::string& error) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        error = "cannot write " + path.string();
        return false;
    }
    if (!bytes.empty()) {
        out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }
    if (!out) {
        error = "failed while writing " + path.string();
        return false;
    }
    return true;
}

bool extractPrefix(const std::string& zipPath, const std::vector<ZipIndexEntry>& indexed, const std::string& prefix,
                   const std::filesystem::path& dest, std::string& error) {
    for (const ZipIndexEntry& item : indexed) {
        if (!entryUnderPrefix(item.name, prefix)) {
            continue;
        }
        std::string relative = item.name.substr(prefix.size());
        if (relative.empty() || item.name.back() == '/') {
            continue;
        }
        if (relative.find("..") != std::string::npos) {
            error = "zip entry escapes the component (" + item.name + ")";
            return false;
        }
        std::vector<std::uint8_t> bytes;
        if (!ZipArchive::readEntry(zipPath, item, bytes, error)) {
            return false;
        }
        std::filesystem::path out = dest;
        for (const std::string& part : splitZipPath(relative)) {
            if (part == "..") {
                error = "zip entry escapes the component (" + item.name + ")";
                return false;
            }
            out /= part;
        }
        if (!writeFile(out, bytes, error)) {
            return false;
        }
    }
    return true;
}

}  // namespace

std::vector<RobotLibraryEntry> scanRobotLibrary(const std::string& root, std::string& error) {
    std::vector<RobotLibraryEntry> found;
    std::error_code ec;
    const std::filesystem::path base(root);
    if (!std::filesystem::is_directory(base, ec)) {
        error = "robot library path is not a directory";
        return found;
    }
    std::filesystem::recursive_directory_iterator it(base, std::filesystem::directory_options::skip_permission_denied, ec);
    const std::filesystem::recursive_directory_iterator end;
    if (ec) {
        error = "cannot walk the robot library";
        return found;
    }
    for (; it != end; it.increment(ec)) {
        if (ec) {
            ec.clear();
            continue;
        }
        const std::filesystem::path path = it->path();
        const std::string name = path.filename().string();
        const std::string lower = lowerCopy(name);
        if (it->is_directory(ec) && endsWith(lower, ".cojt")) {
            addDirectory(found, base, path);
            it.disable_recursion_pending();
            continue;
        }
        if (!it->is_regular_file(ec) || !endsWith(lower, ".cojt.zip")) {
            continue;
        }
        std::vector<ZipIndexEntry> indexed;
        std::string zipError;
        if (!ZipArchive::index(path.string(), indexed, zipError)) {
            continue;
        }
        addZipComponents(found, base, path, indexed, "zip", {});
    }
    std::sort(found.begin(), found.end(), [](const RobotLibraryEntry& a, const RobotLibraryEntry& b) {
        if (a.brand != b.brand) {
            return a.brand < b.brand;
        }
        if (a.model != b.model) {
            return a.model < b.model;
        }
        return a.kind < b.kind;
    });
    error.clear();
    return found;
}

std::vector<std::uint8_t> robotLibraryPreview(const RobotLibraryEntry& entry, std::string& error) {
    std::vector<std::uint8_t> bytes;
    if (!entry.preview) {
        error.clear();
        return bytes;
    }
    if (entry.kind == "directory") {
        std::ifstream in((std::filesystem::path(entry.path) / "preview.bmp").string(), std::ios::binary);
        if (!in) {
            error = "preview.bmp was not read";
            return bytes;
        }
        in.seekg(0, std::ios::end);
        const auto size = static_cast<std::size_t>(in.tellg());
        in.seekg(0, std::ios::beg);
        bytes.resize(size);
        if (size > 0) {
            in.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(size));
        }
        if (!in) {
            error = "preview.bmp was not read";
            bytes.clear();
            return bytes;
        }
        error.clear();
        return bytes;
    }
    auto loadZip = [&](const std::string& zipPath, const std::string& previewName) {
        std::vector<ZipIndexEntry> indexed;
        if (!ZipArchive::index(zipPath, indexed, error)) {
            return false;
        }
        for (const ZipIndexEntry& item : indexed) {
            if (item.name == previewName || (previewName.empty() && isPreviewName(item.name) &&
                                             entryUnderPrefix(item.name, entry.innerCojtPrefix))) {
                return ZipArchive::readEntry(zipPath, item, bytes, error);
            }
        }
        error = "preview.bmp is not in the archive";
        return false;
    };
    if (entry.kind == "nested-zip") {
        std::vector<ZipIndexEntry> outer;
        if (!ZipArchive::index(entry.path, outer, error)) {
            return bytes;
        }
        std::vector<std::uint8_t> innerBytes;
        bool found = false;
        for (const ZipIndexEntry& item : outer) {
            if (item.name == entry.innerZipEntry) {
                found = ZipArchive::readEntry(entry.path, item, innerBytes, error);
                break;
            }
        }
        if (!found) {
            if (error.empty()) {
                error = "nested cojt zip was not found";
            }
            return bytes;
        }
        const std::filesystem::path temp = std::filesystem::temp_directory_path() / "ondot-cojt-preview.zip";
        if (!writeFile(temp, innerBytes, error)) {
            return bytes;
        }
        const bool ok = loadZip(temp.string(), entry.previewEntry);
        std::error_code ec;
        std::filesystem::remove(temp, ec);
        if (!ok) {
            bytes.clear();
            return bytes;
        }
        error.clear();
        return bytes;
    }
    if (!loadZip(entry.path, entry.previewEntry)) {
        bytes.clear();
        return bytes;
    }
    error.clear();
    return bytes;
}

bool materializeRobotComponent(const RobotLibraryEntry& entry, const std::string& scratch, std::string& folder,
                               bool& scratchUsed, std::string& error) {
    scratchUsed = false;
    if (entry.kind == "directory") {
        std::error_code ec;
        if (!std::filesystem::is_directory(entry.path, ec)) {
            error = "cojt directory is missing";
            return false;
        }
        folder = entry.path;
        error.clear();
        return true;
    }
    const std::filesystem::path dest(scratch);
    std::error_code ec;
    std::filesystem::remove_all(dest, ec);
    std::filesystem::create_directories(dest, ec);
    if (ec) {
        error = "cannot create a temporary cojt directory";
        return false;
    }
    std::string zipPath = entry.path;
    std::filesystem::path ownedInner;
    if (entry.kind == "nested-zip") {
        std::vector<ZipIndexEntry> outer;
        if (!ZipArchive::index(entry.path, outer, error)) {
            return false;
        }
        std::vector<std::uint8_t> innerBytes;
        bool found = false;
        for (const ZipIndexEntry& item : outer) {
            if (item.name == entry.innerZipEntry) {
                found = ZipArchive::readEntry(entry.path, item, innerBytes, error);
                break;
            }
        }
        if (!found) {
            error = error.empty() ? "nested cojt zip was not found" : error;
            return false;
        }
        ownedInner = dest / "inner.cojt.zip";
        if (!writeFile(ownedInner, innerBytes, error)) {
            return false;
        }
        zipPath = ownedInner.string();
    }
    std::vector<ZipIndexEntry> indexed;
    if (!ZipArchive::index(zipPath, indexed, error)) {
        return false;
    }
    const std::filesystem::path component = dest / (entry.name + ".cojt");
    if (!extractPrefix(zipPath, indexed, entry.innerCojtPrefix, component, error)) {
        return false;
    }
    folder = component.string();
    scratchUsed = true;
    error.clear();
    return true;
}

}  // namespace sim
