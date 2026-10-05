#include "sim/io/Cojt.hpp"
#include "sim/io/RobotLibrary.hpp"
#include "sim/io/ZipArchive.hpp"

#include <catch2/catch_test_macros.hpp>

#include <zlib.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

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

std::vector<std::uint8_t> rawDeflate(const std::vector<std::uint8_t>& input) {
    z_stream stream{};
    REQUIRE(deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -MAX_WBITS, 8, Z_DEFAULT_STRATEGY) == Z_OK);
    std::vector<std::uint8_t> out(deflateBound(&stream, static_cast<uLong>(input.size())));
    stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(input.data()));
    stream.avail_in = static_cast<uInt>(input.size());
    stream.next_out = out.data();
    stream.avail_out = static_cast<uInt>(out.size());
    const int result = deflate(&stream, Z_FINISH);
    const auto produced = static_cast<std::size_t>(stream.total_out);
    deflateEnd(&stream);
    REQUIRE(result == Z_STREAM_END);
    out.resize(produced);
    return out;
}

bool writeZip(const std::filesystem::path& path, const std::vector<std::pair<std::string, std::vector<std::uint8_t>>>& files,
              bool deflate, std::string& error) {
    std::vector<sim::ZipEntry> entries;
    for (const auto& file : files) {
        sim::ZipEntry entry;
        entry.name = file.first;
        entry.data = file.second;
        entries.push_back(std::move(entry));
    }
    if (!deflate) {
        return sim::ZipArchive::write(path.string(), entries, error);
    }
    std::vector<std::uint8_t> blob;
    struct Item {
        std::string name;
        std::uint32_t crc = 0;
        std::uint32_t compressed = 0;
        std::uint32_t uncompressed = 0;
        std::uint32_t offset = 0;
    };
    std::vector<Item> items;
    for (const auto& file : files) {
        Item item;
        item.name = file.first;
        item.uncompressed = static_cast<std::uint32_t>(file.second.size());
        item.crc = file.second.empty()
                       ? 0
                       : static_cast<std::uint32_t>(crc32(0L, file.second.data(), static_cast<uInt>(file.second.size())));
        const std::vector<std::uint8_t> compressed = rawDeflate(file.second);
        item.compressed = static_cast<std::uint32_t>(compressed.size());
        item.offset = static_cast<std::uint32_t>(blob.size());
        appendU32(blob, 0x04034b50u);
        appendU16(blob, 20);
        appendU16(blob, 0);
        appendU16(blob, 8);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, item.crc);
        appendU32(blob, item.compressed);
        appendU32(blob, item.uncompressed);
        appendU16(blob, static_cast<std::uint16_t>(item.name.size()));
        appendU16(blob, 0);
        blob.insert(blob.end(), item.name.begin(), item.name.end());
        blob.insert(blob.end(), compressed.begin(), compressed.end());
        items.push_back(item);
    }
    const auto centralOffset = static_cast<std::uint32_t>(blob.size());
    for (const Item& item : items) {
        appendU32(blob, 0x02014b50u);
        appendU16(blob, 20);
        appendU16(blob, 20);
        appendU16(blob, 0);
        appendU16(blob, 8);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, item.crc);
        appendU32(blob, item.compressed);
        appendU32(blob, item.uncompressed);
        appendU16(blob, static_cast<std::uint16_t>(item.name.size()));
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, 0);
        appendU32(blob, item.offset);
        blob.insert(blob.end(), item.name.begin(), item.name.end());
    }
    const auto centralSize = static_cast<std::uint32_t>(blob.size() - centralOffset);
    appendU32(blob, 0x06054b50u);
    appendU16(blob, 0);
    appendU16(blob, 0);
    appendU16(blob, static_cast<std::uint16_t>(items.size()));
    appendU16(blob, static_cast<std::uint16_t>(items.size()));
    appendU32(blob, centralSize);
    appendU32(blob, centralOffset);
    appendU16(blob, 0);
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        error = "cannot write " + path.string();
        return false;
    }
    out.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
    error.clear();
    return static_cast<bool>(out);
}

std::vector<std::uint8_t> bytesOf(const std::string& text) {
    return std::vector<std::uint8_t>(text.begin(), text.end());
}

const sim::RobotLibraryEntry* findModel(const std::vector<sim::RobotLibraryEntry>& entries, const std::string& model,
                                        const std::string& kind) {
    for (const sim::RobotLibraryEntry& entry : entries) {
        if (entry.model == model && (kind.empty() || entry.kind == kind)) {
            return &entry;
        }
    }
    return nullptr;
}

}  // namespace

TEST_CASE("zip reader inflates method 8") {
    const auto path = std::filesystem::temp_directory_path() / "ondot-deflate-roundtrip.zip";
    std::string error;
    const std::string payload = "cojt-deflate-payload";
    REQUIRE(writeZip(path, {{"demo.cojt/readme.txt", bytesOf(payload)}}, true, error));
    std::vector<sim::ZipEntry> readBack;
    REQUIRE(sim::ZipArchive::read(path.string(), readBack, error));
    REQUIRE(readBack.size() == 1);
    REQUIRE(readBack[0].name == "demo.cojt/readme.txt");
    REQUIRE(std::string(readBack[0].data.begin(), readBack[0].data.end()) == payload);
    std::filesystem::remove(path);
}

TEST_CASE("robot library indexes a nested cojt zip and refuses path escape") {
    const auto root = std::filesystem::temp_directory_path() / "ondot-robot-lib";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root / "KUKA" / "mini.cojt");
    {
        std::ofstream preview(root / "KUKA" / "mini.cojt" / "preview.bmp", std::ios::binary);
        preview << "BMmini";
    }

    std::string error;
    const auto innerPath = root / "inner.cojt.zip";
    std::vector<std::uint8_t> preview{'B', 'M', 'd', 'e', 'm', 'o'};
    REQUIRE(sim::ZipArchive::write(innerPath.string(),
                                  {{"demo.cojt/readme.txt", bytesOf("hello")}, {"demo.cojt/preview.bmp", preview}}, error));
    std::vector<sim::ZipEntry> wrapped;
    {
        std::ifstream inner(innerPath, std::ios::binary);
        std::vector<std::uint8_t> innerBytes((std::istreambuf_iterator<char>(inner)), std::istreambuf_iterator<char>());
        sim::ZipEntry entry;
        entry.name = "ABB/demo.cojt.zip";
        entry.data = std::move(innerBytes);
        wrapped.push_back(std::move(entry));
    }
    REQUIRE(sim::ZipArchive::write((root / "bundle.cojt.zip").string(), wrapped, error));
    std::filesystem::remove(innerPath);

    const auto escapePath = root / "escape.cojt.zip";
    REQUIRE(sim::ZipArchive::write(escapePath.string(), {{"demo.cojt/../outside.txt", bytesOf("nope")}}, error));

    const std::vector<sim::RobotLibraryEntry> entries = sim::scanRobotLibrary(root.string(), error);
    REQUIRE(error.empty());
    const sim::RobotLibraryEntry* mini = findModel(entries, "mini", "directory");
    REQUIRE(mini != nullptr);
    REQUIRE(mini->brand == "KUKA");
    REQUIRE(mini->preview);
    const std::vector<std::uint8_t> miniPreview = sim::robotLibraryPreview(*mini, error);
    REQUIRE(error.empty());
    REQUIRE(miniPreview.size() >= 2);
    REQUIRE(miniPreview[0] == 'B');
    REQUIRE(miniPreview[1] == 'M');

    const sim::RobotLibraryEntry* nested = findModel(entries, "demo", "nested-zip");
    REQUIRE(nested != nullptr);
    REQUIRE(nested->brand == "ABB");
    REQUIRE(nested->preview);
    const std::vector<std::uint8_t> nestedPreview = sim::robotLibraryPreview(*nested, error);
    REQUIRE(error.empty());
    REQUIRE(nestedPreview.size() >= 2);
    REQUIRE(nestedPreview[0] == 'B');
    REQUIRE(nestedPreview[1] == 'M');

    const auto scratch = root / "scratch";
    std::string folder;
    bool scratchUsed = false;
    REQUIRE(sim::materializeRobotComponent(*nested, scratch.string(), folder, scratchUsed, error));
    REQUIRE(scratchUsed);
    REQUIRE(std::filesystem::is_regular_file(std::filesystem::path(folder) / "readme.txt"));

    const sim::RobotLibraryEntry* escape = findModel(entries, "demo", "zip");
    REQUIRE(escape != nullptr);
    std::string escapedFolder;
    bool escapedScratch = false;
    REQUIRE_FALSE(sim::materializeRobotComponent(*escape, (root / "bad").string(), escapedFolder, escapedScratch, error));
    REQUIRE(error.find("escapes") != std::string::npos);
    std::filesystem::remove_all(root);
}

TEST_CASE("private cojt library lists kr210 preview and imports kr6 from a zip") {
    const auto root = std::filesystem::path(SIM_SOURCE_DIR) / "testdata" / "local" / "cojt";
    if (!std::filesystem::is_directory(root)) {
        SKIP("testdata/local/cojt is not present");
    }
    std::string error;
    const std::vector<sim::RobotLibraryEntry> entries = sim::scanRobotLibrary(root.string(), error);
    REQUIRE(error.empty());
    const sim::RobotLibraryEntry* kr210 = findModel(entries, "kr210_r2700_2", "directory");
    REQUIRE(kr210 != nullptr);
    REQUIRE(kr210->brand == "KUKA");
    REQUIRE(kr210->preview);
    const std::vector<std::uint8_t> preview = sim::robotLibraryPreview(*kr210, error);
    REQUIRE(error.empty());
    REQUIRE(preview.size() >= 2);
    REQUIRE(preview[0] == 'B');
    REQUIRE(preview[1] == 'M');
    REQUIRE(findModel(entries, "kr420r3080_4", "directory") != nullptr);
    REQUIRE(findModel(entries, "kr420r3080_4", "zip") != nullptr);

    const sim::RobotLibraryEntry* kr6 = findModel(entries, "kr6_r900sixx", "zip");
    REQUIRE(kr6 != nullptr);
    const auto scratch = std::filesystem::temp_directory_path() / "ondot-kr6-cojt";
    std::string folder;
    bool scratchUsed = false;
    REQUIRE(sim::materializeRobotComponent(*kr6, scratch.string(), folder, scratchUsed, error));
    REQUIRE(scratchUsed);
    sim::Scene scene;
    const sim::AssetImportResult imported = sim::importCojt(folder, scene);
    INFO(imported.message);
    REQUIRE(imported.ok);
    REQUIRE(imported.triangleCount == 21915);
    REQUIRE(sim::findResource(scene.resources, "kr6_r900sixx") != nullptr);
    std::filesystem::remove_all(scratch);
}
