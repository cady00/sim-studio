#pragma once

#include "sim/core/Scene.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sim {

// Expands xacro includes, properties, macros, and ${} expressions. A plain URDF is returned unchanged.
bool expandXacroDocument(const std::string& text, const std::string& baseDir, std::string& expanded, std::string& error);

struct AssetImportResult {
    bool ok = false;
    std::string message;
    std::string format;
    int nodeCount = 0;
    int meshCount = 0;
    int triangleCount = 0;
};

// One format. The registry picks an importer by extension, then by the first bytes.
class IAssetImporter {
public:
    virtual ~IAssetImporter() = default;
    virtual const char* format() const = 0;
    virtual std::vector<std::string> extensions() const = 0;
    virtual bool probeContent(const std::string& head) const = 0;
    virtual AssetImportResult importFile(const std::string& path, Scene& scene) const = 0;
};

class AssetRegistry {
public:
    void add(std::unique_ptr<IAssetImporter> importer);
    const IAssetImporter* match(const std::string& path, const std::string& head) const;
    AssetImportResult importPath(const std::string& path, Scene& scene) const;
    std::vector<std::string> formats() const;

private:
    std::vector<std::unique_ptr<IAssetImporter>> importers_;
};

std::unique_ptr<AssetRegistry> createBuiltinRegistry();
void registerOcctAssetImporters(AssetRegistry& registry);

// Writes a box so tests can round-trip STEP, IGES, and BREP without a committed CAD file.
bool writeSampleShape(const std::string& format, const std::string& path, std::string& error);

bool exportSceneStl(const Scene& scene, const std::string& path, std::string& error);
// .glb is binary. Any other path is a glTF JSON file with an embedded buffer.
bool exportSceneGltf(const Scene& scene, const std::string& path, std::string& error);

// ASCII USD written by this repository. Lengths stay millimeters and metersPerUnit is 0.001.
// OpenUSD is not linked. `validateUsdaText` checks the subset the writer emits.
struct UsdIoResult {
    bool ok = false;
    std::string message;
    int nodeCount = 0;
    int meshCount = 0;
    int triangleCount = 0;
    int jointCount = 0;
    int sampleCount = 0;
};

UsdIoResult exportSceneUsda(const Scene& scene, const std::string& path);
bool validateUsdaText(const std::string& text, std::string& error);
std::unique_ptr<IAssetImporter> createUsdaImporter();

}  // namespace sim
