#include "sim/io/GeometryKernel.hpp"

#include "sim/io/AssetImport.hpp"
#include "sim/io/ObjImporter.hpp"
#include "sim/io/StepImporter.hpp"

namespace sim {
namespace {

class IgesCadImporter final : public ICadImporter {
public:
    const char* backendName() const override { return "occt"; }
    const char* format() const override { return "iges"; }

    ImportResult importFile(const std::string& path, Scene& scene) override {
        const AssetImportResult asset = createBuiltinRegistry()->importPath(path, scene);
        ImportResult result;
        result.ok = asset.ok;
        result.message = asset.message;
        result.nodeCount = asset.nodeCount;
        result.meshCount = asset.meshCount;
        result.triangleCount = asset.triangleCount;
        return result;
    }
};

class OcctGeometryKernel final : public IGeometryKernel {
public:
    const char* backendName() const override { return "occt"; }

    std::vector<std::string> formats() const override { return {"step", "stp", "iges", "igs", "obj"}; }

    std::unique_ptr<ICadImporter> createImporter(const std::string& format) const override {
        const std::string key = format;
        if (key == "step" || key == "stp") {
            return std::make_unique<StepImporter>();
        }
        if (key == "iges" || key == "igs") {
            return std::make_unique<IgesCadImporter>();
        }
        if (key == "obj") {
            return std::make_unique<ObjImporter>();
        }
        return nullptr;
    }
};

}  // namespace

std::unique_ptr<IGeometryKernel> createGeometryKernel() { return std::make_unique<OcctGeometryKernel>(); }

}  // namespace sim
