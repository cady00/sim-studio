#include "sim/io/GeometryKernel.hpp"

#include "sim/io/ObjImporter.hpp"
#include "sim/io/StepImporter.hpp"

namespace sim {
namespace {

class OcctGeometryKernel final : public IGeometryKernel {
public:
    const char* backendName() const override { return "occt"; }

    std::vector<std::string> formats() const override { return {"step", "stp", "obj"}; }

    std::unique_ptr<ICadImporter> createImporter(const std::string& format) const override {
        const std::string key = format;
        if (key == "step" || key == "stp") {
            return std::make_unique<StepImporter>();
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
