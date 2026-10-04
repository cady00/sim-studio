#pragma once

#include "sim/io/GeometryKernel.hpp"

namespace sim {

class StepImporter : public ICadImporter {
public:
    const char* backendName() const override { return "occt"; }
    const char* format() const override { return "step"; }
    ImportResult importFile(const std::string& path, Scene& scene) override;
    ImportResult importFile(const std::string& path, Scene& scene, const ImportProgressFn& progress,
                            std::atomic<bool>* cancel);
};

}  // namespace sim
