#pragma once

#include "sim/io/Importer.hpp"

namespace sim {

class StepImporter : public IGeometryImporter {
public:
    ImportResult importFile(const std::string& path, Scene& scene) override;
    ImportResult importFile(const std::string& path, Scene& scene, const ImportProgressFn& progress,
                            std::atomic<bool>* cancel);
};

}  // namespace sim
