#pragma once

#include "sim/io/Importer.hpp"

namespace sim {

// Wavefront OBJ (triangles and n-gons fanned into triangles). Phase 1 display import.
class ObjImporter : public IGeometryImporter {
public:
    ImportResult importFile(const std::string& path, Scene& scene) override;
};

// glTF is a planned adapter. The type exists so later phases can register it
// beside STEP and OBJ without changing the application shell.
class GltfImporter : public IGeometryImporter {
public:
    ImportResult importFile(const std::string& path, Scene& scene) override;
};

}  // namespace sim
