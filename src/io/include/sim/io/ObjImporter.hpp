#pragma once

#include "sim/io/GeometryKernel.hpp"

namespace sim {

// Wavefront OBJ (triangles and n-gons fanned into triangles). Parsed here, not by OCCT.
class ObjImporter : public ICadImporter {
public:
    const char* backendName() const override { return "builtin"; }
    const char* format() const override { return "obj"; }
    ImportResult importFile(const std::string& path, Scene& scene) override;
};

// glTF is a planned adapter. The type exists so later phases can register it
// beside STEP and OBJ without changing the application shell.
class GltfImporter : public IGeometryImporter {
public:
    ImportResult importFile(const std::string& path, Scene& scene) override;
};

}  // namespace sim
