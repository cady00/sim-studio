#pragma once

#include "sim/io/Importer.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sim {

// Format-specific CAD reader. Implementations live in src/io. Callers in the
// application and tests see scene nodes, meshes, and face axes, not a kernel type.
class ICadImporter : public IGeometryImporter {
public:
    virtual const char* backendName() const = 0;
    virtual const char* format() const = 0;
};

// Geometry kernel seam. Open CASCADE is the current backend. Swapping it means
// a new IGeometryKernel; Scene, the kinematics editor, and .simproj stay put.
// OBJ is parsed in this repository and is registered beside the kernel.
class IGeometryKernel {
public:
    virtual ~IGeometryKernel() = default;
    virtual const char* backendName() const = 0;
    virtual std::vector<std::string> formats() const = 0;
    virtual std::unique_ptr<ICadImporter> createImporter(const std::string& format) const = 0;
};

std::unique_ptr<IGeometryKernel> createGeometryKernel();

}  // namespace sim
