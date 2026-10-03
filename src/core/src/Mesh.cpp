#include "sim/core/Mesh.hpp"

namespace sim {

void MeshData::recomputeBounds() {
    bounds = {};
    for (std::size_t i = 0; i + 2 < positions.size(); i += 3) {
        bounds.expand(Vec3{positions[i], positions[i + 1], positions[i + 2]});
    }
}

}  // namespace sim
