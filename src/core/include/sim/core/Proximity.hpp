#pragma once

#include "sim/core/Math.hpp"

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace sim {

// PDPS-style collision sets. Empty `pairs` means every collidable part is tested.
// Object names are scene node names. Clearance is millimeters.
struct CollisionSet {
    std::string name;
    std::vector<std::string> objects;
};

struct ProximitySetup {
    double clearanceMm = 5;
    std::vector<CollisionSet> sets;
    std::vector<std::pair<std::string, std::string>> pairs;
};

// One convex piece of a mesh, in the mesh's local millimeters.
struct ConvexPiece {
    std::vector<Vec3> vertices;
};

// Cached automatic decomposition. `fingerprint` is the mesh contents, not the pose.
struct ConvexCacheEntry {
    std::string meshId;
    std::uint64_t fingerprint = 0;
    std::vector<ConvexPiece> pieces;
};

nlohmann::json proximityToJson(const ProximitySetup& setup);
void proximityFromJson(const nlohmann::json& json, ProximitySetup& setup);
nlohmann::json convexCacheToJson(const std::vector<ConvexCacheEntry>& cache);
void convexCacheFromJson(const nlohmann::json& json, std::vector<ConvexCacheEntry>& cache);

}  // namespace sim
