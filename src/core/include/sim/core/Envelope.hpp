#pragma once

#include "sim/core/Math.hpp"

#include <nlohmann/json_fwd.hpp>

#include <string>
#include <utility>
#include <vector>

namespace sim {

// Swept volume of a tool path. `samples` are TCP positions in millimeters.
// `nodes` are the scene parts that draw the swept capsules.
struct MotionEnvelope {
    std::string name;
    std::string resource;
    Aabb bounds;
    std::vector<Vec3> samples;
    std::vector<std::string> nodes;
};

// A sphere several robots must not occupy together.
struct InterlockZone {
    std::string name;
    Vec3 center{0, 0, 0};
    double radius = 400;
    // Seconds added to the shared station when two robots occupy the zone together.
    double releaseSeconds = 1.5;
    std::vector<std::string> resources;
};

struct InterlockHit {
    std::string zone;
    std::string a;
    std::string b;
};

nlohmann::json envelopesToJson(const std::vector<MotionEnvelope>& envelopes);
void envelopesFromJson(const nlohmann::json& json, std::vector<MotionEnvelope>& envelopes);
nlohmann::json interlocksToJson(const std::vector<InterlockZone>& zones);
void interlocksFromJson(const nlohmann::json& json, std::vector<InterlockZone>& zones);

}  // namespace sim
