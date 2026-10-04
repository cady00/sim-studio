#include "sim/core/Envelope.hpp"

#include <nlohmann/json.hpp>

namespace sim {
namespace {

nlohmann::json vec3ToJson(Vec3 value) { return nlohmann::json::array({value.x, value.y, value.z}); }

Vec3 vec3FromJson(const nlohmann::json& json) {
    if (!json.is_array() || json.size() < 3) {
        return {};
    }
    return {json.at(0).get<double>(), json.at(1).get<double>(), json.at(2).get<double>()};
}

}  // namespace

nlohmann::json envelopesToJson(const std::vector<MotionEnvelope>& envelopes) {
    nlohmann::json out = nlohmann::json::array();
    for (const MotionEnvelope& envelope : envelopes) {
        nlohmann::json samples = nlohmann::json::array();
        for (const Vec3& sample : envelope.samples) {
            samples.push_back(vec3ToJson(sample));
        }
        out.push_back({{"name", envelope.name},
                       {"resource", envelope.resource},
                       {"samples", std::move(samples)},
                       {"nodes", envelope.nodes}});
    }
    return out;
}

void envelopesFromJson(const nlohmann::json& json, std::vector<MotionEnvelope>& envelopes) {
    envelopes.clear();
    if (!json.is_array()) {
        return;
    }
    for (const auto& item : json) {
        if (!item.is_object()) {
            continue;
        }
        MotionEnvelope envelope;
        envelope.name = item.value("name", std::string{});
        envelope.resource = item.value("resource", std::string{});
        if (item.contains("samples") && item.at("samples").is_array()) {
            for (const auto& sample : item.at("samples")) {
                const Vec3 point = vec3FromJson(sample);
                envelope.samples.push_back(point);
                envelope.bounds.expand(point);
            }
        }
        if (item.contains("nodes") && item.at("nodes").is_array()) {
            for (const auto& node : item.at("nodes")) {
                if (node.is_string()) {
                    envelope.nodes.push_back(node.get<std::string>());
                }
            }
        }
        if (!envelope.name.empty()) {
            envelopes.push_back(std::move(envelope));
        }
    }
}

nlohmann::json interlocksToJson(const std::vector<InterlockZone>& zones) {
    nlohmann::json out = nlohmann::json::array();
    for (const InterlockZone& zone : zones) {
        out.push_back({{"name", zone.name},
                       {"center", vec3ToJson(zone.center)},
                       {"radius", zone.radius},
                       {"resources", zone.resources}});
    }
    return out;
}

void interlocksFromJson(const nlohmann::json& json, std::vector<InterlockZone>& zones) {
    zones.clear();
    if (!json.is_array()) {
        return;
    }
    for (const auto& item : json) {
        if (!item.is_object()) {
            continue;
        }
        InterlockZone zone;
        zone.name = item.value("name", std::string{});
        zone.center = vec3FromJson(item.value("center", nlohmann::json::array()));
        zone.radius = item.value("radius", 400.0);
        if (item.contains("resources") && item.at("resources").is_array()) {
            for (const auto& name : item.at("resources")) {
                if (name.is_string()) {
                    zone.resources.push_back(name.get<std::string>());
                }
            }
        }
        if (!zone.name.empty()) {
            zones.push_back(std::move(zone));
        }
    }
}

}  // namespace sim
