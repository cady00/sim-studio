#include "sim/core/Proximity.hpp"

#include <nlohmann/json.hpp>

namespace sim {
namespace {

nlohmann::json vec3ToJson(Vec3 v) { return nlohmann::json::array({v.x, v.y, v.z}); }

Vec3 vec3FromJson(const nlohmann::json& json) {
    if (!json.is_array() || json.size() < 3) {
        return {};
    }
    return {json.at(0).get<double>(), json.at(1).get<double>(), json.at(2).get<double>()};
}

}  // namespace

nlohmann::json proximityToJson(const ProximitySetup& setup) {
    nlohmann::json sets = nlohmann::json::array();
    for (const CollisionSet& set : setup.sets) {
        sets.push_back({{"name", set.name}, {"objects", set.objects}});
    }
    nlohmann::json pairs = nlohmann::json::array();
    for (const auto& pair : setup.pairs) {
        pairs.push_back(nlohmann::json::array({pair.first, pair.second}));
    }
    return {{"clearanceMm", setup.clearanceMm}, {"sets", std::move(sets)}, {"pairs", std::move(pairs)}};
}

void proximityFromJson(const nlohmann::json& json, ProximitySetup& setup) {
    setup = {};
    if (!json.is_object()) {
        return;
    }
    setup.clearanceMm = json.value("clearanceMm", 5.0);
    if (json.contains("sets") && json.at("sets").is_array()) {
        for (const auto& item : json.at("sets")) {
            if (!item.is_object()) {
                continue;
            }
            CollisionSet set;
            set.name = item.value("name", std::string{});
            if (item.contains("objects") && item.at("objects").is_array()) {
                for (const auto& name : item.at("objects")) {
                    if (name.is_string()) {
                        set.objects.push_back(name.get<std::string>());
                    }
                }
            }
            if (!set.name.empty()) {
                setup.sets.push_back(std::move(set));
            }
        }
    }
    if (json.contains("pairs") && json.at("pairs").is_array()) {
        for (const auto& item : json.at("pairs")) {
            if (item.is_array() && item.size() >= 2 && item.at(0).is_string() && item.at(1).is_string()) {
                setup.pairs.emplace_back(item.at(0).get<std::string>(), item.at(1).get<std::string>());
            }
        }
    }
}

nlohmann::json convexCacheToJson(const std::vector<ConvexCacheEntry>& cache) {
    nlohmann::json out = nlohmann::json::array();
    for (const ConvexCacheEntry& entry : cache) {
        nlohmann::json pieces = nlohmann::json::array();
        for (const ConvexPiece& piece : entry.pieces) {
            nlohmann::json vertices = nlohmann::json::array();
            for (const Vec3& vertex : piece.vertices) {
                vertices.push_back(vec3ToJson(vertex));
            }
            pieces.push_back(std::move(vertices));
        }
        out.push_back({{"mesh", entry.meshId},
                       {"fingerprint", entry.fingerprint},
                       {"pieces", std::move(pieces)}});
    }
    return out;
}

void convexCacheFromJson(const nlohmann::json& json, std::vector<ConvexCacheEntry>& cache) {
    cache.clear();
    if (!json.is_array()) {
        return;
    }
    for (const auto& item : json) {
        if (!item.is_object()) {
            continue;
        }
        ConvexCacheEntry entry;
        entry.meshId = item.value("mesh", std::string{});
        entry.fingerprint = item.value("fingerprint", std::uint64_t{0});
        if (item.contains("pieces") && item.at("pieces").is_array()) {
            for (const auto& pieceJson : item.at("pieces")) {
                if (!pieceJson.is_array()) {
                    continue;
                }
                ConvexPiece piece;
                for (const auto& vertex : pieceJson) {
                    piece.vertices.push_back(vec3FromJson(vertex));
                }
                if (piece.vertices.size() >= 1) {
                    entry.pieces.push_back(std::move(piece));
                }
            }
        }
        if (!entry.meshId.empty() && !entry.pieces.empty()) {
            cache.push_back(std::move(entry));
        }
    }
}

}  // namespace sim
