#include "sim/core/Plc.hpp"

#include <nlohmann/json.hpp>

#include <functional>

namespace sim {
namespace {

nlohmann::json tagToJson(const PlcTag& tag) {
    return nlohmann::json{{"address", tag.address},
                          {"kind", plcValueKindName(tag.kind)},
                          {"flag", tag.flag},
                          {"integer", tag.integer},
                          {"real", tag.real},
                          {"preset", tag.preset},
                          {"elapsed", tag.elapsed},
                          {"done", tag.done},
                          {"forced", tag.forced},
                          {"forceFlag", tag.forceFlag},
                          {"forceInteger", tag.forceInteger},
                          {"forceReal", tag.forceReal}};
}

nlohmann::json bindingToJson(const PlcBinding& binding) {
    return nlohmann::json{{"device", binding.device},
                          {"address", binding.address},
                          {"direction", binding.direction},
                          {"opcNode", binding.opcNode}};
}

nlohmann::json contactToJson(const PlcContact& contact) {
    nlohmann::json json{{"address", contact.address}, {"normallyOpen", contact.normallyOpen}};
    if (!contact.edge.empty()) {
        json["edge"] = contact.edge;
    }
    if (!contact.alternatives.empty()) {
        nlohmann::json any = nlohmann::json::array();
        for (const PlcContact& alternative : contact.alternatives) {
            nlohmann::json item{{"address", alternative.address}, {"normallyOpen", alternative.normallyOpen}};
            if (!alternative.edge.empty()) {
                item["edge"] = alternative.edge;
            }
            any.push_back(std::move(item));
        }
        json["or"] = std::move(any);
    }
    return json;
}

nlohmann::json rungToJson(const PlcRung& rung) {
    nlohmann::json series = nlohmann::json::array();
    for (const PlcContact& contact : rung.series) {
        series.push_back(contactToJson(contact));
    }
    nlohmann::json json{{"series", std::move(series)}, {"coil", rung.coil}, {"action", rung.action}, {"literal", rung.literal}};
    if (!rung.parallel.empty()) {
        nlohmann::json parallel = nlohmann::json::array();
        for (const std::vector<PlcContact>& branch : rung.parallel) {
            nlohmann::json contacts = nlohmann::json::array();
            for (const PlcContact& contact : branch) {
                contacts.push_back(contactToJson(contact));
            }
            parallel.push_back(std::move(contacts));
        }
        json["parallel"] = std::move(parallel);
    }
    return json;
}

}  // namespace

const char* plcValueKindName(PlcValueKind kind) {
    switch (kind) {
        case PlcValueKind::Bool:
            return "bool";
        case PlcValueKind::Int:
            return "int";
        case PlcValueKind::Real:
            return "real";
        case PlcValueKind::Timer:
            return "timer";
        case PlcValueKind::Counter:
            return "counter";
    }
    return "bool";
}

bool plcValueKindFromName(const std::string& name, PlcValueKind& out) {
    if (name == "bool") {
        out = PlcValueKind::Bool;
        return true;
    }
    if (name == "int") {
        out = PlcValueKind::Int;
        return true;
    }
    if (name == "real") {
        out = PlcValueKind::Real;
        return true;
    }
    if (name == "timer") {
        out = PlcValueKind::Timer;
        return true;
    }
    if (name == "counter") {
        out = PlcValueKind::Counter;
        return true;
    }
    return false;
}

PlcTag* findPlcTag(SoftPlcDocument& doc, const std::string& address) {
    for (PlcTag& tag : doc.tags) {
        if (tag.address == address) {
            return &tag;
        }
    }
    return nullptr;
}

const PlcTag* findPlcTag(const SoftPlcDocument& doc, const std::string& address) {
    for (const PlcTag& tag : doc.tags) {
        if (tag.address == address) {
            return &tag;
        }
    }
    return nullptr;
}

nlohmann::json softPlcToJson(const SoftPlcDocument& doc) {
    nlohmann::json json{{"enabled", doc.enabled}, {"scanSeconds", doc.scanSeconds}, {"backend", doc.backend}};
    json["tags"] = nlohmann::json::array();
    json["bindings"] = nlohmann::json::array();
    json["rungs"] = nlohmann::json::array();
    for (const PlcTag& tag : doc.tags) {
        json["tags"].push_back(tagToJson(tag));
    }
    for (const PlcBinding& binding : doc.bindings) {
        json["bindings"].push_back(bindingToJson(binding));
    }
    for (const PlcRung& rung : doc.rungs) {
        json["rungs"].push_back(rungToJson(rung));
    }
    if (!doc.zones.empty()) {
        json["zones"] = nlohmann::json::array();
        for (const PlcZoneWatch& zone : doc.zones) {
            const std::string shape = zone.shape == "box" || zone.shape == "cylinder" ? zone.shape : "sphere";
            nlohmann::json item{{"name", zone.name},
                                {"shape", shape},
                                {"center", {zone.center.x, zone.center.y, zone.center.z}},
                                {"radius", zone.radius},
                                {"size", {zone.size.x, zone.size.y, zone.size.z}},
                                {"watch", zone.watch}};
            if (!zone.address.empty()) {
                item["address"] = zone.address;
            }
            json["zones"].push_back(std::move(item));
        }
    }
    if (!doc.collisions.empty()) {
        json["collisions"] = nlohmann::json::array();
        for (const PlcCollisionWatch& watch : doc.collisions) {
            json["collisions"].push_back(nlohmann::json{{"address", watch.address}, {"resource", watch.resource}});
        }
    }
    if (!doc.cycles.empty()) {
        json["cycles"] = nlohmann::json::array();
        for (const PlcCycleRule& rule : doc.cycles) {
            json["cycles"].push_back(nlohmann::json{{"id", rule.id},
                                                    {"name", rule.name},
                                                    {"station", rule.station},
                                                    {"stationName", rule.stationName},
                                                    {"startSignal", rule.startSignal},
                                                    {"startEdge", rule.startEdge},
                                                    {"endSignal", rule.endSignal},
                                                    {"endEdge", rule.endEdge},
                                                    {"busySignal", rule.busySignal},
                                                    {"blockSignal", rule.blockSignal}});
        }
    }
    return json;
}

bool softPlcFromJson(const nlohmann::json& json, SoftPlcDocument& doc, std::string& error) {
    doc = {};
    if (!json.is_object()) {
        error = "soft plc document is not an object";
        return false;
    }
    doc.enabled = json.value("enabled", false);
    doc.scanSeconds = json.value("scanSeconds", 0.01);
    doc.backend = json.value("backend", std::string("softplc"));
    if (doc.backend != "softplc" && doc.backend != "plcsim") {
        error = "unknown plc backend";
        return false;
    }
    if (doc.scanSeconds < 1e-4) {
        doc.scanSeconds = 1e-4;
    }
    if (json.contains("tags") && json.at("tags").is_array()) {
        for (const auto& item : json.at("tags")) {
            if (!item.is_object()) {
                continue;
            }
            PlcTag tag;
            tag.address = item.value("address", std::string{});
            PlcValueKind kind = PlcValueKind::Bool;
            if (!plcValueKindFromName(item.value("kind", std::string("bool")), kind)) {
                error = "unknown plc tag kind";
                return false;
            }
            tag.kind = kind;
            tag.flag = item.value("flag", false);
            tag.integer = item.value("integer", static_cast<std::int64_t>(0));
            tag.real = item.value("real", 0.0);
            tag.preset = item.value("preset", 0.0);
            tag.elapsed = item.value("elapsed", 0.0);
            tag.done = item.value("done", false);
            tag.forced = item.value("forced", false);
            tag.forceFlag = item.value("forceFlag", false);
            tag.forceInteger = item.value("forceInteger", static_cast<std::int64_t>(0));
            tag.forceReal = item.value("forceReal", 0.0);
            if (!tag.address.empty()) {
                doc.tags.push_back(std::move(tag));
            }
        }
    }
    if (json.contains("bindings") && json.at("bindings").is_array()) {
        for (const auto& item : json.at("bindings")) {
            if (!item.is_object()) {
                continue;
            }
            PlcBinding binding;
            binding.device = item.value("device", std::string{});
            binding.address = item.value("address", std::string{});
            binding.direction = item.value("direction", std::string("in"));
            binding.opcNode = item.value("opcNode", std::string{});
            if (binding.direction != "in" && binding.direction != "out") {
                error = "plc binding direction must be in or out";
                return false;
            }
            if (!binding.address.empty()) {
                doc.bindings.push_back(std::move(binding));
            }
        }
    }
    if (json.contains("rungs") && json.at("rungs").is_array()) {
        for (const auto& item : json.at("rungs")) {
            if (!item.is_object()) {
                continue;
            }
            PlcRung rung;
            rung.coil = item.value("coil", std::string{});
            rung.action = item.value("action", std::string("out"));
            rung.literal = item.value("literal", 0.0);
            if (rung.action != "out" && rung.action != "set" && rung.action != "reset" && rung.action != "ton" &&
                rung.action != "tof" && rung.action != "move" && rung.action != "ctu" && rung.action != "ctd" &&
                rung.action != "ctr") {
                error = "unknown plc rung action";
                return false;
            }
            std::function<bool(const nlohmann::json&, PlcContact&, bool)> readOne;
            readOne = [&](const nlohmann::json& contactJson, PlcContact& contact, bool allowOr) -> bool {
                contact.address = contactJson.value("address", std::string{});
                contact.normallyOpen = contactJson.value("normallyOpen", true);
                contact.edge = contactJson.value("edge", std::string{});
                if (!contact.edge.empty() && contact.edge != "rising" && contact.edge != "falling") {
                    error = "unknown plc contact edge";
                    return false;
                }
                if (!contactJson.contains("or")) {
                    return true;
                }
                if (!allowOr) {
                    error = "plc contact or does not nest";
                    return false;
                }
                if (!contactJson.at("or").is_array()) {
                    error = "plc contact or is not an array";
                    return false;
                }
                for (const auto& alternativeJson : contactJson.at("or")) {
                    if (!alternativeJson.is_object()) {
                        continue;
                    }
                    PlcContact alternative;
                    if (!readOne(alternativeJson, alternative, false)) {
                        return false;
                    }
                    if (!alternative.address.empty()) {
                        contact.alternatives.push_back(std::move(alternative));
                    }
                }
                return true;
            };
            auto readContacts = [&](const nlohmann::json& list, std::vector<PlcContact>& out) {
                if (!list.is_array()) {
                    error = "plc contact list is not an array";
                    return false;
                }
                for (const auto& contactJson : list) {
                    if (!contactJson.is_object()) {
                        continue;
                    }
                    PlcContact contact;
                    if (!readOne(contactJson, contact, true)) {
                        return false;
                    }
                    if (!contact.address.empty()) {
                        out.push_back(std::move(contact));
                    }
                }
                return true;
            };
            if (item.contains("series") && !readContacts(item.at("series"), rung.series)) {
                return false;
            }
            if (item.contains("parallel") && item.at("parallel").is_array()) {
                for (const auto& branchJson : item.at("parallel")) {
                    std::vector<PlcContact> branch;
                    if (!readContacts(branchJson, branch)) {
                        return false;
                    }
                    if (!branch.empty()) {
                        rung.parallel.push_back(std::move(branch));
                    }
                }
            }
            if (!rung.coil.empty()) {
                doc.rungs.push_back(std::move(rung));
            }
        }
    }
    if (json.contains("zones") && json.at("zones").is_array()) {
        for (const auto& item : json.at("zones")) {
            if (!item.is_object()) {
                continue;
            }
            PlcZoneWatch zone;
            zone.name = item.value("name", std::string{});
            zone.address = item.value("address", std::string{});
            zone.shape = item.value("shape", std::string{"sphere"});
            if (zone.shape != "box" && zone.shape != "cylinder") {
                zone.shape = "sphere";
            }
            zone.radius = item.value("radius", 150.0);
            if (item.contains("center") && item.at("center").is_array() && item.at("center").size() >= 3) {
                zone.center = {item.at("center").at(0).get<double>(), item.at("center").at(1).get<double>(),
                               item.at("center").at(2).get<double>()};
            }
            if (item.contains("size") && item.at("size").is_array() && item.at("size").size() >= 3) {
                zone.size = {item.at("size").at(0).get<double>(), item.at("size").at(1).get<double>(),
                             item.at("size").at(2).get<double>()};
            } else {
                zone.size = {zone.radius, zone.radius, zone.radius};
            }
            if (item.contains("watch") && item.at("watch").is_array()) {
                for (const auto& name : item.at("watch")) {
                    if (name.is_string()) {
                        zone.watch.push_back(name.get<std::string>());
                    }
                }
            }
            if (!zone.name.empty() || !zone.address.empty()) {
                doc.zones.push_back(std::move(zone));
            }
        }
    }
    if (json.contains("collisions") && json.at("collisions").is_array()) {
        for (const auto& item : json.at("collisions")) {
            if (!item.is_object()) {
                continue;
            }
            PlcCollisionWatch watch;
            watch.address = item.value("address", std::string{});
            watch.resource = item.value("resource", std::string{});
            doc.collisions.push_back(std::move(watch));
        }
    }
    if (json.contains("cycles") && json.at("cycles").is_array()) {
        for (const auto& item : json.at("cycles")) {
            if (!item.is_object()) {
                continue;
            }
            PlcCycleRule rule;
            rule.id = item.value("id", std::string{});
            rule.name = item.value("name", std::string{});
            rule.station = item.value("station", std::string{});
            rule.stationName = item.value("stationName", std::string{});
            rule.startSignal = item.value("startSignal", std::string{});
            rule.startEdge = item.value("startEdge", std::string{"rising"});
            rule.endSignal = item.value("endSignal", std::string{});
            rule.endEdge = item.value("endEdge", std::string{"rising"});
            rule.busySignal = item.value("busySignal", std::string{});
            rule.blockSignal = item.value("blockSignal", std::string{});
            if (rule.startEdge != "falling") {
                rule.startEdge = "rising";
            }
            if (rule.endEdge != "falling") {
                rule.endEdge = "rising";
            }
            if (!rule.id.empty() && !rule.startSignal.empty()) {
                doc.cycles.push_back(std::move(rule));
            }
        }
    }
    error.clear();
    return true;
}

}  // namespace sim
