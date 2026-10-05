#include "sim/core/Plc.hpp"

#include <nlohmann/json.hpp>

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
                rung.action != "move" && rung.action != "ctu" && rung.action != "ctr") {
                error = "unknown plc rung action";
                return false;
            }
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
                    contact.address = contactJson.value("address", std::string{});
                    contact.normallyOpen = contactJson.value("normallyOpen", true);
                    contact.edge = contactJson.value("edge", std::string{});
                    if (!contact.edge.empty() && contact.edge != "rising" && contact.edge != "falling") {
                        error = "unknown plc contact edge";
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
    error.clear();
    return true;
}

}  // namespace sim
