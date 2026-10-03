#include "sim/core/I18n.hpp"

#include <nlohmann/json.hpp>

namespace sim {

bool I18nCatalog::loadJson(const std::string& locale, std::string_view utf8Json, std::string& error) {
    nlohmann::json json;
    try {
        json = nlohmann::json::parse(utf8Json.begin(), utf8Json.end());
    } catch (const std::exception& ex) {
        error = ex.what();
        return false;
    }
    if (!json.is_object()) {
        error = "translation catalog must be a JSON object";
        return false;
    }
    std::map<std::string, std::string> table;
    for (auto it = json.begin(); it != json.end(); ++it) {
        if (!it.value().is_string()) {
            error = "translation value for " + it.key() + " is not a string";
            return false;
        }
        table.emplace(it.key(), it.value().get<std::string>());
    }
    tables_[locale] = std::move(table);
    error.clear();
    return true;
}

void I18nCatalog::setLocale(const std::string& locale) {
    locale_ = locale.empty() ? std::string(kDefaultLocale) : locale;
}

bool I18nCatalog::hasLocale(const std::string& locale) const { return tables_.find(locale) != tables_.end(); }

std::string I18nCatalog::translate(std::string_view key) const {
    const std::string keyString(key);
    const auto active = tables_.find(locale_);
    if (active != tables_.end()) {
        const auto it = active->second.find(keyString);
        if (it != active->second.end() && !it->second.empty()) {
            return it->second;
        }
    }
    const auto english = tables_.find(kFallbackLocale);
    if (english != tables_.end()) {
        const auto it = english->second.find(keyString);
        if (it != english->second.end() && !it->second.empty()) {
            return it->second;
        }
    }
    return keyString;
}

}  // namespace sim
