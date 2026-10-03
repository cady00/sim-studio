#pragma once

#include <map>
#include <string>
#include <string_view>

namespace sim {

// Tiny catalog used by the Qt shell. Active locale, then English, then the key.
class I18nCatalog {
public:
    static constexpr const char* kDefaultLocale = "zh_CN";
    static constexpr const char* kFallbackLocale = "en";

    bool loadJson(const std::string& locale, std::string_view utf8Json, std::string& error);
    void setLocale(const std::string& locale);
    const std::string& locale() const { return locale_; }
    std::string translate(std::string_view key) const;
    bool hasLocale(const std::string& locale) const;

private:
    std::map<std::string, std::map<std::string, std::string>> tables_;
    std::string locale_ = kDefaultLocale;
};

}  // namespace sim
