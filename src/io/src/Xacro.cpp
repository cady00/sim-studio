#include "sim/io/AssetImport.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace sim {
namespace {

std::string readTextFile(const std::string& path, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open " + path;
        return {};
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    error.clear();
    return buffer.str();
}

std::vector<std::string> packageRoots(const std::string& baseDir) {
    std::vector<std::string> roots;
    auto pushEnv = [&](const char* key) {
        const char* value = std::getenv(key);
        if (value == nullptr) {
            return;
        }
        std::string text = value;
        std::size_t start = 0;
        while (start <= text.size()) {
            const auto cut = text.find(':', start);
            const std::string piece = text.substr(start, cut == std::string::npos ? std::string::npos : cut - start);
            if (!piece.empty()) {
                roots.push_back(piece);
            }
            if (cut == std::string::npos) {
                break;
            }
            start = cut + 1;
        }
    };
    pushEnv("ROS_PACKAGE_PATH");
    pushEnv("SIM_PACKAGE_PATH");
    if (!baseDir.empty()) {
        std::string dir = baseDir;
        while (!dir.empty() && (dir.back() == '/' || dir.back() == '\\')) {
            dir.pop_back();
        }
        roots.push_back(dir);
        const auto parent = dir.find_last_of("/\\");
        if (parent != std::string::npos && parent > 0) {
            roots.push_back(dir.substr(0, parent));
        }
    }
    return roots;
}

std::string expandFind(const std::string& text, const std::string& baseDir) {
    std::string out = text;
    const std::string token = "$(find ";
    const std::vector<std::string> roots = packageRoots(baseDir);
    for (int guard = 0; guard < 32; ++guard) {
        const auto at = out.find(token);
        if (at == std::string::npos) {
            break;
        }
        const auto end = out.find(')', at + token.size());
        if (end == std::string::npos) {
            break;
        }
        std::string package = out.substr(at + token.size(), end - (at + token.size()));
        while (!package.empty() && std::isspace(static_cast<unsigned char>(package.back()))) {
            package.pop_back();
        }
        std::string resolved = package;
        for (const std::string& root : roots) {
            const std::string candidate = root + "/" + package;
            if (std::filesystem::exists(candidate)) {
                resolved = candidate;
                break;
            }
        }
        out.replace(at, end - at + 1, resolved);
    }
    return out;
}

std::string resolveRelative(const std::string& filename, const std::string& baseDir) {
    const std::string expanded = expandFind(filename, baseDir);
    if (!expanded.empty() && (expanded[0] == '/' || (expanded.size() > 2 && expanded[1] == ':'))) {
        return expanded;
    }
    if (std::filesystem::exists(expanded)) {
        return expanded;
    }
    return baseDir + expanded;
}

struct Macro {
    std::string name;
    std::vector<std::pair<std::string, std::string>> params;
    std::string body;
};

bool evalExpression(const std::string& text, std::size_t& cursor, const std::map<std::string, std::string>& symbols, double& value);

bool readNumber(const std::string& text, std::size_t& cursor, double& value) {
    while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
        ++cursor;
    }
    if (cursor >= text.size()) {
        return false;
    }
    char* end = nullptr;
    value = std::strtod(text.c_str() + cursor, &end);
    if (end == text.c_str() + cursor) {
        return false;
    }
    cursor = static_cast<std::size_t>(end - text.c_str());
    return true;
}

bool evalFactor(const std::string& text, std::size_t& cursor, const std::map<std::string, std::string>& symbols, double& value) {
    while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
        ++cursor;
    }
    if (cursor >= text.size()) {
        return false;
    }
    if (text[cursor] == '(') {
        ++cursor;
        if (!evalExpression(text, cursor, symbols, value)) {
            return false;
        }
        while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
            ++cursor;
        }
        if (cursor >= text.size() || text[cursor] != ')') {
            return false;
        }
        ++cursor;
        return true;
    }
    if (text[cursor] == '-' || text[cursor] == '+') {
        const char sign = text[cursor++];
        if (!evalFactor(text, cursor, symbols, value)) {
            return false;
        }
        if (sign == '-') {
            value = -value;
        }
        return true;
    }
    if (std::isalpha(static_cast<unsigned char>(text[cursor])) || text[cursor] == '_') {
        const std::size_t start = cursor;
        ++cursor;
        while (cursor < text.size() && (std::isalnum(static_cast<unsigned char>(text[cursor])) || text[cursor] == '_')) {
            ++cursor;
        }
        const std::string name = text.substr(start, cursor - start);
        while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
            ++cursor;
        }
        if (cursor < text.size() && text[cursor] == '(') {
            ++cursor;
            double argument = 0;
            if (!evalExpression(text, cursor, symbols, argument)) {
                return false;
            }
            while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
                ++cursor;
            }
            if (cursor >= text.size() || text[cursor] != ')') {
                return false;
            }
            ++cursor;
            if (name == "radians") {
                value = argument * 3.14159265358979323846 / 180.0;
                return true;
            }
            if (name == "degrees") {
                value = argument * 180.0 / 3.14159265358979323846;
                return true;
            }
            return false;
        }
        if (name == "pi") {
            value = 3.14159265358979323846;
            return true;
        }
        const auto found = symbols.find(name);
        if (found == symbols.end()) {
            return false;
        }
        char* end = nullptr;
        value = std::strtod(found->second.c_str(), &end);
        return end != found->second.c_str();
    }
    return readNumber(text, cursor, value);
}

bool evalTerm(const std::string& text, std::size_t& cursor, const std::map<std::string, std::string>& symbols, double& value) {
    if (!evalFactor(text, cursor, symbols, value)) {
        return false;
    }
    while (true) {
        while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
            ++cursor;
        }
        if (cursor >= text.size() || (text[cursor] != '*' && text[cursor] != '/')) {
            return true;
        }
        const char op = text[cursor++];
        double right = 0;
        if (!evalFactor(text, cursor, symbols, right)) {
            return false;
        }
        if (op == '*') {
            value *= right;
        } else if (std::abs(right) < 1e-15) {
            return false;
        } else {
            value /= right;
        }
    }
}

bool evalExpression(const std::string& text, std::size_t& cursor, const std::map<std::string, std::string>& symbols, double& value) {
    if (!evalTerm(text, cursor, symbols, value)) {
        return false;
    }
    while (true) {
        while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
            ++cursor;
        }
        if (cursor >= text.size() || (text[cursor] != '+' && text[cursor] != '-')) {
            return true;
        }
        const char op = text[cursor++];
        double right = 0;
        if (!evalTerm(text, cursor, symbols, right)) {
            return false;
        }
        value = op == '+' ? value + right : value - right;
    }
}

bool evalExpression(const std::string& text, const std::map<std::string, std::string>& symbols, double& value) {
    std::size_t cursor = 0;
    if (!evalExpression(text, cursor, symbols, value)) {
        return false;
    }
    while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
        ++cursor;
    }
    return cursor == text.size();
}

std::string substitute(const std::string& text, const std::map<std::string, std::string>& symbols) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size();) {
        if (text[i] == '$' && i + 1 < text.size() && text[i + 1] == '{') {
            const auto end = text.find('}', i + 2);
            if (end == std::string::npos) {
                out.push_back(text[i]);
                ++i;
                continue;
            }
            const std::string body = text.substr(i + 2, end - (i + 2));
            const auto exact = symbols.find(body);
            if (exact != symbols.end()) {
                out += exact->second;
            } else {
                double value = 0;
                if (evalExpression(body, symbols, value)) {
                    std::ostringstream number;
                    number.setf(std::ios::fmtflags(0), std::ios::floatfield);
                    number.precision(16);
                    number << value;
                    out += number.str();
                } else {
                    out += text.substr(i, end - i + 1);
                }
            }
            i = end + 1;
            continue;
        }
        out.push_back(text[i]);
        ++i;
    }
    return out;
}

std::map<std::string, std::string> readAttributes(const std::string& tag) {
    std::map<std::string, std::string> attributes;
    std::size_t cursor = 0;
    while (cursor < tag.size()) {
        while (cursor < tag.size() && (std::isspace(static_cast<unsigned char>(tag[cursor])) || tag[cursor] == '<' || tag[cursor] == '/')) {
            ++cursor;
        }
        if (cursor >= tag.size() || tag[cursor] == '>') {
            break;
        }
        const std::size_t keyStart = cursor;
        while (cursor < tag.size() && tag[cursor] != '=' && !std::isspace(static_cast<unsigned char>(tag[cursor])) && tag[cursor] != '>') {
            ++cursor;
        }
        const std::string key = tag.substr(keyStart, cursor - keyStart);
        while (cursor < tag.size() && std::isspace(static_cast<unsigned char>(tag[cursor]))) {
            ++cursor;
        }
        if (key.empty() || cursor >= tag.size() || tag[cursor] != '=') {
            if (!key.empty() && key.find(':') == std::string::npos && key != "robot") {
                break;
            }
            continue;
        }
        ++cursor;
        while (cursor < tag.size() && std::isspace(static_cast<unsigned char>(tag[cursor]))) {
            ++cursor;
        }
        if (cursor >= tag.size() || (tag[cursor] != '"' && tag[cursor] != '\'')) {
            break;
        }
        const char quote = tag[cursor++];
        const std::size_t valueStart = cursor;
        while (cursor < tag.size() && tag[cursor] != quote) {
            ++cursor;
        }
        attributes[key] = tag.substr(valueStart, cursor - valueStart);
        if (cursor < tag.size()) {
            ++cursor;
        }
    }
    return attributes;
}

bool expandIncludes(std::string& text, const std::string& baseDir, std::string& error) {
    std::vector<std::string> seen;
    for (int guard = 0; guard < 32; ++guard) {
        const auto at = text.find("<xacro:include");
        if (at == std::string::npos) {
            error.clear();
            return true;
        }
        const auto end = text.find('>', at);
        if (end == std::string::npos) {
            error = "xacro include is not closed";
            return false;
        }
        const auto attributes = readAttributes(text.substr(at, end - at + 1));
        const auto filename = attributes.find("filename");
        if (filename == attributes.end()) {
            error = "xacro include has no filename";
            return false;
        }
        const std::string path = resolveRelative(filename->second, baseDir);
        if (std::find(seen.begin(), seen.end(), path) != seen.end()) {
            error = "xacro include repeats " + path;
            return false;
        }
        seen.push_back(path);
        std::string readError;
        const std::string included = readTextFile(path, readError);
        if (!readError.empty()) {
            error = readError;
            return false;
        }
        text.replace(at, end - at + 1, included);
    }
    error = "xacro includes are nested too deeply";
    return false;
}

std::vector<std::pair<std::string, std::string>> parseParams(const std::string& text) {
    std::vector<std::pair<std::string, std::string>> params;
    std::string token;
    auto flush = [&] {
        if (token.empty()) {
            return;
        }
        const auto eq = token.find(":=");
        if (eq == std::string::npos) {
            params.emplace_back(token, "");
        } else {
            params.emplace_back(token.substr(0, eq), token.substr(eq + 2));
        }
        token.clear();
    };
    for (char c : text) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            flush();
        } else {
            token.push_back(c);
        }
    }
    flush();
    return params;
}

void takeMacros(std::string& text, std::vector<Macro>& macros) {
    for (int guard = 0; guard < 64; ++guard) {
        const auto at = text.find("<xacro:macro");
        if (at == std::string::npos) {
            return;
        }
        const auto openEnd = text.find('>', at);
        if (openEnd == std::string::npos) {
            return;
        }
        const auto attributes = readAttributes(text.substr(at, openEnd - at + 1));
        const auto close = text.find("</xacro:macro>", openEnd);
        if (close == std::string::npos) {
            return;
        }
        Macro macro;
        macro.name = attributes.count("name") ? attributes.at("name") : "";
        macro.params = parseParams(attributes.count("params") ? attributes.at("params") : "");
        macro.body = text.substr(openEnd + 1, close - (openEnd + 1));
        if (!macro.name.empty()) {
            macros.push_back(std::move(macro));
        }
        text.erase(at, close + std::string("</xacro:macro>").size() - at);
    }
}

void takeProperties(std::string& text, std::map<std::string, std::string>& symbols) {
    for (int guard = 0; guard < 128; ++guard) {
        const auto propertyAt = text.find("<xacro:property");
        const auto argAt = text.find("<xacro:arg");
        auto at = std::string::npos;
        if (propertyAt == std::string::npos) {
            at = argAt;
        } else if (argAt == std::string::npos) {
            at = propertyAt;
        } else {
            at = std::min(propertyAt, argAt);
        }
        if (at == std::string::npos) {
            return;
        }
        const auto end = text.find('>', at);
        if (end == std::string::npos) {
            return;
        }
        auto attributes = readAttributes(text.substr(at, end - at + 1));
        if (attributes.count("name")) {
            std::string value = attributes.count("value") ? attributes.at("value") : attributes.count("default") ? attributes.at("default") : "";
            value = substitute(value, symbols);
            symbols[attributes.at("name")] = value;
        }
        text.erase(at, end - at + 1);
    }
}

bool expandMacros(std::string& text, const std::vector<Macro>& macros, const std::map<std::string, std::string>& symbols, std::string& error) {
    for (int pass = 0; pass < 16; ++pass) {
        bool replaced = false;
        for (const Macro& macro : macros) {
            const std::string open = "<xacro:" + macro.name;
            const auto at = text.find(open);
            if (at == std::string::npos) {
                continue;
            }
            const auto tagEnd = text.find('>', at);
            if (tagEnd == std::string::npos) {
                error = "xacro call " + macro.name + " is not closed";
                return false;
            }
            const bool empty = tagEnd > 0 && text[tagEnd - 1] == '/';
            const auto attributes = readAttributes(text.substr(at, tagEnd - at + 1));
            std::size_t eraseEnd = tagEnd + 1;
            if (!empty) {
                const std::string closing = "</xacro:" + macro.name + ">";
                const auto close = text.find(closing, tagEnd);
                if (close == std::string::npos) {
                    error = "xacro call " + macro.name + " is not closed";
                    return false;
                }
                eraseEnd = close + closing.size();
            }
            std::map<std::string, std::string> local = symbols;
            for (const auto& param : macro.params) {
                local[param.first] = substitute(param.second, symbols);
            }
            for (const auto& attribute : attributes) {
                if (attribute.first == "name" || attribute.first.rfind("xmlns", 0) == 0) {
                    continue;
                }
                local[attribute.first] = substitute(attribute.second, local);
            }
            text.replace(at, eraseEnd - at, substitute(macro.body, local));
            replaced = true;
            break;
        }
        if (!replaced) {
            error.clear();
            return true;
        }
    }
    error = "xacro macros are nested too deeply";
    return false;
}

}  // namespace

bool expandXacroDocument(const std::string& text, const std::string& baseDir, std::string& expanded, std::string& error) {
    if (text.find("xacro:") == std::string::npos) {
        expanded = text;
        error.clear();
        return true;
    }
    expanded = text;
    std::string dir = baseDir;
    if (!dir.empty() && dir.back() != '/' && dir.back() != '\\') {
        dir.push_back('/');
    }
    if (!expandIncludes(expanded, dir, error)) {
        return false;
    }
    std::map<std::string, std::string> symbols;
    takeProperties(expanded, symbols);
    std::vector<Macro> macros;
    takeMacros(expanded, macros);
    if (!expandMacros(expanded, macros, symbols, error)) {
        return false;
    }
    expanded = substitute(expandFind(expanded, dir), symbols);
    error.clear();
    return true;
}

}  // namespace sim
