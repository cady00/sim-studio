#pragma once

#include <cctype>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace sim {

struct XmlNode {
    std::string name;
    std::map<std::string, std::string> attributes;
    std::vector<XmlNode> children;
    std::string text;
    // Exact source of this element, from '<' through the closing tag. Comments use the same field.
    std::string raw;
};

// A small XML reader for URDF and PPR documents. It skips declarations, comments,
// and namespaces on the element name. It does not expand entities beyond the five XML builtins.
inline bool parseXml(const std::string& text, XmlNode& root, std::string& error) {
    std::size_t cursor = 0;
    const auto skip = [&] {
        while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) {
            ++cursor;
        }
    };
    const auto decode = [](std::string value) {
        std::string out;
        for (std::size_t i = 0; i < value.size(); ++i) {
            if (value[i] == '&') {
                if (value.compare(i, 4, "&lt;") == 0) {
                    out.push_back('<');
                    i += 3;
                } else if (value.compare(i, 4, "&gt;") == 0) {
                    out.push_back('>');
                    i += 3;
                } else if (value.compare(i, 5, "&amp;") == 0) {
                    out.push_back('&');
                    i += 4;
                } else if (value.compare(i, 6, "&quot;") == 0) {
                    out.push_back('"');
                    i += 5;
                } else if (value.compare(i, 6, "&apos;") == 0) {
                    out.push_back('\'');
                    i += 5;
                } else {
                    out.push_back('&');
                }
            } else {
                out.push_back(value[i]);
            }
        }
        return out;
    };
    std::function<bool(XmlNode&)> read = [&](XmlNode& node) {
        skip();
        while (cursor + 4 <= text.size() && text.compare(cursor, 4, "<!--") == 0) {
            const auto end = text.find("-->", cursor + 4);
            if (end == std::string::npos) {
                error = "xml comment is not closed";
                return false;
            }
            cursor = end + 3;
            skip();
        }
        if (cursor + 2 <= text.size() && text.compare(cursor, 2, "<?") == 0) {
            const auto end = text.find("?>", cursor + 2);
            if (end == std::string::npos) {
                error = "xml declaration is not closed";
                return false;
            }
            cursor = end + 2;
            skip();
        }
        if (cursor >= text.size() || text[cursor] != '<') {
            error = "xml element was expected";
            return false;
        }
        const std::size_t elementStart = cursor;
        ++cursor;
        if (cursor < text.size() && (text[cursor] == '/' || text[cursor] == '!')) {
            error = "xml element was expected";
            return false;
        }
        const std::size_t nameStart = cursor;
        while (cursor < text.size() && !std::isspace(static_cast<unsigned char>(text[cursor])) && text[cursor] != '>' &&
               text[cursor] != '/') {
            ++cursor;
        }
        node.name = text.substr(nameStart, cursor - nameStart);
        if (node.name.empty()) {
            error = "xml element has no name";
            return false;
        }
        while (cursor < text.size() && text[cursor] != '>' && text[cursor] != '/') {
            skip();
            if (cursor >= text.size() || text[cursor] == '>' || text[cursor] == '/') {
                break;
            }
            const std::size_t keyStart = cursor;
            while (cursor < text.size() && text[cursor] != '=' && !std::isspace(static_cast<unsigned char>(text[cursor]))) {
                ++cursor;
            }
            const std::string key = text.substr(keyStart, cursor - keyStart);
            skip();
            if (cursor >= text.size() || text[cursor] != '=') {
                error = "xml attribute " + key + " has no value";
                return false;
            }
            ++cursor;
            skip();
            if (cursor >= text.size() || (text[cursor] != '"' && text[cursor] != '\'')) {
                error = "xml attribute " + key + " is not quoted";
                return false;
            }
            const char quote = text[cursor++];
            const std::size_t valueStart = cursor;
            while (cursor < text.size() && text[cursor] != quote) {
                ++cursor;
            }
            if (cursor >= text.size()) {
                error = "xml attribute " + key + " is not closed";
                return false;
            }
            node.attributes[key] = decode(text.substr(valueStart, cursor - valueStart));
            ++cursor;
        }
        if (cursor < text.size() && text[cursor] == '/') {
            ++cursor;
            if (cursor >= text.size() || text[cursor] != '>') {
                error = "xml empty element is not closed";
                return false;
            }
            ++cursor;
            node.raw = text.substr(elementStart, cursor - elementStart);
            return true;
        }
        if (cursor >= text.size() || text[cursor] != '>') {
            error = "xml element " + node.name + " is not closed";
            return false;
        }
        ++cursor;
        std::string textRun;
        while (cursor < text.size()) {
            if (text.compare(cursor, 4, "<!--") == 0) {
                const auto end = text.find("-->", cursor + 4);
                if (end == std::string::npos) {
                    error = "xml comment is not closed";
                    return false;
                }
                XmlNode comment;
                comment.name = "#comment";
                comment.raw = text.substr(cursor, end + 3 - cursor);
                comment.text = text.substr(cursor + 4, end - (cursor + 4));
                node.children.push_back(std::move(comment));
                cursor = end + 3;
                continue;
            }
            if (text.compare(cursor, 2, "</") == 0) {
                cursor += 2;
                const std::string closing = node.name;
                if (text.compare(cursor, closing.size(), closing) != 0) {
                    error = "xml element " + node.name + " closed by a different name";
                    return false;
                }
                cursor += closing.size();
                skip();
                if (cursor >= text.size() || text[cursor] != '>') {
                    error = "xml element " + node.name + " is not closed";
                    return false;
                }
                ++cursor;
                node.text = decode(textRun);
                node.raw = text.substr(elementStart, cursor - elementStart);
                return true;
            }
            if (text[cursor] == '<') {
                XmlNode child;
                if (!read(child)) {
                    return false;
                }
                node.children.push_back(std::move(child));
                continue;
            }
            textRun.push_back(text[cursor]);
            ++cursor;
        }
        error = "xml element " + node.name + " is not closed";
        return false;
    };
    if (!read(root)) {
        return false;
    }
    error.clear();
    return true;
}

}  // namespace sim
