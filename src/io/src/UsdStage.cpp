#include "sim/io/AssetImport.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace sim {
namespace {

struct ParseError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

std::string num(double value) {
    if (!std::isfinite(value)) {
        value = 0;
    }
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    return buffer;
}

std::string escapeUsda(const std::string& text) {
    std::string out;
    for (char character : text) {
        if (character == '\\' || character == '"') {
            out.push_back('\\');
            out.push_back(character);
        } else if (character == '\n') {
            out += "\\n";
        } else {
            out.push_back(character);
        }
    }
    return out;
}

std::string usdIdent(const std::string& name, std::unordered_set<std::string>& used) {
    std::string out;
    for (unsigned char character : name) {
        if (std::isalnum(character) || character == '_') {
            out.push_back(static_cast<char>(character));
        } else {
            out.push_back('_');
        }
    }
    if (out.empty() || std::isdigit(static_cast<unsigned char>(out.front()))) {
        out.insert(0, "n_");
    }
    std::string unique = out;
    int suffix = 2;
    while (!used.insert(unique).second) {
        unique = out + "_" + std::to_string(suffix++);
    }
    return unique;
}

const char* axisToken(Vec3 axis) {
    const Vec3 direction = normalize(axis);
    const double ax = std::abs(direction.x);
    const double ay = std::abs(direction.y);
    const double az = std::abs(direction.z);
    if (ay >= ax && ay >= az) {
        return "Y";
    }
    if (az >= ax && az >= ay) {
        return "Z";
    }
    return "X";
}

const char* approximationToken(CollisionShape shape) {
    switch (shape) {
    case CollisionShape::Sphere:
        return "boundingSphere";
    case CollisionShape::Box:
        return "boundingCube";
    case CollisionShape::Mesh:
        return "mesh";
    case CollisionShape::ConvexHull:
    case CollisionShape::Capsule:
    case CollisionShape::Cylinder:
    case CollisionShape::Extrusion:
        return "convexHull";
    }
    return "convexHull";
}

const char* jointTypeName(MechKind kind) {
    switch (kind) {
    case MechKind::Hinge:
        return "PhysicsRevoluteJoint";
    case MechKind::Slider:
        return "PhysicsPrismaticJoint";
    case MechKind::Ball:
        return "PhysicsSphericalJoint";
    case MechKind::FixedJoint:
        return "PhysicsFixedJoint";
    default:
        return nullptr;
    }
}

Quat quatFromBasis(Vec3 x, Vec3 y, Vec3 z) {
    const double trace = x.x + y.y + z.z;
    Quat rotation;
    if (trace > 0) {
        const double scale = std::sqrt(trace + 1.0) * 2;
        rotation.w = 0.25 * scale;
        rotation.x = (y.z - z.y) / scale;
        rotation.y = (z.x - x.z) / scale;
        rotation.z = (x.y - y.x) / scale;
    } else if (x.x > y.y && x.x > z.z) {
        const double scale = std::sqrt(1.0 + x.x - y.y - z.z) * 2;
        rotation.w = (y.z - z.y) / scale;
        rotation.x = 0.25 * scale;
        rotation.y = (y.x + x.y) / scale;
        rotation.z = (z.x + x.z) / scale;
    } else if (y.y > z.z) {
        const double scale = std::sqrt(1.0 + y.y - x.x - z.z) * 2;
        rotation.w = (z.x - x.z) / scale;
        rotation.x = (y.x + x.y) / scale;
        rotation.y = 0.25 * scale;
        rotation.z = (z.y + y.z) / scale;
    } else {
        const double scale = std::sqrt(1.0 + z.z - x.x - y.y) * 2;
        rotation.w = (x.y - y.x) / scale;
        rotation.x = (z.x + x.z) / scale;
        rotation.y = (z.y + y.z) / scale;
        rotation.z = 0.25 * scale;
    }
    return rotation.normalized();
}

Transform transformFromMatrix(const Mat4& matrix) {
    Transform transform;
    transform.translation = {matrix(3, 0), matrix(3, 1), matrix(3, 2)};
    Vec3 x{matrix(0, 0), matrix(0, 1), matrix(0, 2)};
    Vec3 y{matrix(1, 0), matrix(1, 1), matrix(1, 2)};
    Vec3 z{matrix(2, 0), matrix(2, 1), matrix(2, 2)};
    transform.scale = {length(x), length(y), length(z)};
    if (transform.scale.x > 1e-12) {
        x = x / transform.scale.x;
    } else {
        x = {1, 0, 0};
    }
    if (transform.scale.y > 1e-12) {
        y = y / transform.scale.y;
    } else {
        y = {0, 1, 0};
    }
    if (transform.scale.z > 1e-12) {
        z = z / transform.scale.z;
    } else {
        z = {0, 0, 1};
    }
    if (dot(cross(x, y), z) < 0) {
        x = x * -1.0;
        transform.scale.x *= -1;
    }
    transform.rotation = quatFromBasis(x, y, z);
    return transform;
}

struct Value {
    enum class Kind { Number, Text, Ident, List, Dict, Path };
    Kind kind = Kind::Ident;
    double number = 0;
    bool tuple = false;
    std::string text;
    std::vector<Value> items;
    std::vector<std::pair<double, Value>> entries;
};

class Reader {
public:
    explicit Reader(const std::string& text) : text_(text) {
        if (text_.size() >= 3 && static_cast<unsigned char>(text_[0]) == 0xEF &&
            static_cast<unsigned char>(text_[1]) == 0xBB && static_cast<unsigned char>(text_[2]) == 0xBF) {
            index_ = 3;
        }
    }

    int line() const { return line_; }
    bool eof() const { return index_ >= text_.size(); }
    std::vector<std::string>& notes() { return notes_; }

    void skipSpace() {
        while (index_ < text_.size()) {
            const char character = text_[index_];
            if (character == '#') {
                while (index_ < text_.size() && text_[index_] != '\n') {
                    ++index_;
                }
                continue;
            }
            if (character == ' ' || character == '\t' || character == '\r') {
                ++index_;
                continue;
            }
            if (character == '\n') {
                ++index_;
                ++line_;
                continue;
            }
            break;
        }
    }

    void skipInline() {
        while (index_ < text_.size()) {
            const char character = text_[index_];
            if (character == ' ' || character == '\t' || character == '\r') {
                ++index_;
                continue;
            }
            if (character == '#') {
                while (index_ < text_.size() && text_[index_] != '\n') {
                    ++index_;
                }
                continue;
            }
            break;
        }
    }

    char peekInline() {
        skipInline();
        return index_ < text_.size() ? text_[index_] : 0;
    }

    char peek() {
        skipSpace();
        return index_ < text_.size() ? text_[index_] : 0;
    }

    bool consume(char character) {
        if (peek() != character) {
            return false;
        }
        ++index_;
        return true;
    }

    void expect(char character) {
        if (!consume(character)) {
            throw ParseError(std::string("expected '") + character + "' at line " + std::to_string(line_));
        }
    }

    std::string readToken() {
        skipSpace();
        if (eof()) {
            throw ParseError("unexpected end of USDA at line " + std::to_string(line_));
        }
        const std::size_t start = index_;
        const unsigned char first = static_cast<unsigned char>(text_[index_]);
        if (!(std::isalnum(first) || text_[index_] == '_' || text_[index_] == ':' || text_[index_] == '.' || text_[index_] == '[' ||
              text_[index_] == ']')) {
            throw ParseError("expected a word at line " + std::to_string(line_));
        }
        while (index_ < text_.size()) {
            const unsigned char character = static_cast<unsigned char>(text_[index_]);
            if (std::isalnum(character) || text_[index_] == '_' || text_[index_] == ':' || text_[index_] == '.' || text_[index_] == '[' ||
                text_[index_] == ']') {
                ++index_;
            } else {
                break;
            }
        }
        return text_.substr(start, index_ - start);
    }

    std::string peekWord() {
        const std::size_t saved = index_;
        const int savedLine = line_;
        std::string word;
        try {
            word = readToken();
        } catch (const ParseError&) {
            index_ = saved;
            line_ = savedLine;
            return {};
        }
        index_ = saved;
        line_ = savedLine;
        return word;
    }

    Value parseValue() {
        if (++depth_ > 64) {
            throw ParseError("USDA nesting is too deep");
        }
        struct Depth {
            int& depth;
            ~Depth() { --depth; }
        } guard{depth_};
        skipSpace();
        const char character = peek();
        if (character == '"') {
            ++index_;
            Value value;
            value.kind = Value::Kind::Text;
            value.text = readString();
            return value;
        }
        if (character == '<') {
            Value value;
            value.kind = Value::Kind::Path;
            value.text = readPath();
            return value;
        }
        if (character == '@') {
            Value value;
            value.kind = Value::Kind::Text;
            value.text = readAsset();
            return value;
        }
        if (character == '(' || character == '[') {
            const bool tuple = character == '(';
            const char close = tuple ? ')' : ']';
            ++index_;
            Value value;
            value.kind = Value::Kind::List;
            value.tuple = tuple;
            value.items = parseList(close);
            return value;
        }
        if (character == '{') {
            return parseDict();
        }
        double number = 0;
        if (parseNumber(number)) {
            Value value;
            value.kind = Value::Kind::Number;
            value.number = number;
            return value;
        }
        Value value;
        value.kind = Value::Kind::Ident;
        value.text = readToken();
        return value;
    }

    void skipGroup(char open, char close) {
        expect(open);
        int depth = 1;
        bool inString = false;
        while (index_ < text_.size() && depth > 0) {
            const char character = text_[index_++];
            if (character == '\n') {
                ++line_;
            }
            if (inString) {
                if (character == '\\' && index_ < text_.size()) {
                    ++index_;
                } else if (character == '"') {
                    inString = false;
                }
                continue;
            }
            if (character == '"') {
                inString = true;
            } else if (character == '#') {
                while (index_ < text_.size() && text_[index_] != '\n') {
                    ++index_;
                }
            } else if (character == open) {
                ++depth;
            } else if (character == close) {
                --depth;
            }
        }
        if (depth != 0) {
            throw ParseError("unbalanced group at line " + std::to_string(line_));
        }
    }

private:
    bool parseNumber(double& out) {
        skipSpace();
        if (eof()) {
            return false;
        }
        const char character = text_[index_];
        if (!(std::isdigit(static_cast<unsigned char>(character)) || character == '+' || character == '-' || character == '.')) {
            return false;
        }
        if (character == '+' || character == '-' || character == '.') {
            if (index_ + 1 >= text_.size()) {
                return false;
            }
            const char next = text_[index_ + 1];
            if (character == '.' && !std::isdigit(static_cast<unsigned char>(next))) {
                return false;
            }
            if ((character == '+' || character == '-') && !(std::isdigit(static_cast<unsigned char>(next)) || next == '.')) {
                return false;
            }
        }
        char* end = nullptr;
        const double value = std::strtod(text_.c_str() + index_, &end);
        if (end == text_.c_str() + index_) {
            return false;
        }
        index_ = static_cast<std::size_t>(end - text_.c_str());
        out = value;
        return true;
    }

    std::string readString() {
        std::string out;
        while (index_ < text_.size()) {
            const char character = text_[index_++];
            if (character == '\n') {
                ++line_;
            }
            if (character == '\\' && index_ < text_.size()) {
                const char escaped = text_[index_++];
                if (escaped == 'n') {
                    out.push_back('\n');
                } else {
                    out.push_back(escaped);
                }
                continue;
            }
            if (character == '"') {
                return out;
            }
            out.push_back(character);
        }
        throw ParseError("unterminated string at line " + std::to_string(line_));
    }

    std::string readPath() {
        expect('<');
        const std::size_t start = index_;
        while (index_ < text_.size() && text_[index_] != '>') {
            if (text_[index_] == '\n') {
                ++line_;
            }
            ++index_;
        }
        if (index_ >= text_.size()) {
            throw ParseError("unterminated path at line " + std::to_string(line_));
        }
        std::string path = text_.substr(start, index_ - start);
        ++index_;
        const auto first = path.find_first_not_of(" \t");
        const auto last = path.find_last_not_of(" \t");
        if (first == std::string::npos) {
            return {};
        }
        return path.substr(first, last - first + 1);
    }

    std::string readAsset() {
        expect('@');
        std::string out;
        while (index_ < text_.size() && text_[index_] != '@') {
            out.push_back(text_[index_++]);
        }
        if (index_ >= text_.size()) {
            throw ParseError("unterminated asset path at line " + std::to_string(line_));
        }
        ++index_;
        return out;
    }

    std::vector<Value> parseList(char close) {
        std::vector<Value> items;
        while (!eof()) {
            skipSpace();
            if (peek() == close) {
                ++index_;
                return items;
            }
            items.push_back(parseValue());
            consume(',');
        }
        throw ParseError("unterminated list at line " + std::to_string(line_));
    }

    Value parseDict() {
        expect('{');
        Value value;
        value.kind = Value::Kind::Dict;
        while (!eof()) {
            skipSpace();
            if (consume('}')) {
                return value;
            }
            double key = 0;
            if (!parseNumber(key)) {
                throw ParseError("time sample key is not a number at line " + std::to_string(line_));
            }
            expect(':');
            value.entries.emplace_back(key, parseValue());
            consume(',');
        }
        throw ParseError("unterminated time samples at line " + std::to_string(line_));
    }

    const std::string& text_;
    std::size_t index_ = 0;
    int line_ = 1;
    int depth_ = 0;
    std::vector<std::string> notes_;
};

bool isQualifier(const std::string& word) {
    return word == "uniform" || word == "custom" || word == "varying" || word == "prepend" || word == "append" || word == "delete";
}

std::string valueText(const Value& value) {
    if (value.kind == Value::Kind::Text || value.kind == Value::Kind::Ident || value.kind == Value::Kind::Path) {
        return value.text;
    }
    return {};
}

bool valueNumber(const Value& value, double& out) {
    if (value.kind != Value::Kind::Number || !std::isfinite(value.number)) {
        return false;
    }
    out = value.number;
    return true;
}

bool valueBool(const Value& value, bool& out) {
    if (value.kind == Value::Kind::Number) {
        out = value.number != 0;
        return true;
    }
    const std::string text = valueText(value);
    if (text == "true" || text == "True") {
        out = true;
        return true;
    }
    if (text == "false" || text == "False") {
        out = false;
        return true;
    }
    return false;
}

bool valueVec3(const Value& value, Vec3& out) {
    if (value.kind != Value::Kind::List || value.items.size() != 3) {
        return false;
    }
    double x = 0;
    double y = 0;
    double z = 0;
    if (!valueNumber(value.items[0], x) || !valueNumber(value.items[1], y) || !valueNumber(value.items[2], z)) {
        return false;
    }
    out = {x, y, z};
    return true;
}

struct Sample {
    double code = 0;
    bool hasTranslate = false;
    Vec3 translate{};
    bool hasJoints = false;
    std::vector<double> joints;
};

struct Prim {
    std::string type;
    std::string name;
    std::string path;
    std::vector<std::string> apis;
    std::vector<std::string> xformOrder;
    bool hasMatrix = false;
    Mat4 matrix = Mat4::identity();
    bool hasTranslate = false;
    Vec3 translate{};
    std::vector<Sample> samples;
    std::vector<Vec3> points;
    std::vector<Vec3> normals;
    std::vector<int> faceCounts;
    std::vector<int> indices;
    bool hasColor = false;
    Vec3 color{0.55, 0.58, 0.66};
    bool hasMetallic = false;
    float metallic = 0;
    bool hasRoughness = false;
    float roughness = 1;
    bool preview = false;
    bool hasMass = false;
    double mass = 1;
    bool hasDensity = false;
    double density = 900;
    bool hasAutoMass = false;
    bool autoMass = true;
    bool hasRigid = false;
    bool rigid = false;
    bool hasKinematic = false;
    bool kinematic = false;
    bool hasCollision = false;
    bool collision = false;
    std::string approximation;
    std::string shape;
    std::string mech;
    std::string motion;
    std::string other;
    bool hasAxis = false;
    Vec3 axis{1, 0, 0};
    std::string axisToken;
    bool hasLimits = false;
    double lower = 0;
    double upper = 0;
    bool hasHalf = false;
    Vec3 half{25, 25, 25};
    bool hasRadius = false;
    double radius = 25;
    bool hasHeight = false;
    double height = 50;
    std::string role;
    std::string resource;
    std::string simName;
    std::string simKind;
    bool hasVisible = false;
    bool visible = true;
    std::string body0;
    std::string body1;
    bool hasFriction = false;
    double friction = 0.4;
    bool hasRestitution = false;
    double restitution = 0.05;
    std::vector<Prim> children;
};

struct Stage {
    double metersPerUnit = -1;
    double framesPerSecond = -1;
    std::string upAxis;
    std::string defaultPrim;
    bool hasPreview = false;
    std::vector<std::string> notes;
    std::vector<Prim> roots;
};

Sample& sampleAt(Prim& prim, double code) {
    for (Sample& sample : prim.samples) {
        if (std::abs(sample.code - code) < 1e-6) {
            return sample;
        }
    }
    Sample sample;
    sample.code = code;
    prim.samples.push_back(std::move(sample));
    return prim.samples.back();
}

bool asMatrix(const Value& value, Mat4& matrix) {
    if (value.kind != Value::Kind::List || value.items.size() != 4) {
        return false;
    }
    double usd[16];
    int count = 0;
    for (const Value& row : value.items) {
        if (row.kind != Value::Kind::List || row.items.size() != 4) {
            return false;
        }
        for (const Value& cell : row.items) {
            if (!valueNumber(cell, usd[count])) {
                return false;
            }
            ++count;
        }
    }
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            matrix(column, row) = usd[column * 4 + row];
        }
    }
    return true;
}

void applyAttribute(Prim& prim, Stage& stage, const std::string& name, const Value& value) {
    std::string base = name;
    bool timed = false;
    constexpr const char* kTimed = ".timeSamples";
    if (base.size() >= 12 && base.compare(base.size() - 12, 12, kTimed) == 0) {
        timed = true;
        base.resize(base.size() - 12);
    }
    if (base == "xformOp:transform" && !timed && asMatrix(value, prim.matrix)) {
        prim.hasMatrix = true;
        return;
    }
    if (base == "xformOp:translate") {
        if (timed && value.kind == Value::Kind::Dict) {
            for (const auto& entry : value.entries) {
                Vec3 translation;
                if (valueVec3(entry.second, translation)) {
                    Sample& sample = sampleAt(prim, entry.first);
                    sample.hasTranslate = true;
                    sample.translate = translation;
                }
            }
        } else if (valueVec3(value, prim.translate)) {
            prim.hasTranslate = true;
        }
        return;
    }
    if (base == "xformOpOrder" && value.kind == Value::Kind::List) {
        for (const Value& item : value.items) {
            const std::string text = valueText(item);
            if (!text.empty()) {
                prim.xformOrder.push_back(text);
            }
        }
        return;
    }
    if (base == "faceVertexCounts" && value.kind == Value::Kind::List) {
        for (const Value& item : value.items) {
            double number = 0;
            if (valueNumber(item, number)) {
                prim.faceCounts.push_back(static_cast<int>(number));
            }
        }
        return;
    }
    if (base == "faceVertexIndices" && value.kind == Value::Kind::List) {
        for (const Value& item : value.items) {
            double number = 0;
            if (valueNumber(item, number)) {
                prim.indices.push_back(static_cast<int>(number));
            }
        }
        return;
    }
    if ((base == "points" || base == "normals") && value.kind == Value::Kind::List) {
        std::vector<Vec3>& destination = base == "points" ? prim.points : prim.normals;
        for (const Value& item : value.items) {
            Vec3 point;
            if (valueVec3(item, point)) {
                destination.push_back(point);
            }
        }
        return;
    }
    if ((base == "primvars:displayColor" || base == "inputs:diffuseColor" || base == "sim:color") && !timed) {
        Vec3 color;
        if (value.kind == Value::Kind::List && !value.items.empty() && valueVec3(value.items.front(), color)) {
            prim.color = color;
            prim.hasColor = true;
        } else if (valueVec3(value, color)) {
            prim.color = color;
            prim.hasColor = true;
        }
        return;
    }
    if (base == "inputs:metallic" || base == "sim:metallic") {
        double number = 0;
        if (valueNumber(value, number)) {
            prim.metallic = static_cast<float>(number);
            prim.hasMetallic = true;
        }
        return;
    }
    if (base == "inputs:roughness" || base == "sim:roughness") {
        double number = 0;
        if (valueNumber(value, number)) {
            prim.roughness = static_cast<float>(number);
            prim.hasRoughness = true;
        }
        return;
    }
    if (base == "info:id" && valueText(value) == "UsdPreviewSurface") {
        prim.preview = true;
        stage.hasPreview = true;
        return;
    }
    if (base == "physics:mass" || base == "sim:mass") {
        if (valueNumber(value, prim.mass)) {
            prim.hasMass = true;
        }
        return;
    }
    if (base == "physics:density") {
        if (valueNumber(value, prim.density)) {
            prim.hasDensity = true;
        }
        return;
    }
    if (base == "sim:autoMass") {
        if (valueBool(value, prim.autoMass)) {
            prim.hasAutoMass = true;
        }
        return;
    }
    if (base == "physics:rigidBodyEnabled") {
        if (valueBool(value, prim.rigid)) {
            prim.hasRigid = true;
        }
        return;
    }
    if (base == "physics:kinematicEnabled") {
        if (valueBool(value, prim.kinematic)) {
            prim.hasKinematic = true;
        }
        return;
    }
    if (base == "physics:collisionEnabled") {
        if (valueBool(value, prim.collision)) {
            prim.hasCollision = true;
        }
        return;
    }
    if (base == "physics:approximation") {
        prim.approximation = valueText(value);
        return;
    }
    if (base == "physics:lowerLimit" || base == "sim:lower") {
        double number = 0;
        if (valueNumber(value, number)) {
            prim.lower = number;
            prim.hasLimits = true;
        }
        return;
    }
    if (base == "physics:upperLimit" || base == "sim:upper") {
        double number = 0;
        if (valueNumber(value, number)) {
            prim.upper = number;
            prim.hasLimits = true;
        }
        return;
    }
    if (base == "physics:axis") {
        prim.axisToken = valueText(value);
        return;
    }
    if (base == "physics:body0") {
        prim.body0 = value.kind == Value::Kind::List && !value.items.empty() ? valueText(value.items.front()) : valueText(value);
        return;
    }
    if (base == "physics:body1") {
        prim.body1 = value.kind == Value::Kind::List && !value.items.empty() ? valueText(value.items.front()) : valueText(value);
        return;
    }
    if (base == "physics:dynamicFriction") {
        if (valueNumber(value, prim.friction)) {
            prim.hasFriction = true;
        }
        return;
    }
    if (base == "physics:restitution") {
        if (valueNumber(value, prim.restitution)) {
            prim.hasRestitution = true;
        }
        return;
    }
    if (base == "sim:joints" && timed && value.kind == Value::Kind::Dict) {
        for (const auto& entry : value.entries) {
            if (entry.second.kind != Value::Kind::List) {
                continue;
            }
            Sample& sample = sampleAt(prim, entry.first);
            sample.hasJoints = true;
            for (const Value& item : entry.second.items) {
                double number = 0;
                if (valueNumber(item, number)) {
                    sample.joints.push_back(number);
                }
            }
        }
        return;
    }
    if (base == "sim:role") {
        prim.role = valueText(value);
        return;
    }
    if (base == "sim:resource") {
        prim.resource = valueText(value);
        return;
    }
    if (base == "sim:name") {
        prim.simName = valueText(value);
        return;
    }
    if (base == "sim:kind") {
        prim.simKind = valueText(value);
        return;
    }
    if (base == "sim:shape") {
        prim.shape = valueText(value);
        return;
    }
    if (base == "sim:mech") {
        prim.mech = valueText(value);
        return;
    }
    if (base == "sim:motion") {
        prim.motion = valueText(value);
        return;
    }
    if (base == "sim:other") {
        prim.other = valueText(value);
        return;
    }
    if (base == "sim:axis" && valueVec3(value, prim.axis)) {
        prim.hasAxis = true;
        return;
    }
    if (base == "sim:halfExtents" && valueVec3(value, prim.half)) {
        prim.hasHalf = true;
        return;
    }
    if (base == "sim:radius") {
        if (valueNumber(value, prim.radius)) {
            prim.hasRadius = true;
        }
        return;
    }
    if (base == "sim:height") {
        if (valueNumber(value, prim.height)) {
            prim.hasHeight = true;
        }
        return;
    }
    if (base == "sim:visible") {
        if (valueBool(value, prim.visible)) {
            prim.hasVisible = true;
        }
        return;
    }
}

class Parser {
public:
    explicit Parser(const std::string& text) : reader_(text) {}

    Stage parse() {
        Stage stage;
        if (reader_.peek() == '(') {
            parsePreamble(stage);
        }
        while (reader_.peek() != 0) {
            const std::string word = reader_.peekWord();
            if (word == "def" || word == "over" || word == "class") {
                stage.roots.push_back(parsePrim(stage, ""));
            } else if (word.empty()) {
                break;
            } else {
                throw ParseError("unexpected '" + word + "' at line " + std::to_string(reader_.line()));
            }
        }
        stage.notes.insert(stage.notes.end(), reader_.notes().begin(), reader_.notes().end());
        return stage;
    }

private:
    void skipQualifiers() {
        while (isQualifier(reader_.peekWord())) {
            reader_.readToken();
        }
    }

    void parsePreamble(Stage& stage) {
        reader_.expect('(');
        while (reader_.peek() != ')' && reader_.peek() != 0) {
            skipQualifiers();
            if (reader_.peek() == ')') {
                break;
            }
            const std::string name = reader_.readToken();
            reader_.expect('=');
            const Value value = reader_.parseValue();
            if (name == "defaultPrim") {
                stage.defaultPrim = valueText(value);
            } else if (name == "metersPerUnit") {
                valueNumber(value, stage.metersPerUnit);
            } else if (name == "upAxis") {
                stage.upAxis = valueText(value);
            } else if (name == "timeCodesPerSecond") {
                valueNumber(value, stage.framesPerSecond);
            } else if (name == "subLayers") {
                stage.notes.push_back("subLayers were not composed");
            }
            if (reader_.peek() == '(') {
                reader_.skipGroup('(', ')');
            }
        }
        reader_.expect(')');
    }

    void parseMetadata(Stage& stage, Prim& prim) {
        while (reader_.peek() != ')' && reader_.peek() != 0) {
            skipQualifiers();
            if (reader_.peek() == ')') {
                break;
            }
            const std::string first = reader_.readToken();
            reader_.skipSpace();
            std::string name = first;
            if (reader_.peek() != '=') {
                name = reader_.readToken();
            }
            reader_.expect('=');
            const Value value = reader_.parseValue();
            if (name == "apiSchemas" && value.kind == Value::Kind::List) {
                for (const Value& item : value.items) {
                    const std::string text = valueText(item);
                    if (!text.empty()) {
                        prim.apis.push_back(text);
                    }
                }
            } else if (name == "references" || name == "payload" || name == "inherits" || name == "specializes") {
                stage.notes.push_back(name + " arcs were skipped");
            }
            if (reader_.peek() == '(') {
                reader_.skipGroup('(', ')');
            }
        }
    }

    Prim parsePrim(Stage& stage, const std::string& parent) {
        if (++prims_ > 100000) {
            throw ParseError("too many prims");
        }
        const std::string keyword = reader_.readToken();
        if (keyword != "def" && keyword != "over" && keyword != "class") {
            throw ParseError("expected def at line " + std::to_string(reader_.line()));
        }
        Prim prim;
        if (reader_.peek() == '"') {
            prim.type = "Xform";
        } else {
            prim.type = reader_.readToken();
        }
        const Value name = reader_.parseValue();
        prim.name = valueText(name);
        if (prim.name.empty()) {
            throw ParseError("empty prim name at line " + std::to_string(reader_.line()));
        }
        prim.path = parent.empty() ? "/" + prim.name : parent + "/" + prim.name;
        if (reader_.consume('(')) {
            parseMetadata(stage, prim);
            reader_.expect(')');
        }
        reader_.expect('{');
        while (reader_.peek() != '}') {
            const std::string word = reader_.peekWord();
            if (word == "def" || word == "over" || word == "class") {
                prim.children.push_back(parsePrim(stage, prim.path));
            } else if (word == "variantSet") {
                reader_.readToken();
                if (reader_.peek() == '"') {
                    reader_.parseValue();
                }
                if (reader_.consume('=')) {
                }
                if (reader_.peek() == '{') {
                    reader_.skipGroup('{', '}');
                }
                stage.notes.push_back("variantSet was skipped");
            } else if (word.empty()) {
                throw ParseError("unexpected end inside " + prim.path);
            } else {
                parseAttribute(stage, prim);
            }
        }
        reader_.expect('}');
        std::sort(prim.samples.begin(), prim.samples.end(),
                  [](const Sample& left, const Sample& right) { return left.code < right.code; });
        if (prim.preview) {
            stage.hasPreview = true;
        }
        return prim;
    }

    void parseAttribute(Stage& stage, Prim& prim) {
        skipQualifiers();
        const std::string first = reader_.readToken();
        const char ahead = reader_.peekInline();
        std::string name = first;
        if (ahead != '=' && ahead != '\n' && ahead != '}' && ahead != 0) {
            name = reader_.readToken();
        }
        if (reader_.peekInline() != '=') {
            return;
        }
        if (name.size() >= 8 && name.compare(name.size() - 8, 8, ".connect") == 0) {
            reader_.expect('=');
            reader_.parseValue();
            if (reader_.peek() == '(') {
                reader_.skipGroup('(', ')');
            }
            return;
        }
        reader_.expect('=');
        if (name == "references" || name == "payload" || name == "inherits" || name == "specializes") {
            stage.notes.push_back(name + " arcs were skipped");
        }
        const Value value = reader_.parseValue();
        if (reader_.peek() == '(') {
            reader_.skipGroup('(', ')');
        }
        applyAttribute(prim, stage, name, value);
    }

    Reader reader_;
    int prims_ = 0;
};

bool parseStage(const std::string& text, Stage& stage, std::string& error) {
    try {
        Parser parser(text);
        stage = parser.parse();
        return true;
    } catch (const ParseError& failure) {
        error = failure.what();
        return false;
    } catch (const std::exception& failure) {
        error = failure.what();
        return false;
    }
}

int validatePrim(const Prim& prim, std::string& error) {
    if (!prim.faceCounts.empty()) {
        long sum = 0;
        for (int count : prim.faceCounts) {
            if (count < 3) {
                error = prim.path + " has a face with fewer than 3 vertices";
                return -1;
            }
            sum += count;
        }
        if (sum != static_cast<long>(prim.indices.size())) {
            error = prim.path + " faceVertexIndices do not match faceVertexCounts";
            return -1;
        }
        if (prim.points.empty()) {
            error = prim.path + " has faces and no points";
            return -1;
        }
        int maxIndex = -1;
        for (int index : prim.indices) {
            maxIndex = std::max(maxIndex, index);
        }
        if (maxIndex >= static_cast<int>(prim.points.size())) {
            error = prim.path + " indexes a point that does not exist";
            return -1;
        }
    }
    if (prim.hasMass && !std::isfinite(prim.mass)) {
        error = prim.path + " has a non-finite mass";
        return -1;
    }
    int faces = prim.faceCounts.empty() ? 0 : static_cast<int>(prim.faceCounts.size());
    for (const Prim& child : prim.children) {
        const int childFaces = validatePrim(child, error);
        if (childFaces < 0) {
            return -1;
        }
        faces += childFaces;
    }
    return faces;
}

struct Out {
    std::ostringstream text;
    int indent = 0;

    void line(const std::string& body) { text << std::string(static_cast<std::size_t>(indent) * 4, ' ') << body << '\n'; }
};

void writeMatrix(Out& out, const Mat4& matrix) {
    out.text << "( ";
    for (int row = 0; row < 4; ++row) {
        out.text << "(" << num(matrix(row, 0)) << ", " << num(matrix(row, 1)) << ", " << num(matrix(row, 2)) << ", " << num(matrix(row, 3))
                 << ")";
        if (row + 1 < 4) {
            out.text << ", ";
        }
    }
    out.text << " )";
}

struct ExportItem {
    std::shared_ptr<SceneNode> node;
    std::string name;
    std::string path;
    std::vector<ExportItem> children;
};

void buildItems(const std::shared_ptr<SceneNode>& node, const std::string& parentPath, std::unordered_set<std::string>& siblings, ExportItem& item) {
    item.node = node;
    item.name = usdIdent(node ? node->name : "node", siblings);
    item.path = parentPath + "/" + item.name;
    std::unordered_set<std::string> childNames;
    if (node) {
        for (const auto& child : node->children) {
            ExportItem sub;
            buildItems(child, item.path, childNames, sub);
            item.children.push_back(std::move(sub));
        }
    }
}

const ExportItem* findNamed(const ExportItem& item, const std::string& name) {
    if (item.node && item.node->name == name) {
        return &item;
    }
    for (const ExportItem& child : item.children) {
        if (const ExportItem* found = findNamed(child, name)) {
            return found;
        }
    }
    return nullptr;
}

struct JointOut {
    std::string type;
    std::string name;
    std::string body0;
    std::string body1;
    Vec3 axis{1, 0, 0};
    bool limits = false;
    double lower = 0;
    double upper = 0;
};

void writeVec3(std::ostringstream& out, Vec3 value) {
    out << "(" << num(value.x) << ", " << num(value.y) << ", " << num(value.z) << ")";
}

void writeMesh(Out& out, const MeshData& mesh, const SceneNode& node, const std::string& primPath) {
    out.line("def Mesh \"Geometry\" (");
    out.indent++;
    out.line("prepend apiSchemas = [\"MaterialBindingAPI\"]");
    out.indent--;
    out.line(")");
    out.line("{");
    out.indent++;
    out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "int[] faceVertexCounts = [";
    const int triangles = static_cast<int>(mesh.triangleCount());
    for (int triangle = 0; triangle < triangles; ++triangle) {
        if (triangle) {
            out.text << ", ";
        }
        out.text << "3";
    }
    out.text << "]\n";
    out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "int[] faceVertexIndices = [";
    for (int triangle = 0; triangle < triangles; ++triangle) {
        for (int corner = 0; corner < 3; ++corner) {
            if (triangle || corner) {
                out.text << ", ";
            }
            out.text << mesh.indices[static_cast<std::size_t>(triangle * 3 + corner)];
        }
    }
    out.text << "]\n";
    out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "point3f[] points = [";
    const int vertices = static_cast<int>(mesh.vertexCount());
    for (int vertex = 0; vertex < vertices; ++vertex) {
        if (vertex) {
            out.text << ", ";
        }
        const std::size_t base = static_cast<std::size_t>(vertex) * 3;
        writeVec3(out.text, {mesh.positions[base], mesh.positions[base + 1], mesh.positions[base + 2]});
    }
    out.text << "]\n";
    if (mesh.normals.size() == mesh.positions.size() && !mesh.normals.empty()) {
        out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "normal3f[] normals = [";
        for (int vertex = 0; vertex < vertices; ++vertex) {
            if (vertex) {
                out.text << ", ";
            }
            const std::size_t base = static_cast<std::size_t>(vertex) * 3;
            writeVec3(out.text, {mesh.normals[base], mesh.normals[base + 1], mesh.normals[base + 2]});
        }
        out.text << "] (\n";
        out.line("    interpolation = \"vertex\"");
        out.line(")");
    }
    out.line("uniform token subdivisionScheme = \"none\"");
    out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "color3f[] primvars:displayColor = [";
    writeVec3(out.text, node.color);
    out.text << "] (\n";
    out.line("    interpolation = \"constant\"");
    out.line(")");
    out.line("rel material:binding = <" + primPath + "/Geometry/Preview>");
    out.line("def Material \"Preview\"");
    out.line("{");
    out.indent++;
    out.line("token outputs:surface.connect = <" + primPath + "/Geometry/Preview/Surface.outputs:surface>");
    out.line("def Shader \"Surface\"");
    out.line("{");
    out.indent++;
    out.line("uniform token info:id = \"UsdPreviewSurface\"");
    out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "color3f inputs:diffuseColor = ";
    writeVec3(out.text, node.color);
    out.text << "\n";
    out.line("float inputs:metallic = " + num(node.metallic));
    out.line("float inputs:roughness = " + num(node.roughness));
    out.line("token outputs:surface");
    out.indent--;
    out.line("}");
    out.indent--;
    out.line("}");
    out.indent--;
    out.line("}");
}

void writePhysics(Out& out, const SceneNode& node) {
    const bool kinematic = node.mech.kinematic || node.mech.motion == BodyMotion::Anchored || node.mech.motion == BodyMotion::Driven;
    out.line(std::string("bool physics:rigidBodyEnabled = ") + (mechCreatesBody(node.mech.kind) ? "true" : "false"));
    out.line(std::string("bool physics:kinematicEnabled = ") + (kinematic ? "true" : "false"));
    out.line(std::string("bool physics:collisionEnabled = ") + (node.mech.kind == MechKind::None ? "false" : "true"));
    out.line("float physics:mass = " + num(node.mech.mass));
    out.line("float physics:density = " + num(node.mech.density));
    out.line(std::string("uniform token physics:approximation = \"") + approximationToken(node.mech.shape) + "\"");
    out.line(std::string("custom string sim:mech = \"") + mechKindName(node.mech.kind) + "\"");
    out.line(std::string("custom string sim:motion = \"") + bodyMotionName(node.mech.motion) + "\"");
    out.line(std::string("custom string sim:shape = \"") + collisionShapeName(node.mech.shape) + "\"");
    out.line(std::string("custom bool sim:autoMass = ") + (node.mech.autoMass ? "true" : "false"));
    out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "custom float3 sim:halfExtents = ";
    writeVec3(out.text, node.mech.halfExtents);
    out.text << "\n";
    out.line("custom float sim:radius = " + num(node.mech.radius));
    out.line("custom float sim:height = " + num(node.mech.height));
    out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "custom float3 sim:axis = ";
    writeVec3(out.text, node.mech.axis);
    out.text << "\n";
    if (!node.mech.other.empty()) {
        out.line("custom string sim:other = \"" + escapeUsda(node.mech.other) + "\"");
    }
    if (node.mech.limited && mechIsJoint(node.mech.kind)) {
        out.line("custom float sim:lower = " + num(node.mech.lower));
        out.line("custom float sim:upper = " + num(node.mech.upper));
    }
}

void writeNode(Out& out, const ExportItem& item, const Scene& scene, const ExportItem& world, std::vector<JointOut>& joints, UsdIoResult& result) {
    const SceneNode& node = *item.node;
    const bool physics = mechCreatesBody(node.mech.kind);
    out.line("def Xform \"" + item.name + "\" (");
    if (physics) {
        out.indent++;
        out.line("prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\", \"PhysicsCollisionAPI\", \"PhysicsMeshCollisionAPI\"]");
        out.indent--;
    }
    out.line(")");
    out.line("{");
    out.indent++;
    if (item.name != node.name) {
        out.line("custom string sim:name = \"" + escapeUsda(node.name) + "\"");
    }
    out.line(std::string("custom string sim:kind = \"") + nodeKindName(node.kind) + "\"");
    if (!node.visible) {
        out.line("custom bool sim:visible = false");
    }
    out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "matrix4d xformOp:transform = ";
    writeMatrix(out, node.local.matrix());
    out.text << "\n";
    out.line("uniform token[] xformOpOrder = [\"xformOp:transform\"]");
    if (physics) {
        writePhysics(out, node);
        if (const char* type = jointTypeName(node.mech.kind)) {
            JointOut joint;
            joint.type = type;
            joint.name = item.name + "_joint";
            joint.body1 = item.path;
            joint.axis = node.mech.axis;
            joint.limits = node.mech.limited;
            joint.lower = node.mech.lower;
            joint.upper = node.mech.upper;
            if (!node.mech.other.empty()) {
                if (const ExportItem* other = findNamed(world, node.mech.other)) {
                    joint.body0 = other->path;
                }
            }
            if (joint.body0.empty()) {
                joint.body0 = item.path.substr(0, item.path.rfind('/'));
            }
            joints.push_back(std::move(joint));
            ++result.jointCount;
        }
    }
    if (!node.meshId.empty()) {
        if (const auto mesh = scene.mesh(node.meshId)) {
            if (!mesh->empty()) {
                writeMesh(out, *mesh, node, item.path);
                ++result.meshCount;
                result.triangleCount += static_cast<int>(mesh->triangleCount());
            }
        }
    }
    for (const ExportItem& child : item.children) {
        writeNode(out, child, scene, world, joints, result);
    }
    out.indent--;
    out.line("}");
    ++result.nodeCount;
}

void writeAnimations(Out& out, const Scene& scene, UsdIoResult& result, double& endCode) {
    if (scene.robotPaths.empty()) {
        return;
    }
    out.line("def Xform \"Animations\"");
    out.line("{");
    out.indent++;
    std::unordered_set<std::string> names;
    for (const RobotPath& path : scene.robotPaths) {
        const std::string name = usdIdent(path.name.empty() ? "path" : path.name, names);
        std::vector<double> codes;
        std::vector<Vec3> positions;
        std::vector<std::vector<double>> joints;
        double seconds = 0;
        for (std::size_t index = 0; index < path.points.size(); ++index) {
            const PathPoint& point = path.points[index];
            codes.push_back(seconds * 60.0);
            positions.push_back(point.cartesian ? point.position : Vec3{});
            joints.push_back(point.joints);
            if (index + 1 >= path.points.size()) {
                continue;
            }
            const PathPoint& next = path.points[index + 1];
            double delta = 0;
            if (point.cartesian && next.cartesian) {
                delta = length(next.position - point.position) / std::max(point.speed, 1.0);
            } else {
                const std::size_t count = std::max(point.joints.size(), next.joints.size());
                for (std::size_t joint = 0; joint < count; ++joint) {
                    const double left = joint < point.joints.size() ? point.joints[joint] : 0;
                    const double right = joint < next.joints.size() ? next.joints[joint] : 0;
                    delta = std::max(delta, std::abs(right - left));
                }
                delta /= std::max(point.speed, 1.0);
            }
            if (delta < 1.0 / 60.0) {
                delta = 1.0 / 60.0;
            }
            seconds += delta;
        }
        if (!codes.empty()) {
            endCode = std::max(endCode, codes.back());
        }
        result.sampleCount += static_cast<int>(codes.size());
        out.line("def Xform \"" + name + "\"");
        out.line("{");
        out.indent++;
        out.line("custom string sim:role = \"animation\"");
        out.line("custom string sim:resource = \"" + escapeUsda(path.resource) + "\"");
        if (name != path.name && !path.name.empty()) {
            out.line("custom string sim:name = \"" + escapeUsda(path.name) + "\"");
        }
        out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "double3 xformOp:translate.timeSamples = {\n";
        for (std::size_t index = 0; index < codes.size(); ++index) {
            out.text << std::string(static_cast<std::size_t>(out.indent + 1) * 4, ' ') << num(codes[index]) << ": ";
            writeVec3(out.text, positions[index]);
            out.text << ",\n";
        }
        out.line("}");
        out.line("uniform token[] xformOpOrder = [\"xformOp:translate\"]");
        bool anyJoints = false;
        for (const auto& row : joints) {
            anyJoints = anyJoints || !row.empty();
        }
        if (anyJoints) {
            out.text << std::string(static_cast<std::size_t>(out.indent) * 4, ' ') << "float[] sim:joints.timeSamples = {\n";
            for (std::size_t index = 0; index < codes.size(); ++index) {
                out.text << std::string(static_cast<std::size_t>(out.indent + 1) * 4, ' ') << num(codes[index]) << ": [";
                for (std::size_t joint = 0; joint < joints[index].size(); ++joint) {
                    if (joint) {
                        out.text << ", ";
                    }
                    out.text << num(joints[index][joint]);
                }
                out.text << "],\n";
            }
            out.line("}");
        }
        out.indent--;
        out.line("}");
    }
    out.indent--;
    out.line("}");
}

std::string writeStage(const Scene& scene, UsdIoResult& result) {
    ExportItem world;
    world.name = "World";
    world.path = "/World";
    world.node = scene.root();
    std::unordered_set<std::string> childNames;
    if (scene.root()) {
        for (const auto& child : scene.root()->children) {
            ExportItem item;
            buildItems(child, "/World", childNames, item);
            world.children.push_back(std::move(item));
        }
    }
    std::vector<JointOut> joints;
    Out body;
    if (scene.root()) {
        for (const ExportItem& child : world.children) {
            writeNode(body, child, scene, world, joints, result);
        }
    }
    for (const JointOut& joint : joints) {
        body.line("def " + joint.type + " \"" + joint.name + "\"");
        body.line("{");
        body.indent++;
        if (!joint.body0.empty()) {
            body.line("rel physics:body0 = <" + joint.body0 + ">");
        }
        body.line("rel physics:body1 = <" + joint.body1 + ">");
        body.line(std::string("uniform token physics:axis = \"") + axisToken(joint.axis) + "\"");
        body.text << std::string(static_cast<std::size_t>(body.indent) * 4, ' ') << "custom float3 sim:axis = ";
        writeVec3(body.text, joint.axis);
        body.text << "\n";
        if (joint.limits && joint.type != "PhysicsFixedJoint") {
            body.line("float physics:lowerLimit = " + num(joint.lower));
            body.line("float physics:upperLimit = " + num(joint.upper));
        }
        body.line("point3f physics:localPos0 = (0, 0, 0)");
        body.line("point3f physics:localPos1 = (0, 0, 0)");
        body.line("quatf physics:localRot0 = (1, 0, 0, 0)");
        body.line("quatf physics:localRot1 = (1, 0, 0, 0)");
        body.indent--;
        body.line("}");
    }
    double endCode = 0;
    writeAnimations(body, scene, result, endCode);
    body.line("def Material \"DefaultPhysics\" (");
    body.indent++;
    body.line("prepend apiSchemas = [\"PhysicsMaterialAPI\"]");
    body.indent--;
    body.line(")");
    body.line("{");
    body.indent++;
    body.line("float physics:dynamicFriction = " + num(scene.physicsDoc.defaultFriction));
    body.line("float physics:restitution = " + num(scene.physicsDoc.defaultRestitution));
    body.indent--;
    body.line("}");

    Out file;
    file.line("#usda 1.0");
    file.line("(");
    file.indent++;
    file.line("doc = \"Ondot Sim USDA. Coordinates are millimeters. metersPerUnit 0.001. OpenUSD is not linked.\"");
    file.line("defaultPrim = \"World\"");
    file.line("metersPerUnit = 0.001");
    file.line("upAxis = \"Z\"");
    file.line("kilogramsPerUnit = 1");
    if (result.sampleCount > 0) {
        file.line("startTimeCode = 0");
        file.line("endTimeCode = " + num(endCode));
        file.line("timeCodesPerSecond = 60");
    }
    file.indent--;
    file.line(")");
    file.line("def Xform \"World\" (");
    file.indent++;
    file.line("kind = \"assembly\"");
    file.indent--;
    file.line(")");
    file.line("{");
    file.text << body.text.str();
    file.line("}");
    return file.text.str();
}

Mat4 composedLocal(const Prim& prim) {
    auto opMatrix = [&](const std::string& op) {
        if (op == "xformOp:transform" && prim.hasMatrix) {
            return prim.matrix;
        }
        if (op == "xformOp:translate" && prim.hasTranslate) {
            Mat4 translation = Mat4::identity();
            translation(3, 0) = prim.translate.x;
            translation(3, 1) = prim.translate.y;
            translation(3, 2) = prim.translate.z;
            return translation;
        }
        return Mat4::identity();
    };
    if (prim.xformOrder.empty()) {
        if (prim.hasMatrix) {
            return prim.matrix;
        }
        if (prim.hasTranslate) {
            return opMatrix("xformOp:translate");
        }
        return Mat4::identity();
    }
    Mat4 acc = Mat4::identity();
    for (const std::string& op : prim.xformOrder) {
        acc = mul(opMatrix(op), acc);
    }
    return acc;
}

struct ImportCtx {
    Scene* scene = nullptr;
    double toMm = 1;
    double framesPerSecond = 60;
    std::unordered_map<std::string, std::shared_ptr<SceneNode>> byPath;
    std::vector<const Prim*> joints;
    std::vector<std::string> notes;
    int nodes = 0;
    int meshes = 0;
    int triangles = 0;
    int samples = 0;
    bool fanned = false;
};

void pullMaterial(const Prim& prim, SceneNode& node) {
    if (prim.hasColor) {
        node.color = prim.color;
    }
    if (prim.hasMetallic) {
        node.metallic = prim.metallic;
    }
    if (prim.hasRoughness) {
        node.roughness = prim.roughness;
    }
    for (const Prim& child : prim.children) {
        pullMaterial(child, node);
    }
}

void adoptMesh(ImportCtx& ctx, SceneNode& node, const Prim& mesh) {
    auto data = std::make_shared<MeshData>();
    data->positions.reserve(mesh.points.size() * 3);
    for (Vec3 point : mesh.points) {
        point = point * ctx.toMm;
        data->positions.push_back(static_cast<float>(point.x));
        data->positions.push_back(static_cast<float>(point.y));
        data->positions.push_back(static_cast<float>(point.z));
    }
    int cursor = 0;
    auto push = [&](int index) {
        if (index < 0 || index >= static_cast<int>(mesh.points.size())) {
            return false;
        }
        data->indices.push_back(static_cast<std::uint32_t>(index));
        return true;
    };
    for (int count : mesh.faceCounts) {
        if (cursor + count > static_cast<int>(mesh.indices.size())) {
            break;
        }
        if (count == 3) {
            const int a = mesh.indices[static_cast<std::size_t>(cursor)];
            const int b = mesh.indices[static_cast<std::size_t>(cursor + 1)];
            const int c = mesh.indices[static_cast<std::size_t>(cursor + 2)];
            const std::size_t before = data->indices.size();
            if (!(push(a) && push(b) && push(c))) {
                data->indices.resize(before);
            }
        } else if (count > 3) {
            ctx.fanned = true;
            const int anchor = mesh.indices[static_cast<std::size_t>(cursor)];
            for (int corner = 1; corner + 1 < count; ++corner) {
                const std::size_t before = data->indices.size();
                const int b = mesh.indices[static_cast<std::size_t>(cursor + corner)];
                const int c = mesh.indices[static_cast<std::size_t>(cursor + corner + 1)];
                if (!(push(anchor) && push(b) && push(c))) {
                    data->indices.resize(before);
                }
            }
        }
        cursor += count;
    }
    data->normals.assign(data->positions.size(), 0);
    if (mesh.normals.size() == mesh.points.size() && !mesh.normals.empty()) {
        for (std::size_t index = 0; index < mesh.normals.size(); ++index) {
            data->normals[index * 3] = static_cast<float>(mesh.normals[index].x);
            data->normals[index * 3 + 1] = static_cast<float>(mesh.normals[index].y);
            data->normals[index * 3 + 2] = static_cast<float>(mesh.normals[index].z);
        }
    } else {
        for (std::size_t index = 0; index + 2 < data->indices.size(); index += 3) {
            const std::uint32_t ia = data->indices[index];
            const std::uint32_t ib = data->indices[index + 1];
            const std::uint32_t ic = data->indices[index + 2];
            const Vec3 a{data->positions[ia * 3], data->positions[ia * 3 + 1], data->positions[ia * 3 + 2]};
            const Vec3 b{data->positions[ib * 3], data->positions[ib * 3 + 1], data->positions[ib * 3 + 2]};
            const Vec3 c{data->positions[ic * 3], data->positions[ic * 3 + 1], data->positions[ic * 3 + 2]};
            const Vec3 normal = cross(b - a, c - a);
            for (std::uint32_t vertex : {ia, ib, ic}) {
                data->normals[vertex * 3] += static_cast<float>(normal.x);
                data->normals[vertex * 3 + 1] += static_cast<float>(normal.y);
                data->normals[vertex * 3 + 2] += static_cast<float>(normal.z);
            }
        }
        for (std::size_t index = 0; index + 2 < data->normals.size(); index += 3) {
            Vec3 normal{data->normals[index], data->normals[index + 1], data->normals[index + 2]};
            if (length(normal) < 1e-8) {
                normal = {0, 0, 1};
            } else {
                normal = normalize(normal);
            }
            data->normals[index] = static_cast<float>(normal.x);
            data->normals[index + 1] = static_cast<float>(normal.y);
            data->normals[index + 2] = static_cast<float>(normal.z);
        }
    }
    data->recomputeBounds();
    pullMaterial(mesh, node);
    if (data->empty()) {
        return;
    }
    node.meshId = ctx.scene->addMesh(std::move(data));
    ++ctx.meshes;
    if (const auto stored = ctx.scene->mesh(node.meshId)) {
        ctx.triangles += static_cast<int>(stored->triangleCount());
    }
}

void applyPhysics(SceneNode& node, const Prim& prim, double toMm) {
    if (!prim.mech.empty()) {
        mechKindFromName(prim.mech, node.mech.kind);
    }
    if (!prim.motion.empty()) {
        bodyMotionFromName(prim.motion, node.mech.motion);
    }
    if (!prim.shape.empty()) {
        collisionShapeFromName(prim.shape, node.mech.shape);
    } else if (prim.approximation == "boundingSphere") {
        node.mech.shape = CollisionShape::Sphere;
    } else if (prim.approximation == "boundingCube") {
        node.mech.shape = CollisionShape::Box;
    } else if (prim.approximation == "mesh") {
        node.mech.shape = CollisionShape::Mesh;
    } else if (prim.approximation == "convexHull") {
        node.mech.shape = CollisionShape::ConvexHull;
    }
    if (prim.hasMass) {
        node.mech.mass = prim.mass;
        node.mech.autoMass = false;
    }
    if (prim.hasAutoMass) {
        node.mech.autoMass = prim.autoMass;
    }
    if (prim.hasDensity) {
        node.mech.density = prim.density;
    }
    if (prim.hasAxis) {
        node.mech.axis = prim.axis;
    }
    if (prim.hasLimits) {
        node.mech.limited = true;
        node.mech.lower = prim.lower;
        node.mech.upper = prim.upper;
        if (node.mech.kind == MechKind::Slider) {
            node.mech.lower *= toMm;
            node.mech.upper *= toMm;
        }
    }
    if (prim.hasHalf) {
        node.mech.halfExtents = prim.half * toMm;
    }
    if (prim.hasRadius) {
        node.mech.radius = prim.radius * toMm;
    }
    if (prim.hasHeight) {
        node.mech.height = prim.height * toMm;
    }
    if (prim.hasKinematic) {
        node.mech.kinematic = prim.kinematic;
    }
    if (!prim.other.empty()) {
        node.mech.other = prim.other;
    }
    if (prim.hasRigid && node.mech.kind == MechKind::None && prim.rigid) {
        node.mech.kind = MechKind::Rigid;
    }
}

void addAnimation(ImportCtx& ctx, const Prim& prim) {
    if (prim.samples.empty()) {
        return;
    }
    RobotPath path;
    path.name = prim.simName.empty() ? prim.name : prim.simName;
    path.resource = prim.resource.empty() ? path.name : prim.resource;
    for (std::size_t index = 0; index < prim.samples.size(); ++index) {
        const Sample& sample = prim.samples[index];
        PathPoint point;
        point.name = "s" + std::to_string(index);
        point.cartesian = sample.hasTranslate;
        point.position = sample.translate * ctx.toMm;
        point.joints = sample.joints;
        point.move = point.cartesian ? PathMove::Lin : PathMove::Ptp;
        if (index + 1 < prim.samples.size()) {
            const Sample& next = prim.samples[index + 1];
            const double dt = (next.code - sample.code) / std::max(ctx.framesPerSecond, 1e-6);
            if (dt > 1e-9 && sample.hasTranslate && next.hasTranslate) {
                point.speed = length((next.translate - sample.translate) * ctx.toMm) / dt;
            } else if (dt > 1e-9) {
                double delta = 0;
                const std::size_t count = std::max(sample.joints.size(), next.joints.size());
                for (std::size_t joint = 0; joint < count; ++joint) {
                    const double left = joint < sample.joints.size() ? sample.joints[joint] : 0;
                    const double right = joint < next.joints.size() ? next.joints[joint] : 0;
                    delta = std::max(delta, std::abs(right - left));
                }
                point.speed = delta / dt;
            }
        } else if (!path.points.empty()) {
            point.speed = path.points.back().speed;
        }
        if (point.speed <= 0) {
            point.speed = 100;
        }
        path.points.push_back(std::move(point));
    }
    ctx.samples += static_cast<int>(path.points.size());
    ctx.scene->robotPaths.push_back(std::move(path));
}

bool animationFolder(const Prim& prim) {
    if (prim.children.empty() || prim.role == "animation") {
        return false;
    }
    for (const Prim& child : prim.children) {
        if (child.role != "animation") {
            return false;
        }
    }
    return true;
}

void addMeshPrim(ImportCtx& ctx, const std::shared_ptr<SceneNode>& parent, const Prim& mesh) {
    if (mesh.name == "Geometry" && parent) {
        adoptMesh(ctx, *parent, mesh);
        pullMaterial(mesh, *parent);
        return;
    }
    const std::string name = mesh.simName.empty() ? mesh.name : mesh.simName;
    auto node = ctx.scene->makeNode(NodeKind::Part, name);
    ctx.byPath[mesh.path] = node;
    if (mesh.hasMatrix || mesh.hasTranslate || !mesh.xformOrder.empty()) {
        Mat4 matrix = composedLocal(mesh);
        matrix(3, 0) *= ctx.toMm;
        matrix(3, 1) *= ctx.toMm;
        matrix(3, 2) *= ctx.toMm;
        node->local = transformFromMatrix(matrix);
    }
    adoptMesh(ctx, *node, mesh);
    pullMaterial(mesh, *node);
    if (parent) {
        ctx.scene->addChild(parent, node);
    }
    ++ctx.nodes;
}

void addNodes(ImportCtx& ctx, const std::shared_ptr<SceneNode>& parent, const Prim& prim) {
    if (prim.type == "Material" || prim.type == "Shader") {
        if (prim.hasFriction) {
            ctx.scene->physicsDoc.defaultFriction = prim.friction;
        }
        if (prim.hasRestitution) {
            ctx.scene->physicsDoc.defaultRestitution = prim.restitution;
        }
        return;
    }
    if (prim.type.find("Joint") != std::string::npos) {
        ctx.joints.push_back(&prim);
        return;
    }
    if (prim.role == "animation") {
        addAnimation(ctx, prim);
        return;
    }
    if (animationFolder(prim)) {
        for (const Prim& child : prim.children) {
            addAnimation(ctx, child);
        }
        return;
    }
    if (prim.type == "Mesh") {
        return;
    }
    NodeKind kind = NodeKind::Part;
    if (!prim.simKind.empty()) {
        nodeKindFromName(prim.simKind, kind);
    }
    const std::string name = prim.simName.empty() ? prim.name : prim.simName;
    auto node = ctx.scene->makeNode(kind, name);
    ctx.byPath[prim.path] = node;
    if (prim.hasMatrix || prim.hasTranslate || !prim.xformOrder.empty()) {
        Mat4 matrix = composedLocal(prim);
        matrix(3, 0) *= ctx.toMm;
        matrix(3, 1) *= ctx.toMm;
        matrix(3, 2) *= ctx.toMm;
        node->local = transformFromMatrix(matrix);
    }
    applyPhysics(*node, prim, ctx.toMm);
    if (prim.hasColor) {
        node->color = prim.color;
    }
    if (prim.hasMetallic) {
        node->metallic = prim.metallic;
    }
    if (prim.hasRoughness) {
        node->roughness = prim.roughness;
    }
    if (prim.hasVisible) {
        node->visible = prim.visible;
    }
    if (parent) {
        ctx.scene->addChild(parent, node);
    }
    ++ctx.nodes;
    for (const Prim& child : prim.children) {
        if (child.type == "Mesh") {
            addMeshPrim(ctx, node, child);
        } else if (child.type == "Material" || child.type == "Shader") {
            pullMaterial(child, *node);
            if (child.hasFriction) {
                ctx.scene->physicsDoc.defaultFriction = child.friction;
            }
            if (child.hasRestitution) {
                ctx.scene->physicsDoc.defaultRestitution = child.restitution;
            }
        } else {
            addNodes(ctx, node, child);
        }
    }
}

void applyJoint(ImportCtx& ctx, const Prim& joint) {
    const auto found = ctx.byPath.find(joint.body1);
    if (found == ctx.byPath.end() || !found->second) {
        ctx.notes.push_back("joint " + joint.name + " has no body1 in the imported hierarchy");
        return;
    }
    SceneNode& node = *found->second;
    if (joint.type.find("Revolute") != std::string::npos) {
        node.mech.kind = MechKind::Hinge;
    } else if (joint.type.find("Prismatic") != std::string::npos) {
        node.mech.kind = MechKind::Slider;
    } else if (joint.type.find("Spherical") != std::string::npos) {
        node.mech.kind = MechKind::Ball;
    } else if (joint.type.find("Fixed") != std::string::npos) {
        node.mech.kind = MechKind::FixedJoint;
    }
    const auto other = ctx.byPath.find(joint.body0);
    if (other != ctx.byPath.end() && other->second) {
        node.mech.other = other->second->name;
    }
    if (joint.hasAxis) {
        node.mech.axis = joint.axis;
    } else if (joint.axisToken == "Y") {
        node.mech.axis = {0, 1, 0};
    } else if (joint.axisToken == "Z") {
        node.mech.axis = {0, 0, 1};
    } else if (joint.axisToken == "X") {
        node.mech.axis = {1, 0, 0};
    }
    if (joint.hasLimits) {
        node.mech.limited = true;
        node.mech.lower = joint.lower;
        node.mech.upper = joint.upper;
        if (node.mech.kind == MechKind::Slider) {
            node.mech.lower *= ctx.toMm;
            node.mech.upper *= ctx.toMm;
        }
    }
}

AssetImportResult importUsdaText(const std::string& text, Scene& scene) {
    AssetImportResult result;
    result.format = "usda";
    if (text.size() >= 8 && text.compare(0, 8, "PXR-USDC") == 0) {
        result.message = "binary USD crate is not supported; this writer emits USDA text";
        return result;
    }
    const auto newline = text.find('\n');
    const std::string first = text.substr(0, newline == std::string::npos ? text.size() : newline);
    if (first.find("#usda") == std::string::npos) {
        result.message = "not a USDA file";
        return result;
    }
    Stage stage;
    std::string error;
    if (!parseStage(text, stage, error)) {
        result.message = error;
        return result;
    }
    const Prim* rootPrim = nullptr;
    if (!stage.defaultPrim.empty()) {
        for (const Prim& prim : stage.roots) {
            if (prim.name == stage.defaultPrim) {
                rootPrim = &prim;
                break;
            }
        }
    }
    if (rootPrim == nullptr && stage.roots.size() == 1) {
        rootPrim = &stage.roots.front();
    }
    if (rootPrim == nullptr && stage.roots.empty()) {
        result.message = "USDA file has no prims";
        return result;
    }
    ImportCtx ctx;
    ctx.scene = &scene;
    const double unit = stage.metersPerUnit < 0 ? 0.01 : stage.metersPerUnit;
    ctx.toMm = unit * 1000.0;
    if (std::abs(ctx.toMm - 1.0) < 1e-9) {
        ctx.toMm = 1;
    }
    if (stage.metersPerUnit < 0) {
        ctx.notes.push_back("metersPerUnit was absent; used the USD default 0.01 and scaled into millimeters");
    }
    if (!stage.upAxis.empty() && stage.upAxis != "Z" && stage.upAxis != "z") {
        ctx.notes.push_back("upAxis is " + stage.upAxis + "; coordinates were not rotated into Z-up");
    }
    ctx.framesPerSecond = stage.framesPerSecond > 0 ? stage.framesPerSecond : 24.0;
    ctx.notes.insert(ctx.notes.end(), stage.notes.begin(), stage.notes.end());
    scene.beginEdit();
    const bool graft = scene.root() && scene.root()->children.empty() && scene.root()->meshId.empty();
    if (rootPrim != nullptr && graft) {
        ctx.byPath[rootPrim->path] = scene.root();
        ctx.byPath["/World"] = scene.root();
        if (rootPrim->hasMatrix || rootPrim->hasTranslate) {
            Mat4 matrix = composedLocal(*rootPrim);
            matrix(3, 0) *= ctx.toMm;
            matrix(3, 1) *= ctx.toMm;
            matrix(3, 2) *= ctx.toMm;
            scene.root()->local = transformFromMatrix(matrix);
        }
        for (const Prim& child : rootPrim->children) {
            if (child.type == "Mesh") {
                addMeshPrim(ctx, scene.root(), child);
            } else {
                addNodes(ctx, scene.root(), child);
            }
        }
    } else if (rootPrim != nullptr) {
        addNodes(ctx, scene.root(), *rootPrim);
    } else {
        for (const Prim& prim : stage.roots) {
            addNodes(ctx, scene.root(), prim);
        }
    }
    for (const Prim* joint : ctx.joints) {
        applyJoint(ctx, *joint);
    }
    scene.endEdit();
    if (ctx.samples > 0 && stage.framesPerSecond < 0) {
        ctx.notes.push_back("timeCodesPerSecond was absent; used 24");
    }
    if (ctx.fanned) {
        ctx.notes.push_back("n-gon faces were fanned into triangles");
    }
    result.ok = true;
    result.nodeCount = ctx.nodes;
    result.meshCount = ctx.meshes;
    result.triangleCount = ctx.triangles;
    result.message = "imported USDA";
    if (!ctx.notes.empty()) {
        result.message += ": ";
        for (std::size_t index = 0; index < ctx.notes.size(); ++index) {
            if (index) {
                result.message += "; ";
            }
            result.message += ctx.notes[index];
        }
    }
    return result;
}

class UsdaImporter : public IAssetImporter {
public:
    const char* format() const override { return "usda"; }
    std::vector<std::string> extensions() const override { return {"usda", "usd"}; }
    bool probeContent(const std::string& head) const override {
        return head.find("#usda") != std::string::npos || head.find("PXR-USDC") != std::string::npos;
    }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            AssetImportResult result;
            result.format = "usda";
            result.message = "cannot open " + path;
            return result;
        }
        std::ostringstream buffer;
        buffer << in.rdbuf();
        const std::string text = buffer.str();
        if (text.size() > 64u * 1024u * 1024u) {
            AssetImportResult result;
            result.format = "usda";
            result.message = "USDA file is larger than 64 MB";
            return result;
        }
        return importUsdaText(text, scene);
    }
};

}  // namespace

UsdIoResult exportSceneUsda(const Scene& scene, const std::string& path) {
    UsdIoResult result;
    const std::string text = writeStage(scene, result);
    std::string error;
    if (!validateUsdaText(text, error)) {
        result.ok = false;
        result.message = error.empty() ? "USDA validation failed" : error;
        return result;
    }
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        result.ok = false;
        result.message = "cannot write " + path;
        return result;
    }
    out << text;
    if (!out) {
        result.ok = false;
        result.message = "cannot write " + path;
        return result;
    }
    result.ok = true;
    result.message = "wrote USDA";
    return result;
}

bool validateUsdaText(const std::string& text, std::string& error) {
    Stage stage;
    if (!parseStage(text, stage, error)) {
        return false;
    }
    if (stage.defaultPrim != "World") {
        error = "defaultPrim is not World";
        return false;
    }
    if (std::abs(stage.metersPerUnit - 0.001) > 1e-9) {
        error = "metersPerUnit is not 0.001";
        return false;
    }
    if (stage.upAxis != "Z") {
        error = "upAxis is not Z";
        return false;
    }
    int faces = 0;
    for (const Prim& prim : stage.roots) {
        const int count = validatePrim(prim, error);
        if (count < 0) {
            return false;
        }
        faces += count;
    }
    if (faces > 0 && !stage.hasPreview) {
        error = "a mesh is missing UsdPreviewSurface";
        return false;
    }
    error.clear();
    return true;
}

std::unique_ptr<IAssetImporter> createUsdaImporter() { return std::make_unique<UsdaImporter>(); }

}  // namespace sim
