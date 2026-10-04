#include "sim/core/LineModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <vector>

namespace sim {
namespace {

std::string trimCopy(const std::string& text) {
    std::size_t begin = 0;
    while (begin < text.size() && (text[begin] == ' ' || text[begin] == '\t' || text[begin] == '\r')) {
        ++begin;
    }
    std::size_t end = text.size();
    while (end > begin && (text[end - 1] == ' ' || text[end - 1] == '\t' || text[end - 1] == '\r')) {
        --end;
    }
    return text.substr(begin, end - begin);
}

struct Group {
    int code = 0;
    std::string value;
};

void pushSegment(DxfResult& result, Vec3 a, Vec3 b, const std::string& layer) {
    result.lines.push_back({a, b});
    result.layers.push_back(layer);
}

void addArc(DxfResult& result, double cx, double cy, double radius, double startDeg, double endDeg, double scale,
            const std::string& layer) {
    if (radius <= 0) {
        return;
    }
    double sweep = endDeg - startDeg;
    while (sweep <= 0) {
        sweep += 360.0;
    }
    const int steps = std::max(8, static_cast<int>(std::ceil(sweep / 15.0)));
    Vec3 previous{(cx + radius * std::cos(startDeg * kPi / 180.0)) * scale, (cy + radius * std::sin(startDeg * kPi / 180.0)) * scale, 0};
    for (int step = 1; step <= steps; ++step) {
        const double angle = startDeg + sweep * (static_cast<double>(step) / steps);
        const double rad = angle * kPi / 180.0;
        Vec3 next{(cx + radius * std::cos(rad)) * scale, (cy + radius * std::sin(rad)) * scale, 0};
        pushSegment(result, previous, next, layer);
        previous = next;
    }
}

void addCircle(DxfResult& result, double cx, double cy, double radius, double scale, const std::string& layer) {
    if (radius <= 0) {
        return;
    }
    constexpr int kSegments = 16;
    Vec3 previous{(cx + radius) * scale, cy * scale, 0};
    for (int step = 1; step <= kSegments; ++step) {
        const double angle = (2.0 * kPi * step) / kSegments;
        Vec3 next{(cx + radius * std::cos(angle)) * scale, (cy + radius * std::sin(angle)) * scale, 0};
        pushSegment(result, previous, next, layer);
        previous = next;
    }
}

void addSpline(DxfResult& result, const std::vector<double>& xs, const std::vector<double>& ys, int degree, double scale,
               const std::string& layer) {
    const std::size_t count = std::min(xs.size(), ys.size());
    if (count < 2) {
        return;
    }
    const int p = std::clamp(degree <= 0 ? 3 : degree, 1, 3);
    if (static_cast<int>(count) <= p) {
        for (std::size_t i = 1; i < count; ++i) {
            pushSegment(result, {xs[i - 1] * scale, ys[i - 1] * scale, 0}, {xs[i] * scale, ys[i] * scale, 0}, layer);
        }
        return;
    }
    std::vector<double> knots(count + static_cast<std::size_t>(p) + 1, 0);
    const int span = static_cast<int>(count) - p;
    for (int i = 0; i <= p; ++i) {
        knots[static_cast<std::size_t>(i)] = 0;
        knots[knots.size() - 1 - static_cast<std::size_t>(i)] = 1;
    }
    for (int i = 1; i < span; ++i) {
        knots[static_cast<std::size_t>(p + i)] = static_cast<double>(i) / span;
    }
    constexpr int kSamples = 16;
    Vec3 previous{xs.front() * scale, ys.front() * scale, 0};
    for (int sample = 1; sample <= kSamples; ++sample) {
        const double t = sample == kSamples ? 1.0 - 1e-9 : static_cast<double>(sample) / kSamples;
        int k = p;
        while (k < static_cast<int>(count) && !(t >= knots[static_cast<std::size_t>(k)] && t < knots[static_cast<std::size_t>(k + 1)])) {
            ++k;
        }
        if (k >= static_cast<int>(count)) {
            k = static_cast<int>(count) - 1;
        }
        std::vector<double> dx(static_cast<std::size_t>(p) + 1);
        std::vector<double> dy(static_cast<std::size_t>(p) + 1);
        for (int j = 0; j <= p; ++j) {
            const int index = k - p + j;
            dx[static_cast<std::size_t>(j)] = xs[static_cast<std::size_t>(std::clamp(index, 0, static_cast<int>(count) - 1))];
            dy[static_cast<std::size_t>(j)] = ys[static_cast<std::size_t>(std::clamp(index, 0, static_cast<int>(count) - 1))];
        }
        for (int r = 1; r <= p; ++r) {
            for (int j = p; j >= r; --j) {
                const int ii = k - p + j;
                const double left = knots[static_cast<std::size_t>(ii)];
                const double right = knots[static_cast<std::size_t>(ii + p - r + 1)];
                const double denom = right - left;
                const double alpha = denom == 0 ? 0 : (t - left) / denom;
                dx[static_cast<std::size_t>(j)] = (1 - alpha) * dx[static_cast<std::size_t>(j - 1)] + alpha * dx[static_cast<std::size_t>(j)];
                dy[static_cast<std::size_t>(j)] = (1 - alpha) * dy[static_cast<std::size_t>(j - 1)] + alpha * dy[static_cast<std::size_t>(j)];
            }
        }
        Vec3 next{dx.back() * scale, dy.back() * scale, 0};
        pushSegment(result, previous, next, layer);
        previous = next;
    }
}

Vec3 transformPlan(Vec3 point, double x, double y, double rotationDeg, double sx, double sy) {
    const double rad = rotationDeg * kPi / 180.0;
    const double px = point.x * sx;
    const double py = point.y * sy;
    return {x + px * std::cos(rad) - py * std::sin(rad), y + px * std::sin(rad) + py * std::cos(rad), 0};
}

enum class ValueKind { String, Double, Int16, Int32 };

ValueKind valueKind(int code) {
    if ((code >= 10 && code <= 59) || (code >= 110 && code <= 149) || (code >= 210 && code <= 239) ||
        (code >= 460 && code <= 469) || (code >= 1010 && code <= 1059)) {
        return ValueKind::Double;
    }
    if ((code >= 60 && code <= 79) || (code >= 170 && code <= 179) || (code >= 270 && code <= 289) ||
        (code >= 370 && code <= 389) || (code >= 400 && code <= 409) || (code >= 1060 && code <= 1070)) {
        return ValueKind::Int16;
    }
    if ((code >= 90 && code <= 99) || (code >= 420 && code <= 429) || (code >= 440 && code <= 459) || code == 1071) {
        return ValueKind::Int32;
    }
    return ValueKind::String;
}

bool decodeBinary(const std::string& text, std::vector<Group>& groups, std::string& error) {
    if (text.size() < 22) {
        error = "binary DXF header is truncated";
        return false;
    }
    std::size_t cursor = 22;
    while (cursor + 2 <= text.size()) {
        const int code = static_cast<unsigned char>(text[cursor]) | (static_cast<unsigned char>(text[cursor + 1]) << 8);
        cursor += 2;
        Group group;
        group.code = code;
        const ValueKind kind = valueKind(code);
        if (kind == ValueKind::Double) {
            if (cursor + 8 > text.size()) {
                error = "binary DXF double is truncated";
                return false;
            }
            double value = 0;
            std::memcpy(&value, text.data() + cursor, sizeof(value));
            cursor += 8;
            group.value = std::to_string(value);
        } else if (kind == ValueKind::Int16) {
            if (cursor + 2 > text.size()) {
                error = "binary DXF integer is truncated";
                return false;
            }
            const int value = static_cast<unsigned char>(text[cursor]) | (static_cast<unsigned char>(text[cursor + 1]) << 8);
            cursor += 2;
            group.value = std::to_string(value);
        } else if (kind == ValueKind::Int32) {
            if (cursor + 4 > text.size()) {
                error = "binary DXF integer is truncated";
                return false;
            }
            const std::uint32_t value = static_cast<unsigned char>(text[cursor]) |
                                        (static_cast<unsigned char>(text[cursor + 1]) << 8) |
                                        (static_cast<unsigned char>(text[cursor + 2]) << 16) |
                                        (static_cast<unsigned char>(text[cursor + 3]) << 24);
            cursor += 4;
            group.value = std::to_string(value);
        } else {
            const auto end = text.find('\0', cursor);
            if (end == std::string::npos) {
                error = "binary DXF string is truncated";
                return false;
            }
            group.value = text.substr(cursor, end - cursor);
            cursor = end + 1;
        }
        groups.push_back(std::move(group));
        if (!groups.empty() && groups.back().code == 0 && groups.back().value == "EOF") {
            break;
        }
    }
    if (groups.size() < 2) {
        error = "binary DXF has no entities";
        return false;
    }
    error.clear();
    return true;
}

bool decodeAscii(const std::string& text, std::vector<Group>& groups, std::string& error) {
    std::string line;
    std::istringstream in(text);
    std::vector<std::string> lines;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
    }
    if (lines.size() < 2 || lines.size() % 2 != 0) {
        error = "dxf group codes are incomplete";
        return false;
    }
    try {
        for (std::size_t i = 0; i + 1 < lines.size(); i += 2) {
            Group group;
            group.code = std::stoi(trimCopy(lines[i]));
            group.value = trimCopy(lines[i + 1]);
            groups.push_back(std::move(group));
        }
    } catch (const std::exception&) {
        error = "dxf group codes are incomplete";
        return false;
    }
    error.clear();
    return true;
}

struct Block {
    std::string name;
    std::vector<std::pair<Vec3, Vec3>> lines;
    std::vector<std::string> layers;
};

}  // namespace

DxfResult parseDxf(const std::string& text, double mmPerUnit) {
    DxfResult result;
    const double scale = mmPerUnit > 1e-12 ? mmPerUnit : 1.0;
    std::vector<Group> groups;
    const bool binary = text.size() >= 18 && text.compare(0, 18, "AutoCAD Binary DXF") == 0;
    if (binary) {
        if (!decodeBinary(text, groups, result.message)) {
            return result;
        }
    } else {
        if (text.find('\0') != std::string::npos) {
            result.message = "binary DXF is not supported";
            return result;
        }
        if (!decodeAscii(text, groups, result.message)) {
            return result;
        }
    }

    struct Entity {
        std::string type;
        std::string layer;
        std::string name;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
        std::vector<double> x2;
        std::vector<double> y2;
        std::vector<double> z2;
        std::vector<double> x3;
        std::vector<double> y3;
        std::vector<double> z3;
        std::vector<double> x4;
        std::vector<double> y4;
        std::vector<double> z4;
        std::vector<double> x5;
        std::vector<double> y5;
        std::vector<double> knots;
        double radius = 0;
        double startAngle = 0;
        double endAngle = 0;
        double rotation = 0;
        double sx = 1;
        double sy = 1;
        int flags = 0;
        int degree = 3;
        bool alive = false;
    } current;
    enum class Section { None, Entities, Blocks };
    Section section = Section::None;
    bool pending = false;
    bool sawEntities = false;
    std::vector<Block> blocks;
    Block building;
    bool inBlock = false;

    auto emitInto = [&](std::vector<std::pair<Vec3, Vec3>>& lines, std::vector<std::string>& layers) {
        DxfResult local;
        if (current.type == "LINE" && !current.x.empty() && !current.y.empty() && !current.x2.empty() && !current.y2.empty()) {
            pushSegment(local, {current.x.front() * scale, current.y.front() * scale, 0},
                        {current.x2.front() * scale, current.y2.front() * scale, 0}, current.layer);
        } else if (current.type == "LWPOLYLINE") {
            const std::size_t n = std::min(current.x.size(), current.y.size());
            for (std::size_t i = 1; i < n; ++i) {
                pushSegment(local, {current.x[i - 1] * scale, current.y[i - 1] * scale, 0},
                            {current.x[i] * scale, current.y[i] * scale, 0}, current.layer);
            }
            if ((current.flags & 1) != 0 && n >= 2) {
                pushSegment(local, {current.x[n - 1] * scale, current.y[n - 1] * scale, 0},
                            {current.x.front() * scale, current.y.front() * scale, 0}, current.layer);
            }
        } else if (current.type == "CIRCLE" && !current.x.empty() && !current.y.empty()) {
            addCircle(local, current.x.front(), current.y.front(), current.radius, scale, current.layer);
        } else if (current.type == "ARC" && !current.x.empty() && !current.y.empty()) {
            addArc(local, current.x.front(), current.y.front(), current.radius, current.startAngle, current.endAngle, scale,
                   current.layer);
        } else if (current.type == "SPLINE") {
            addSpline(local, current.x, current.y, current.degree, scale, current.layer);
        } else if (current.type == "ELLIPSE" && !current.x.empty() && !current.y.empty() && !current.x2.empty() && !current.y2.empty()) {
            const Vec3 center{current.x.front(), current.y.front(), current.z.empty() ? 0 : current.z.front()};
            const Vec3 major{current.x2.front(), current.y2.front(), 0};
            const double majorLen = length(major);
            const double ratio = current.radius > 1e-9 ? current.radius : 1;
            if (majorLen > 1e-9 && ratio > 0) {
                const Vec3 majorDir = major * (1.0 / majorLen);
                const Vec3 minorDir{-majorDir.y, majorDir.x, 0};
                double a0 = current.startAngle;
                double a1 = current.endAngle == 0 && current.startAngle == 0 ? 2.0 * kPi : current.endAngle;
                if (a1 <= a0) {
                    a1 += 2.0 * kPi;
                }
                const int steps = std::max(12, static_cast<int>(std::ceil((a1 - a0) / (kPi / 12.0))));
                auto at = [&](double angle) {
                    return Vec3{(center.x + majorDir.x * majorLen * std::cos(angle) + minorDir.x * majorLen * ratio * std::sin(angle)) * scale,
                                (center.y + majorDir.y * majorLen * std::cos(angle) + minorDir.y * majorLen * ratio * std::sin(angle)) * scale,
                                center.z * scale};
                };
                Vec3 previous = at(a0);
                for (int step = 1; step <= steps; ++step) {
                    const Vec3 next = at(a0 + (a1 - a0) * step / steps);
                    pushSegment(local, previous, next, current.layer);
                    previous = next;
                }
            }
        } else if (current.type == "3DFACE") {
            auto coord = [](const std::vector<double>& values, std::size_t index) {
                return index < values.size() ? values[index] : 0.0;
            };
            const Vec3 points[4] = {
                {coord(current.x, 0) * scale, coord(current.y, 0) * scale, coord(current.z, 0) * scale},
                {coord(current.x2, 0) * scale, coord(current.y2, 0) * scale, coord(current.z2, 0) * scale},
                {coord(current.x3, 0) * scale, coord(current.y3, 0) * scale, coord(current.z3, 0) * scale},
                {coord(current.x4, 0) * scale, coord(current.y4, 0) * scale, coord(current.z4, 0) * scale},
            };
            const bool triangle = length(points[3] - points[2]) < 1e-6;
            const int edges = triangle ? 3 : 4;
            for (int edge = 0; edge < edges; ++edge) {
                pushSegment(local, points[edge], points[(edge + 1) % edges], current.layer);
            }
        } else if (current.type == "HATCH") {
            const std::size_t n = std::min(current.x.size(), current.y.size());
            for (std::size_t i = 1; i < n; ++i) {
                pushSegment(local, {current.x[i - 1] * scale, current.y[i - 1] * scale, 0},
                            {current.x[i] * scale, current.y[i] * scale, 0}, current.layer);
            }
            if (n >= 3) {
                pushSegment(local, {current.x[n - 1] * scale, current.y[n - 1] * scale, 0},
                            {current.x.front() * scale, current.y.front() * scale, 0}, current.layer);
            }
        } else if (current.type == "DIMENSION") {
            auto coord = [](const std::vector<double>& values) { return values.empty() ? 0.0 : values.front(); };
            if (!current.x4.empty() && !current.x5.empty()) {
                pushSegment(local, {coord(current.x4) * scale, coord(current.y4) * scale, coord(current.z4) * scale},
                            {coord(current.x5) * scale, coord(current.y5) * scale, 0}, current.layer);
            }
            if (!current.x.empty() && !current.x2.empty()) {
                pushSegment(local, {coord(current.x) * scale, coord(current.y) * scale, coord(current.z) * scale},
                            {coord(current.x2) * scale, coord(current.y2) * scale, coord(current.z2) * scale}, current.layer);
            }
        } else if (current.type == "INSERT" && !current.name.empty()) {
            const Block* block = nullptr;
            for (const Block& candidate : blocks) {
                if (candidate.name == current.name) {
                    block = &candidate;
                }
            }
            if (block != nullptr) {
                const double originX = (current.x.empty() ? 0 : current.x.front()) * scale;
                const double originY = (current.y.empty() ? 0 : current.y.front()) * scale;
                for (std::size_t i = 0; i < block->lines.size(); ++i) {
                    const Vec3 a = transformPlan(block->lines[i].first, originX, originY, current.rotation, current.sx, current.sy);
                    const Vec3 b = transformPlan(block->lines[i].second, originX, originY, current.rotation, current.sx, current.sy);
                    const std::string layer = current.layer.empty() && i < block->layers.size() ? block->layers[i] : current.layer;
                    pushSegment(local, a, b, layer);
                }
            }
        }
        if (section == Section::Blocks && inBlock) {
            building.lines.insert(building.lines.end(), local.lines.begin(), local.lines.end());
            building.layers.insert(building.layers.end(), local.layers.begin(), local.layers.end());
        } else if (section == Section::Entities) {
            lines.insert(lines.end(), local.lines.begin(), local.lines.end());
            layers.insert(layers.end(), local.layers.begin(), local.layers.end());
        }
    };

    auto flush = [&] {
        if (!current.alive) {
            return;
        }
        emitInto(result.lines, result.layers);
        current = {};
    };

    try {
        for (const Group& group : groups) {
            if (group.code == 0) {
                if (group.value == "ENDBLK") {
                    flush();
                    if (inBlock) {
                        blocks.push_back(std::move(building));
                        building = {};
                        inBlock = false;
                    }
                    continue;
                }
                flush();
                if (group.value == "ENDSEC") {
                    section = Section::None;
                    pending = false;
                    continue;
                }
                if (group.value == "SECTION") {
                    pending = true;
                    continue;
                }
                if (group.value == "EOF") {
                    break;
                }
                if (group.value == "BLOCK" && section == Section::Blocks) {
                    inBlock = true;
                    building = {};
                    current.type = "BLOCK";
                    current.alive = true;
                    continue;
                }
                if (section == Section::Entities || (section == Section::Blocks && inBlock)) {
                    current.type = group.value;
                    current.alive = true;
                }
                continue;
            }
            if (group.code == 2 && pending) {
                if (group.value == "ENTITIES") {
                    section = Section::Entities;
                    sawEntities = true;
                } else if (group.value == "BLOCKS") {
                    section = Section::Blocks;
                } else {
                    section = Section::None;
                }
                pending = false;
                continue;
            }
            if (group.code == 2 && current.type == "BLOCK" && inBlock && building.name.empty()) {
                building.name = group.value;
                continue;
            }
            if (!current.alive) {
                continue;
            }
            if (group.code == 2 && current.type == "INSERT") {
                current.name = group.value;
            } else if (group.code == 8) {
                current.layer = group.value;
            } else if (group.code == 10) {
                current.x.push_back(std::stod(group.value));
            } else if (group.code == 20) {
                current.y.push_back(std::stod(group.value));
            } else if (group.code == 30) {
                current.z.push_back(std::stod(group.value));
            } else if (group.code == 11) {
                current.x2.push_back(std::stod(group.value));
            } else if (group.code == 21) {
                current.y2.push_back(std::stod(group.value));
            } else if (group.code == 31) {
                current.z2.push_back(std::stod(group.value));
            } else if (group.code == 12) {
                current.x3.push_back(std::stod(group.value));
            } else if (group.code == 22) {
                current.y3.push_back(std::stod(group.value));
            } else if (group.code == 32) {
                current.z3.push_back(std::stod(group.value));
            } else if (group.code == 13) {
                current.x4.push_back(std::stod(group.value));
            } else if (group.code == 23) {
                current.y4.push_back(std::stod(group.value));
            } else if (group.code == 33) {
                current.z4.push_back(std::stod(group.value));
            } else if (group.code == 14) {
                current.x5.push_back(std::stod(group.value));
            } else if (group.code == 24) {
                current.y5.push_back(std::stod(group.value));
            } else if (group.code == 40) {
                current.radius = std::stod(group.value);
                current.knots.push_back(current.radius);
            } else if (group.code == 41) {
                if (current.type == "ELLIPSE") {
                    current.startAngle = std::stod(group.value);
                } else {
                    current.sx = std::stod(group.value);
                }
            } else if (group.code == 42) {
                if (current.type == "ELLIPSE") {
                    current.endAngle = std::stod(group.value);
                } else {
                    current.sy = std::stod(group.value);
                }
            } else if (group.code == 50) {
                current.startAngle = std::stod(group.value);
                current.rotation = current.startAngle;
            } else if (group.code == 51) {
                current.endAngle = std::stod(group.value);
            } else if (group.code == 70) {
                current.flags = std::stoi(group.value);
            } else if (group.code == 71) {
                current.degree = std::stoi(group.value);
            }
        }
        flush();
    } catch (const std::exception&) {
        result.lines.clear();
        result.layers.clear();
        result.message = "dxf group codes are incomplete";
        return result;
    }
    if (!sawEntities) {
        result.message = binary ? "binary DXF has no entities" : "dxf has no entities section";
        result.lines.clear();
        result.layers.clear();
        return result;
    }
    result.ok = true;
    result.message.clear();
    return result;
}

}  // namespace sim
