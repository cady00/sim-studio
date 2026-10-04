#include "sim/core/LineModel.hpp"

#include "sim/core/Scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <sstream>
#include <unordered_map>

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

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> lines;
    std::string line;
    std::istringstream in(text);
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
    }
    return lines;
}

std::string xmlEscape(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (unsigned char c : text) {
        if (c == '&') {
            out += "&amp;";
        } else if (c == '<') {
            out += "&lt;";
        } else if (c == '>') {
            out += "&gt;";
        } else if (c == '"') {
            out += "&quot;";
        } else {
            out.push_back(static_cast<char>(c));
        }
    }
    return out;
}

std::string csvField(const std::string& text) {
    if (text.find_first_of(",\"\n") == std::string::npos) {
        return text;
    }
    std::string out = "\"";
    for (char c : text) {
        if (c == '"') {
            out += "\"\"";
        } else {
            out.push_back(c);
        }
    }
    out.push_back('"');
    return out;
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

void appendU16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

bool writeStoreZip(const std::string& path, const std::vector<std::pair<std::string, std::string>>& files, std::string& error) {
    std::vector<std::uint8_t> blob;
    struct Central {
        std::string name;
        std::uint32_t crc = 0;
        std::uint32_t size = 0;
        std::uint32_t offset = 0;
    };
    std::vector<Central> centrals;
    for (const auto& file : files) {
        Central central;
        central.name = file.first;
        central.crc = crc32(reinterpret_cast<const std::uint8_t*>(file.second.data()), file.second.size());
        central.size = static_cast<std::uint32_t>(file.second.size());
        central.offset = static_cast<std::uint32_t>(blob.size());
        constexpr std::uint16_t kUtf8 = 1u << 11;
        appendU32(blob, 0x04034b50u);
        appendU16(blob, 20);
        appendU16(blob, kUtf8);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, central.crc);
        appendU32(blob, central.size);
        appendU32(blob, central.size);
        appendU16(blob, static_cast<std::uint16_t>(file.first.size()));
        appendU16(blob, 0);
        blob.insert(blob.end(), file.first.begin(), file.first.end());
        blob.insert(blob.end(), file.second.begin(), file.second.end());
        centrals.push_back(std::move(central));
    }
    const auto centralOffset = static_cast<std::uint32_t>(blob.size());
    for (const Central& central : centrals) {
        constexpr std::uint16_t kUtf8 = 1u << 11;
        appendU32(blob, 0x02014b50u);
        appendU16(blob, 20);
        appendU16(blob, 20);
        appendU16(blob, kUtf8);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, central.crc);
        appendU32(blob, central.size);
        appendU32(blob, central.size);
        appendU16(blob, static_cast<std::uint16_t>(central.name.size()));
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU16(blob, 0);
        appendU32(blob, 0);
        appendU32(blob, central.offset);
        blob.insert(blob.end(), central.name.begin(), central.name.end());
    }
    const auto centralSize = static_cast<std::uint32_t>(blob.size() - centralOffset);
    appendU32(blob, 0x06054b50u);
    appendU16(blob, 0);
    appendU16(blob, 0);
    appendU16(blob, static_cast<std::uint16_t>(centrals.size()));
    appendU16(blob, static_cast<std::uint16_t>(centrals.size()));
    appendU32(blob, centralSize);
    appendU32(blob, centralOffset);
    appendU16(blob, 0);
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        error = "cannot write " + path;
        return false;
    }
    out.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
    if (!out) {
        error = "failed while writing " + path;
        return false;
    }
    error.clear();
    return true;
}

std::string cellRef(int column, int row) {
    std::string name;
    int index = column;
    do {
        name.insert(name.begin(), static_cast<char>('A' + (index % 26)));
        index = index / 26 - 1;
    } while (index >= 0);
    return name + std::to_string(row);
}

std::string inlineCell(int column, int row, const std::string& text) {
    return "<c r=\"" + cellRef(column, row) + "\" t=\"inlineStr\"><is><t>" + xmlEscape(text) + "</t></is></c>";
}

std::string numberCell(int column, int row, double value) {
    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(3);
    text << value;
    return "<c r=\"" + cellRef(column, row) + "\"><v>" + text.str() + "</v></c>";
}

void addCircle(std::vector<std::pair<Vec3, Vec3>>& lines, double cx, double cy, double radius, double scale) {
    if (radius <= 0) {
        return;
    }
    constexpr int kSegments = 16;
    Vec3 previous{(cx + radius) * scale, cy * scale, 0};
    for (int step = 1; step <= kSegments; ++step) {
        const double angle = (2.0 * 3.14159265358979323846 * step) / kSegments;
        Vec3 next{(cx + radius * std::cos(angle)) * scale, (cy + radius * std::sin(angle)) * scale, 0};
        lines.emplace_back(previous, next);
        previous = next;
    }
}

}  // namespace

const char* stationKindName(StationKind kind) {
    switch (kind) {
        case StationKind::Robot: return "robot";
        case StationKind::Fixture: return "fixture";
        case StationKind::Conveyor: return "conveyor";
        case StationKind::Manual: return "manual";
        case StationKind::Buffer: return "buffer";
    }
    return "manual";
}

bool stationKindFromName(const std::string& name, StationKind& out) {
    if (name == "robot") {
        out = StationKind::Robot;
    } else if (name == "fixture") {
        out = StationKind::Fixture;
    } else if (name == "conveyor") {
        out = StationKind::Conveyor;
    } else if (name == "manual") {
        out = StationKind::Manual;
    } else if (name == "buffer") {
        out = StationKind::Buffer;
    } else {
        return false;
    }
    return true;
}

const char* opKindName(OpKind kind) {
    switch (kind) {
        case OpKind::Task: return "task";
        case OpKind::Sequence: return "sequence";
        case OpKind::Parallel: return "parallel";
    }
    return "task";
}

bool opKindFromName(const std::string& name, OpKind& out) {
    if (name == "task") {
        out = OpKind::Task;
    } else if (name == "sequence") {
        out = OpKind::Sequence;
    } else if (name == "parallel") {
        out = OpKind::Parallel;
    } else {
        return false;
    }
    return true;
}

double snapToGrid(double value, double gridMm) {
    if (!(gridMm > 1e-9)) {
        return value;
    }
    return std::round(value / gridMm) * gridMm;
}

void snapStations(LineLayout& layout) {
    for (Station& station : layout.stations) {
        station.x = snapToGrid(station.x, layout.gridMm);
        station.y = snapToGrid(station.y, layout.gridMm);
    }
}

void alignStationsMinX(LineLayout& layout) {
    if (layout.stations.empty()) {
        return;
    }
    double left = layout.stations.front().x - layout.stations.front().width * 0.5;
    for (const Station& station : layout.stations) {
        left = std::min(left, station.x - station.width * 0.5);
    }
    for (Station& station : layout.stations) {
        station.x = left + station.width * 0.5;
    }
    snapStations(layout);
}

void distributeStationsX(LineLayout& layout) {
    if (layout.stations.size() < 3) {
        snapStations(layout);
        return;
    }
    std::vector<Station*> ordered;
    ordered.reserve(layout.stations.size());
    for (Station& station : layout.stations) {
        ordered.push_back(&station);
    }
    std::sort(ordered.begin(), ordered.end(), [](const Station* a, const Station* b) {
        if (a->x != b->x) {
            return a->x < b->x;
        }
        return a->id < b->id;
    });
    const double first = ordered.front()->x;
    const double last = ordered.back()->x;
    const double step = (last - first) / static_cast<double>(ordered.size() - 1);
    for (std::size_t i = 0; i < ordered.size(); ++i) {
        ordered[i]->x = first + step * static_cast<double>(i);
    }
    snapStations(layout);
}

Station* findStation(LineLayout& layout, const std::string& id) {
    for (Station& station : layout.stations) {
        if (station.id == id) {
            return &station;
        }
    }
    return nullptr;
}

const Station* findStation(const LineLayout& layout, const std::string& id) {
    for (const Station& station : layout.stations) {
        if (station.id == id) {
            return &station;
        }
    }
    return nullptr;
}

ScheduleReport scheduleOperations(const std::vector<Operation>& operations, const LineLayout& layout) {
    ScheduleReport report;
    const int count = static_cast<int>(operations.size());
    if (count == 0) {
        return report;
    }
    std::unordered_map<std::string, int> index;
    index.reserve(operations.size());
    for (int i = 0; i < count; ++i) {
        if (operations[static_cast<std::size_t>(i)].id.empty() ||
            !index.emplace(operations[static_cast<std::size_t>(i)].id, i).second) {
            report.error = "duplicate or empty operation id";
            return report;
        }
    }
    std::vector<std::vector<int>> children(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const std::string& parent = operations[static_cast<std::size_t>(i)].parent;
        if (parent.empty()) {
            continue;
        }
        const auto it = index.find(parent);
        if (it == index.end()) {
            report.error = "missing parent " + parent;
            return report;
        }
        children[static_cast<std::size_t>(it->second)].push_back(i);
    }
    for (std::vector<int>& list : children) {
        std::sort(list.begin(), list.end(), [&](int a, int b) {
            const Operation& left = operations[static_cast<std::size_t>(a)];
            const Operation& right = operations[static_cast<std::size_t>(b)];
            if (left.index != right.index) {
                return left.index < right.index;
            }
            return left.id < right.id;
        });
    }

    std::vector<std::vector<int>> preds(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        for (const std::string& after : operations[static_cast<std::size_t>(i)].after) {
            const auto it = index.find(after);
            if (it == index.end()) {
                report.error = "missing predecessor " + after;
                return report;
            }
            preds[static_cast<std::size_t>(i)].push_back(it->second);
        }
        const std::string& parent = operations[static_cast<std::size_t>(i)].parent;
        if (parent.empty()) {
            continue;
        }
        const int parentIndex = index[parent];
        if (operations[static_cast<std::size_t>(parentIndex)].kind != OpKind::Sequence) {
            continue;
        }
        const std::vector<int>& siblings = children[static_cast<std::size_t>(parentIndex)];
        const auto pos = std::find(siblings.begin(), siblings.end(), i);
        if (pos != siblings.end() && pos != siblings.begin()) {
            preds[static_cast<std::size_t>(i)].push_back(*(pos - 1));
        }
    }

    std::vector<std::vector<int>> edges(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        for (int pred : preds[static_cast<std::size_t>(i)]) {
            edges[static_cast<std::size_t>(pred)].push_back(i);
        }
        const std::string& parent = operations[static_cast<std::size_t>(i)].parent;
        if (!parent.empty()) {
            edges[static_cast<std::size_t>(i)].push_back(index[parent]);
        }
    }
    std::vector<int> color(static_cast<std::size_t>(count), 0);
    std::function<bool(int)> cyclic = [&](int node) {
        color[static_cast<std::size_t>(node)] = 1;
        for (int next : edges[static_cast<std::size_t>(node)]) {
            if (color[static_cast<std::size_t>(next)] == 1) {
                return true;
            }
            if (color[static_cast<std::size_t>(next)] == 0 && cyclic(next)) {
                return true;
            }
        }
        color[static_cast<std::size_t>(node)] = 2;
        return false;
    };
    for (int i = 0; i < count; ++i) {
        if (color[static_cast<std::size_t>(i)] == 0 && cyclic(i)) {
            report.error = "operation predecessors contain a cycle";
            return report;
        }
    }

    std::vector<double> es(static_cast<std::size_t>(count), 0);
    std::vector<double> ef(static_cast<std::size_t>(count), 0);
    bool changed = true;
    int guard = 0;
    const int limit = count * count + 4;
    while (changed && guard < limit) {
        changed = false;
        ++guard;
        for (int i = 0; i < count; ++i) {
            double start = 0;
            const std::string& parent = operations[static_cast<std::size_t>(i)].parent;
            if (!parent.empty()) {
                start = es[static_cast<std::size_t>(index[parent])];
            }
            for (int pred : preds[static_cast<std::size_t>(i)]) {
                start = std::max(start, ef[static_cast<std::size_t>(pred)]);
            }
            double finish = start;
            if (operations[static_cast<std::size_t>(i)].kind == OpKind::Task) {
                finish = start + std::max(0.0, operations[static_cast<std::size_t>(i)].seconds);
            } else {
                for (int child : children[static_cast<std::size_t>(i)]) {
                    finish = std::max(finish, ef[static_cast<std::size_t>(child)]);
                }
            }
            if (start > es[static_cast<std::size_t>(i)] + 1e-9 || finish > ef[static_cast<std::size_t>(i)] + 1e-9) {
                es[static_cast<std::size_t>(i)] = start;
                ef[static_cast<std::size_t>(i)] = finish;
                changed = true;
            }
        }
    }
    if (changed) {
        report.error = "operation predecessors contain a cycle";
        return report;
    }

    double makespan = 0;
    for (double finish : ef) {
        makespan = std::max(makespan, finish);
    }
    report.makespan = makespan;

    std::vector<std::vector<int>> succs(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        for (int pred : preds[static_cast<std::size_t>(i)]) {
            succs[static_cast<std::size_t>(pred)].push_back(i);
        }
    }
    std::vector<double> latestFinish(static_cast<std::size_t>(count), makespan);
    std::vector<double> latestStart(static_cast<std::size_t>(count), makespan);
    changed = true;
    guard = 0;
    while (changed && guard < limit) {
        changed = false;
        ++guard;
        for (int i = count - 1; i >= 0; --i) {
            double finish = makespan;
            for (int succ : succs[static_cast<std::size_t>(i)]) {
                finish = std::min(finish, latestStart[static_cast<std::size_t>(succ)]);
            }
            const std::string& parent = operations[static_cast<std::size_t>(i)].parent;
            if (!parent.empty()) {
                finish = std::min(finish, latestFinish[static_cast<std::size_t>(index[parent])]);
            }
            const double span = operations[static_cast<std::size_t>(i)].kind == OpKind::Task
                                    ? std::max(0.0, operations[static_cast<std::size_t>(i)].seconds)
                                    : std::max(0.0, ef[static_cast<std::size_t>(i)] - es[static_cast<std::size_t>(i)]);
            double start = finish - span;
            for (int child : children[static_cast<std::size_t>(i)]) {
                start = std::min(start, latestStart[static_cast<std::size_t>(child)]);
            }
            if (finish < latestFinish[static_cast<std::size_t>(i)] - 1e-9 ||
                start < latestStart[static_cast<std::size_t>(i)] - 1e-9) {
                latestFinish[static_cast<std::size_t>(i)] = finish;
                latestStart[static_cast<std::size_t>(i)] = start;
                changed = true;
            }
        }
    }

    struct Load {
        double critical = 0;
        double total = 0;
        std::string name;
    };
    std::unordered_map<std::string, Load> loads;
    for (const Station& station : layout.stations) {
        loads[station.id].name = station.name;
    }
    for (int i = 0; i < count; ++i) {
        if (operations[static_cast<std::size_t>(i)].kind != OpKind::Task) {
            continue;
        }
        const bool critical = std::abs(latestStart[static_cast<std::size_t>(i)] - es[static_cast<std::size_t>(i)]) < 1e-6;
        ScheduleBar bar;
        bar.id = operations[static_cast<std::size_t>(i)].id;
        bar.name = operations[static_cast<std::size_t>(i)].name;
        bar.station = operations[static_cast<std::size_t>(i)].station;
        bar.start = es[static_cast<std::size_t>(i)];
        bar.finish = ef[static_cast<std::size_t>(i)];
        bar.critical = critical;
        report.bars.push_back(std::move(bar));
        Load& load = loads[operations[static_cast<std::size_t>(i)].station];
        const double seconds = std::max(0.0, operations[static_cast<std::size_t>(i)].seconds);
        load.total += seconds;
        if (critical) {
            load.critical += seconds;
        }
        if (load.name.empty()) {
            load.name = operations[static_cast<std::size_t>(i)].station;
        }
    }
    std::sort(report.bars.begin(), report.bars.end(), [](const ScheduleBar& a, const ScheduleBar& b) {
        if (a.start != b.start) {
            return a.start < b.start;
        }
        return a.id < b.id;
    });
    for (const ScheduleBar& bar : report.bars) {
        if (bar.critical) {
            report.criticalIds.push_back(bar.id);
        }
    }

    std::string best;
    double bestCritical = -1;
    double bestTotal = -1;
    for (const auto& [id, load] : loads) {
        if (id.empty()) {
            continue;
        }
        const bool higher = load.critical > bestCritical + 1e-9;
        const bool sameCritical = std::abs(load.critical - bestCritical) <= 1e-9;
        const bool busier = sameCritical && load.total > bestTotal + 1e-9;
        const bool sameTotal = sameCritical && std::abs(load.total - bestTotal) <= 1e-9;
        if (best.empty() || higher || busier || (sameTotal && id < best)) {
            best = id;
            bestCritical = load.critical;
            bestTotal = load.total;
        }
    }
    report.bottleneck = best;
    if (!best.empty()) {
        report.bottleneckName = loads[best].name;
    }
    return report;
}

DxfResult parseDxf(const std::string& text, double mmPerUnit) {
    DxfResult result;
    if (text.size() >= 18 && text.compare(0, 18, "AutoCAD Binary DXF") == 0) {
        result.message = "binary DXF is not supported";
        return result;
    }
    if (text.find('\0') != std::string::npos) {
        result.message = "binary DXF is not supported";
        return result;
    }
    const double scale = mmPerUnit > 1e-12 ? mmPerUnit : 1.0;
    const std::vector<std::string> lines = splitLines(text);
    if (lines.size() < 2 || lines.size() % 2 != 0) {
        result.message = "dxf group codes are incomplete";
        return result;
    }
    struct Entity {
        std::string type;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> x2;
        std::vector<double> y2;
        double radius = 0;
        int flags = 0;
        bool alive = false;
    } current;
    bool inEntities = false;
    bool pendingSection = false;
    bool sawEntities = false;
    auto flush = [&]() {
        if (!current.alive) {
            return;
        }
        if (current.type == "LINE" && !current.x.empty() && !current.y.empty() && !current.x2.empty() && !current.y2.empty()) {
            result.lines.push_back({Vec3{current.x.front() * scale, current.y.front() * scale, 0},
                                    Vec3{current.x2.front() * scale, current.y2.front() * scale, 0}});
        } else if (current.type == "LWPOLYLINE") {
            const std::size_t n = std::min(current.x.size(), current.y.size());
            for (std::size_t i = 1; i < n; ++i) {
                result.lines.push_back({Vec3{current.x[i - 1] * scale, current.y[i - 1] * scale, 0},
                                        Vec3{current.x[i] * scale, current.y[i] * scale, 0}});
            }
            if ((current.flags & 1) != 0 && n >= 2) {
                result.lines.push_back({Vec3{current.x[n - 1] * scale, current.y[n - 1] * scale, 0},
                                        Vec3{current.x.front() * scale, current.y.front() * scale, 0}});
            }
        } else if (current.type == "CIRCLE" && !current.x.empty() && !current.y.empty()) {
            addCircle(result.lines, current.x.front(), current.y.front(), current.radius, scale);
        }
        current = {};
    };
    try {
        for (std::size_t i = 0; i + 1 < lines.size(); i += 2) {
            const int code = std::stoi(trimCopy(lines[i]));
            const std::string value = trimCopy(lines[i + 1]);
            if (code == 0) {
                flush();
                if (value == "ENDSEC") {
                    inEntities = false;
                    pendingSection = false;
                    continue;
                }
                if (value == "SECTION") {
                    pendingSection = true;
                    continue;
                }
                if (value == "EOF") {
                    break;
                }
                if (inEntities) {
                    current.type = value;
                    current.alive = true;
                }
                continue;
            }
            if (code == 2 && pendingSection) {
                inEntities = value == "ENTITIES";
                sawEntities = sawEntities || inEntities;
                pendingSection = false;
                continue;
            }
            if (!current.alive) {
                continue;
            }
            if (code == 10) {
                current.x.push_back(std::stod(value));
            } else if (code == 20) {
                current.y.push_back(std::stod(value));
            } else if (code == 11) {
                current.x2.push_back(std::stod(value));
            } else if (code == 21) {
                current.y2.push_back(std::stod(value));
            } else if (code == 40) {
                current.radius = std::stod(value);
            } else if (code == 70) {
                current.flags = std::stoi(value);
            }
        }
        flush();
    } catch (const std::exception&) {
        result.lines.clear();
        result.message = "dxf group codes are incomplete";
        return result;
    }
    if (!sawEntities) {
        result.message = "dxf has no entities section";
        return result;
    }
    result.ok = true;
    result.message.clear();
    return result;
}

bool loadDxfUnderlay(LineLayout& layout, const std::string& path, double mmPerUnit, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    const DxfResult parsed = parseDxf(buffer.str(), mmPerUnit);
    if (!parsed.ok) {
        error = parsed.message.empty() ? "dxf was not read" : parsed.message;
        return false;
    }
    layout.underlay.kind = "dxf";
    layout.underlay.path = path;
    layout.underlay.mmPerUnit = mmPerUnit > 1e-12 ? mmPerUnit : 1;
    layout.underlay.lines = parsed.lines;
    error.clear();
    return true;
}

void setImageUnderlay(LineLayout& layout, const std::string& path, double opacity) {
    layout.underlay.kind = "image";
    layout.underlay.path = path;
    layout.underlay.opacity = std::clamp(opacity, 0.0, 1.0);
    layout.underlay.lines.clear();
}

EmsImportResult importEmsXml(const std::string& path, Scene& scene) {
    (void)path;
    (void)scene;
    EmsImportResult result;
    result.ok = false;
    result.message = "eMS/XML import is reserved and does not read the file";
    return result;
}

bool exportScheduleCsv(const ScheduleReport& report, const std::string& path, std::string& error) {
    std::ofstream out(path);
    if (!out) {
        error = "cannot write " + path;
        return false;
    }
    out << "station,operation,start_s,finish_s,critical,bottleneck\n";
    out.setf(std::ios::fixed);
    out.precision(3);
    for (const ScheduleBar& bar : report.bars) {
        out << csvField(bar.station) << ',' << csvField(bar.name) << ',' << bar.start << ',' << bar.finish << ','
            << (bar.critical ? "yes" : "no") << ',' << (bar.station == report.bottleneck ? "yes" : "no") << '\n';
    }
    if (!out) {
        error = "failed while writing " + path;
        return false;
    }
    error.clear();
    return true;
}

bool exportScheduleXlsx(const ScheduleReport& report, const std::string& path, std::string& error) {
    const char* contentTypes =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
        "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
        "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
        "<Default Extension=\"xml\" ContentType=\"application/xml\"/>"
        "<Override PartName=\"/xl/workbook.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml\"/>"
        "<Override PartName=\"/xl/worksheets/sheet1.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>"
        "<Override PartName=\"/xl/styles.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml\"/>"
        "</Types>";
    const char* rels =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
        "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
        "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument\" Target=\"xl/workbook.xml\"/>"
        "</Relationships>";
    const char* workbook =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
        "<workbook xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" "
        "xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\">"
        "<sheets><sheet name=\"cycle\" sheetId=\"1\" r:id=\"rId1\"/></sheets></workbook>";
    const char* workbookRels =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
        "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
        "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"worksheets/sheet1.xml\"/>"
        "<Relationship Id=\"rId2\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles\" Target=\"styles.xml\"/>"
        "</Relationships>";
    const char* styles =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
        "<styleSheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"
        "<fonts count=\"1\"><font><sz val=\"11\"/><name val=\"Calibri\"/></font></fonts>"
        "<fills count=\"1\"><fill><patternFill patternType=\"none\"/></fill></fills>"
        "<borders count=\"1\"><border/></borders>"
        "<cellStyleXfs count=\"1\"><xf/></cellStyleXfs>"
        "<cellXfs count=\"1\"><xf/></cellXfs>"
        "</styleSheet>";
    std::ostringstream sheet;
    sheet << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
          << "<worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\"><sheetData>";
    sheet << "<row r=\"1\">" << inlineCell(0, 1, "station") << inlineCell(1, 1, "operation") << inlineCell(2, 1, "start_s")
          << inlineCell(3, 1, "finish_s") << inlineCell(4, 1, "critical") << inlineCell(5, 1, "bottleneck") << "</row>";
    int row = 2;
    for (const ScheduleBar& bar : report.bars) {
        sheet << "<row r=\"" << row << "\">" << inlineCell(0, row, bar.station) << inlineCell(1, row, bar.name)
              << numberCell(2, row, bar.start) << numberCell(3, row, bar.finish)
              << inlineCell(4, row, bar.critical ? "yes" : "no")
              << inlineCell(5, row, bar.station == report.bottleneck ? "yes" : "no") << "</row>";
        ++row;
    }
    sheet << "</sheetData></worksheet>";
    return writeStoreZip(path,
                         {{"[Content_Types].xml", contentTypes},
                          {"_rels/.rels", rels},
                          {"xl/workbook.xml", workbook},
                          {"xl/_rels/workbook.xml.rels", workbookRels},
                          {"xl/styles.xml", styles},
                          {"xl/worksheets/sheet1.xml", sheet.str()}},
                         error);
}

nlohmann::json layoutToJson(const LineLayout& layout) {
    nlohmann::json stations = nlohmann::json::array();
    for (const Station& station : layout.stations) {
        stations.push_back({{"id", station.id},
                            {"name", station.name},
                            {"x", station.x},
                            {"y", station.y},
                            {"width", station.width},
                            {"depth", station.depth},
                            {"kind", stationKindName(station.kind)},
                            {"resource", station.resource},
                            {"node", station.nodeName}});
    }
    nlohmann::json lines = nlohmann::json::array();
    for (const auto& segment : layout.underlay.lines) {
        lines.push_back({segment.first.x, segment.first.y, segment.second.x, segment.second.y});
    }
    return {{"example", layout.example},
            {"title", layout.title},
            {"gridMm", layout.gridMm},
            {"stations", std::move(stations)},
            {"underlay",
             {{"kind", layout.underlay.kind},
              {"path", layout.underlay.path},
              {"opacity", layout.underlay.opacity},
              {"mmPerUnit", layout.underlay.mmPerUnit},
              {"lines", std::move(lines)}}}};
}

void layoutFromJson(const nlohmann::json& json, LineLayout& layout) {
    layout = {};
    if (!json.is_object()) {
        return;
    }
    layout.example = json.value("example", false);
    layout.title = json.value("title", std::string{});
    layout.gridMm = json.value("gridMm", 500.0);
    if (json.contains("stations") && json.at("stations").is_array()) {
        for (const auto& item : json.at("stations")) {
            if (!item.is_object()) {
                continue;
            }
            Station station;
            station.id = item.value("id", std::string{});
            station.name = item.value("name", std::string{});
            station.x = item.value("x", 0.0);
            station.y = item.value("y", 0.0);
            station.width = item.value("width", 2000.0);
            station.depth = item.value("depth", 2000.0);
            station.resource = item.value("resource", std::string{});
            station.nodeName = item.value("node", std::string{});
            StationKind kind = StationKind::Manual;
            if (stationKindFromName(item.value("kind", std::string{"manual"}), kind)) {
                station.kind = kind;
            }
            if (!station.id.empty()) {
                layout.stations.push_back(std::move(station));
            }
        }
    }
    if (json.contains("underlay") && json.at("underlay").is_object()) {
        const auto& underlay = json.at("underlay");
        layout.underlay.kind = underlay.value("kind", std::string{});
        layout.underlay.path = underlay.value("path", std::string{});
        layout.underlay.opacity = underlay.value("opacity", 0.35);
        layout.underlay.mmPerUnit = underlay.value("mmPerUnit", 1.0);
        if (underlay.contains("lines") && underlay.at("lines").is_array()) {
            for (const auto& line : underlay.at("lines")) {
                if (line.is_array() && line.size() >= 4) {
                    layout.underlay.lines.push_back(
                        {Vec3{line[0].get<double>(), line[1].get<double>(), 0}, Vec3{line[2].get<double>(), line[3].get<double>(), 0}});
                }
            }
        }
    }
}

nlohmann::json operationsToJson(const std::vector<Operation>& operations) {
    nlohmann::json json = nlohmann::json::array();
    for (const Operation& operation : operations) {
        json.push_back({{"id", operation.id},
                        {"name", operation.name},
                        {"station", operation.station},
                        {"parent", operation.parent},
                        {"kind", opKindName(operation.kind)},
                        {"index", operation.index},
                        {"seconds", operation.seconds},
                        {"after", operation.after},
                        {"path", operation.path},
                        {"logic", operation.logic}});
    }
    return json;
}

void operationsFromJson(const nlohmann::json& json, std::vector<Operation>& operations) {
    operations.clear();
    if (!json.is_array()) {
        return;
    }
    for (const auto& item : json) {
        if (!item.is_object()) {
            continue;
        }
        Operation operation;
        operation.id = item.value("id", std::string{});
        operation.name = item.value("name", std::string{});
        operation.station = item.value("station", std::string{});
        operation.parent = item.value("parent", std::string{});
        operation.index = item.value("index", 0);
        operation.seconds = item.value("seconds", 1.0);
        operation.path = item.value("path", std::string{});
        operation.logic = item.value("logic", std::string{});
        OpKind kind = OpKind::Task;
        if (opKindFromName(item.value("kind", std::string{"task"}), kind)) {
            operation.kind = kind;
        }
        if (item.contains("after") && item.at("after").is_array()) {
            for (const auto& after : item.at("after")) {
                if (after.is_string()) {
                    operation.after.push_back(after.get<std::string>());
                }
            }
        }
        if (!operation.id.empty()) {
            operations.push_back(std::move(operation));
        }
    }
}

nlohmann::json pprToJson(const PprDocument& ppr) {
    nlohmann::json products = nlohmann::json::array();
    for (const ProductNode& node : ppr.products) {
        products.push_back({{"id", node.id}, {"name", node.name}, {"parent", node.parent}, {"part", node.partName}});
    }
    nlohmann::json processes = nlohmann::json::array();
    for (const ProcessNode& node : ppr.processes) {
        processes.push_back({{"id", node.id},
                             {"name", node.name},
                             {"parent", node.parent},
                             {"operation", node.operation},
                             {"product", node.product}});
    }
    nlohmann::json plants = nlohmann::json::array();
    for (const PlantNode& node : ppr.plants) {
        plants.push_back({{"id", node.id},
                          {"name", node.name},
                          {"parent", node.parent},
                          {"station", node.station},
                          {"resource", node.resource}});
    }
    return {{"example", ppr.example},
            {"title", ppr.title},
            {"products", std::move(products)},
            {"processes", std::move(processes)},
            {"plants", std::move(plants)}};
}

void pprFromJson(const nlohmann::json& json, PprDocument& ppr) {
    ppr = {};
    if (!json.is_object()) {
        return;
    }
    ppr.example = json.value("example", false);
    ppr.title = json.value("title", std::string{});
    if (json.contains("products") && json.at("products").is_array()) {
        for (const auto& item : json.at("products")) {
            ProductNode node;
            node.id = item.value("id", std::string{});
            node.name = item.value("name", std::string{});
            node.parent = item.value("parent", std::string{});
            node.partName = item.value("part", std::string{});
            if (!node.id.empty()) {
                ppr.products.push_back(std::move(node));
            }
        }
    }
    if (json.contains("processes") && json.at("processes").is_array()) {
        for (const auto& item : json.at("processes")) {
            ProcessNode node;
            node.id = item.value("id", std::string{});
            node.name = item.value("name", std::string{});
            node.parent = item.value("parent", std::string{});
            node.operation = item.value("operation", std::string{});
            node.product = item.value("product", std::string{});
            if (!node.id.empty()) {
                ppr.processes.push_back(std::move(node));
            }
        }
    }
    if (json.contains("plants") && json.at("plants").is_array()) {
        for (const auto& item : json.at("plants")) {
            PlantNode node;
            node.id = item.value("id", std::string{});
            node.name = item.value("name", std::string{});
            node.parent = item.value("parent", std::string{});
            node.station = item.value("station", std::string{});
            node.resource = item.value("resource", std::string{});
            if (!node.id.empty()) {
                ppr.plants.push_back(std::move(node));
            }
        }
    }
}

nlohmann::json scheduleToJson(const ScheduleReport& report) {
    nlohmann::json bars = nlohmann::json::array();
    for (const ScheduleBar& bar : report.bars) {
        bars.push_back({{"id", bar.id},
                        {"name", bar.name},
                        {"station", bar.station},
                        {"start", bar.start},
                        {"finish", bar.finish},
                        {"critical", bar.critical}});
    }
    return {{"makespan", report.makespan},
            {"bottleneck", report.bottleneck},
            {"bottleneckName", report.bottleneckName},
            {"critical", report.criticalIds},
            {"bars", std::move(bars)},
            {"error", report.error}};
}

}  // namespace sim
