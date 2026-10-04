#pragma once

#include "sim/core/Math.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <utility>
#include <vector>

namespace sim {

class Scene;

enum class StationKind { Robot, Fixture, Conveyor, Manual, Buffer };
enum class OpKind { Task, Sequence, Parallel };

struct Station {
    std::string id;
    std::string name;
    double x = 0;
    double y = 0;
    double width = 2000;
    double depth = 2000;
    StationKind kind = StationKind::Manual;
    std::string resource;
    std::string nodeName;
};

struct Underlay {
    // "", "image", or "dxf".
    std::string kind;
    std::string path;
    double opacity = 0.35;
    double mmPerUnit = 1;
    // Plan segments in millimeters. `layers` matches `lines` when a DXF entity named one.
    std::vector<std::pair<Vec3, Vec3>> lines;
    std::vector<std::string> layers;
    // Image bytes live in the project zip when `embedded` is set. `path` is the source file.
    std::string embedded;
    std::vector<std::uint8_t> bytes;
};

struct LineLayout {
    bool example = false;
    // The operation list is one job. A cyclic line releases the next job when the first station is free.
    bool cyclic = false;
    std::string title;
    double gridMm = 500;
    Underlay underlay;
    std::vector<Station> stations;
};

struct Operation {
    std::string id;
    std::string name;
    std::string station;
    std::string parent;
    OpKind kind = OpKind::Task;
    int index = 0;
    // Work duration for a task. Groups take their duration from their children.
    double seconds = 1;
    // Extra delay after predecessors. A gantt drag of the bar start writes this.
    double wait = 0;
    std::vector<std::string> after;
    std::string path;
    std::string logic;
    // plan, logic, conveyor, pinned, or stored. Empty means the authored number.
    std::string timeSource;
    std::string timeKey;
    std::string timeNote;
};

struct ProductNode {
    std::string id;
    std::string name;
    std::string parent;
    std::string partName;
};

struct ProcessNode {
    std::string id;
    std::string name;
    std::string parent;
    std::string operation;
    std::string product;
};

struct PlantNode {
    std::string id;
    std::string name;
    std::string parent;
    std::string station;
    std::string resource;
};

struct PprDocument {
    bool example = false;
    std::string title;
    std::vector<ProductNode> products;
    std::vector<ProcessNode> processes;
    std::vector<PlantNode> plants;
};

struct ScheduleBar {
    std::string id;
    std::string name;
    std::string station;
    double start = 0;
    double finish = 0;
    bool critical = false;
};

struct ScheduleReport {
    double makespan = 0;
    std::string bottleneck;
    std::string bottleneckName;
    std::vector<ScheduleBar> bars;
    std::vector<std::string> criticalIds;
    std::string error;
};

struct DxfResult {
    bool ok = false;
    std::string message;
    std::vector<std::pair<Vec3, Vec3>> lines;
    std::vector<std::string> layers;
};

struct EmsImportResult {
    bool ok = false;
    std::string message;
};

const char* stationKindName(StationKind kind);
bool stationKindFromName(const std::string& name, StationKind& out);
const char* opKindName(OpKind kind);
bool opKindFromName(const std::string& name, OpKind& out);

double snapToGrid(double value, double gridMm);
void snapStations(LineLayout& layout);
// Align station left edges to the leftmost edge, then snap anchors to the grid.
void alignStationsMinX(LineLayout& layout);
// Spread station anchors evenly between the current leftmost and rightmost, then snap.
void distributeStationsX(LineLayout& layout);

Station* findStation(LineLayout& layout, const std::string& id);
const Station* findStation(const LineLayout& layout, const std::string& id);

// Finish-start graph: explicit `after`, plus the previous sibling when the parent is a sequence.
// A cycle sets error and leaves the bars empty.
ScheduleReport scheduleOperations(const std::vector<Operation>& operations, const LineLayout& layout);

// ASCII and binary DXF: LINE, LWPOLYLINE, CIRCLE, ARC, SPLINE, INSERT, and layer names.
DxfResult parseDxf(const std::string& text, double mmPerUnit);
// Move a bar's start by writing `wait`, or its finish by writing `seconds` and pinning it.
bool adjustOperationStart(std::vector<Operation>& operations, const LineLayout& layout, const std::string& id, double newStart);
bool adjustOperationFinish(std::vector<Operation>& operations, const LineLayout& layout, const std::string& id, double newFinish);
bool loadDxfUnderlay(LineLayout& layout, const std::string& path, double mmPerUnit, std::string& error);
void setImageUnderlay(LineLayout& layout, const std::string& path, double opacity);

// Generic product/process/plant XML. Unknown roots fail and do not invent nodes.
EmsImportResult importEmsXml(const std::string& path, Scene& scene);

bool exportScheduleCsv(const ScheduleReport& report, const std::string& path, std::string& error);
bool exportScheduleXlsx(const ScheduleReport& report, const std::string& path, std::string& error);

nlohmann::json layoutToJson(const LineLayout& layout);
void layoutFromJson(const nlohmann::json& json, LineLayout& layout);
nlohmann::json operationsToJson(const std::vector<Operation>& operations);
void operationsFromJson(const nlohmann::json& json, std::vector<Operation>& operations);
nlohmann::json pprToJson(const PprDocument& ppr);
void pprFromJson(const nlohmann::json& json, PprDocument& ppr);
nlohmann::json scheduleToJson(const ScheduleReport& report);

}  // namespace sim
