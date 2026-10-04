#include "sim/ai/Commands.hpp"
#include "sim/core/DemoScene.hpp"
#include "sim/core/LineModel.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/io/ZipArchive.hpp"
#include "sim/scripting/ScriptHost.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

using Catch::Approx;

namespace {

sim::Operation task(const std::string& id, const std::string& parent, int index, const std::string& station, double seconds) {
    sim::Operation operation;
    operation.id = id;
    operation.name = id;
    operation.parent = parent;
    operation.index = index;
    operation.station = station;
    operation.kind = sim::OpKind::Task;
    operation.seconds = seconds;
    return operation;
}

}  // namespace

TEST_CASE("stations snap, align, and distribute on the plan grid", "[line]") {
    sim::LineLayout layout;
    layout.gridMm = 500;
    sim::Station left;
    left.id = "left";
    left.x = 1230;
    left.y = 260;
    left.width = 2000;
    sim::Station mid;
    mid.id = "mid";
    mid.x = 1800;
    mid.y = 40;
    mid.width = 1000;
    sim::Station right;
    right.id = "right";
    right.x = 4100;
    right.y = -80;
    right.width = 2000;
    layout.stations = {left, mid, right};
    sim::snapStations(layout);
    REQUIRE(sim::findStation(layout, "left")->x == Approx(1000));
    REQUIRE(sim::findStation(layout, "left")->y == Approx(500));

    layout.stations[0].x = 1000;
    layout.stations[0].width = 2000;
    layout.stations[1].x = 4000;
    layout.stations[1].width = 1000;
    layout.stations[2].x = 7000;
    layout.stations[2].width = 2000;
    sim::alignStationsMinX(layout);
    REQUIRE(sim::findStation(layout, "left")->x == Approx(1000));
    REQUIRE(sim::findStation(layout, "mid")->x == Approx(500));
    REQUIRE(sim::findStation(layout, "right")->x == Approx(1000));

    layout.stations[0].x = 0;
    layout.stations[1].x = 100;
    layout.stations[2].x = 1000;
    layout.gridMm = 100;
    sim::distributeStationsX(layout);
    REQUIRE(sim::findStation(layout, "left")->x == Approx(0));
    REQUIRE(sim::findStation(layout, "mid")->x == Approx(500));
    REQUIRE(sim::findStation(layout, "right")->x == Approx(1000));
}

TEST_CASE("ascii dxf lines and polylines become plan segments", "[line]") {
    const char* text = R"(0
SECTION
2
ENTITIES
0
LINE
10
0
20
10
11
30
21
40
0
LWPOLYLINE
70
1
10
0
20
0
10
10
20
0
10
10
20
5
0
CIRCLE
10
0
20
0
40
2
0
ENDSEC
0
EOF
)";
    const sim::DxfResult parsed = sim::parseDxf(text, 2);
    REQUIRE(parsed.ok);
    REQUIRE(parsed.lines.size() == 1 + 3 + 16);
    REQUIRE(parsed.lines.front().second.x == Approx(60));
    REQUIRE(parsed.lines.front().second.y == Approx(80));

    const sim::DxfResult binary = sim::parseDxf(std::string("AutoCAD Binary DXF\r\n\0", 22), 1);
    REQUIRE_FALSE(binary.ok);
    const sim::DxfResult garbage = sim::parseDxf("not a drawing", 1);
    REQUIRE_FALSE(garbage.ok);
}

TEST_CASE("sequence and parallel operations expose the critical path", "[line]") {
    sim::LineLayout layout;
    layout.stations.push_back(sim::Station{"s1", "A", 0, 0, 1000, 1000, sim::StationKind::Manual, {}, {}});
    layout.stations.push_back(sim::Station{"s2", "B", 0, 0, 1000, 1000, sim::StationKind::Manual, {}, {}});
    layout.stations.push_back(sim::Station{"s3", "C", 0, 0, 1000, 1000, sim::StationKind::Manual, {}, {}});
    std::vector<sim::Operation> operations;
    sim::Operation seq;
    seq.id = "seq";
    seq.kind = sim::OpKind::Sequence;
    sim::Operation par;
    par.id = "par";
    par.kind = sim::OpKind::Parallel;
    par.after = {"seq"};
    operations.push_back(seq);
    operations.push_back(par);
    operations.push_back(task("a", "seq", 0, "s1", 10));
    operations.push_back(task("b", "seq", 1, "s2", 4));
    operations.push_back(task("c", "par", 0, "s1", 6));
    operations.push_back(task("d", "par", 1, "s3", 3));
    sim::Operation tail = task("e", "", 0, "s2", 2);
    tail.after = {"par"};
    operations.push_back(tail);

    const sim::ScheduleReport report = sim::scheduleOperations(operations, layout);
    REQUIRE(report.error.empty());
    REQUIRE(report.makespan == Approx(22));
    REQUIRE(report.bottleneck == "s1");
    REQUIRE(report.criticalIds == std::vector<std::string>{"a", "b", "c", "e"});

    operations[2].after = {"b"};
    operations[3].after = {"a"};
    const sim::ScheduleReport cycle = sim::scheduleOperations(operations, layout);
    REQUIRE_FALSE(cycle.error.empty());
    REQUIRE(cycle.bars.empty());
}

TEST_CASE("cycle report writes csv and a spreadsheet", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    const sim::ScheduleReport report = sim::scheduleOperations(scene.operations, scene.layout);
    REQUIRE(report.makespan == Approx(32));
    REQUIRE(report.bottleneck == "st_weld");
    const auto csvPath = std::filesystem::temp_directory_path() / "simstudio_cycle.csv";
    const auto xlsxPath = std::filesystem::temp_directory_path() / "simstudio_cycle.xlsx";
    std::string error;
    REQUIRE(sim::exportScheduleCsv(report, csvPath.string(), error));
    std::ifstream csv(csvPath);
    std::string header;
    std::getline(csv, header);
    REQUIRE(header == "station,operation,start_s,finish_s,critical,bottleneck");
    const std::string body((std::istreambuf_iterator<char>(csv)), std::istreambuf_iterator<char>());
    REQUIRE(body.find("焊接") != std::string::npos);
    REQUIRE(body.find("st_weld") != std::string::npos);
    REQUIRE(sim::exportScheduleXlsx(report, xlsxPath.string(), error));
    std::vector<sim::ZipEntry> entries;
    REQUIRE(sim::ZipArchive::read(xlsxPath.string(), entries, error));
    const auto sheet = std::find_if(entries.begin(), entries.end(), [](const sim::ZipEntry& entry) {
        return entry.name == "xl/worksheets/sheet1.xml";
    });
    REQUIRE(sheet != entries.end());
    const std::string xml(sheet->data.begin(), sheet->data.end());
    REQUIRE(xml.find("焊接") != std::string::npos);
}

TEST_CASE("ppr xml import and project version 9 keep the line", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    const std::size_t stations = scene.layout.stations.size();
    const sim::EmsImportResult missing = sim::importEmsXml("missing.ems.xml", scene);
    REQUIRE_FALSE(missing.ok);
    REQUIRE(missing.message.find("cannot open") != std::string::npos);
    REQUIRE(scene.layout.stations.size() == stations);

    const auto xmlPath = std::filesystem::temp_directory_path() / "simstudio_ppr.xml";
    {
        std::ofstream out(xmlPath);
        out << "<Plant name=\"cell\"><Product id=\"p1\" name=\"bracket\"/>"
               "<Process id=\"w1\" name=\"weld\" duration=\"3.5\" station=\"st1\"/>"
               "<Resource id=\"r1\" name=\"robot\" station=\"st1\" resource=\"kr420\"/></Plant>";
    }
    sim::Scene parsed;
    const sim::EmsImportResult imported = sim::importEmsXml(xmlPath.string(), parsed);
    REQUIRE(imported.ok);
    REQUIRE(parsed.ppr.products.size() == 1);
    REQUIRE(parsed.ppr.processes.size() == 1);
    REQUIRE(parsed.ppr.plants.size() == 2);
    REQUIRE(parsed.operations.size() == 1);
    REQUIRE(parsed.operations.front().seconds == Approx(3.5));
    sim::Scene garbage;
    garbage.layout.stations.push_back(sim::Station{});
    const auto badPath = std::filesystem::temp_directory_path() / "simstudio_bad.xml";
    {
        std::ofstream out(badPath);
        out << "<Note>hello</Note>";
    }
    const sim::EmsImportResult rejected = sim::importEmsXml(badPath.string(), garbage);
    REQUIRE_FALSE(rejected.ok);
    REQUIRE(garbage.layout.stations.size() == 1);
    REQUIRE(garbage.ppr.products.empty());

    const auto path = std::filesystem::temp_directory_path() / "simstudio_line.simproj";
    REQUIRE(sim::ProjectStore::save(scene, path.string()).ok);
    sim::Scene loaded;
    REQUIRE(sim::ProjectStore::load(path.string(), loaded).ok);
    REQUIRE(loaded.layout.example);
    REQUIRE(loaded.layout.stations.size() == 4);
    REQUIRE(loaded.operations.size() == scene.operations.size());
    REQUIRE(loaded.ppr.plants.size() == 1);
    REQUIRE(loaded.ppr.plants.front().resource == "kr420");

    sim::ZipEntry project;
    project.name = "project.json";
    const std::string json = R"({"format":"simstudio.project","version":7,"name":"old","root":{"id":"root","name":"World","kind":"assembly","children":[]}})";
    project.data.assign(json.begin(), json.end());
    const auto oldPath = std::filesystem::temp_directory_path() / "simstudio_v7.simproj";
    std::string error;
    REQUIRE(sim::ZipArchive::write(oldPath.string(), {project}, error));
    sim::Scene old;
    old.layout.stations.push_back(sim::Station{});
    REQUIRE(sim::ProjectStore::load(oldPath.string(), old).ok);
    REQUIRE(old.layout.stations.empty());
    REQUIRE(old.operations.empty());
    REQUIRE(old.ppr.products.empty());
}

TEST_CASE("example weld line links the kr420 path and logic block", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    REQUIRE(scene.layout.example);
    REQUIRE(scene.layout.title == "示例焊装线");
    REQUIRE(scene.layout.stations.size() == 4);
    const sim::Station* weld = sim::findStation(scene.layout, "st_weld");
    REQUIRE(weld != nullptr);
    REQUIRE(weld->resource == "kr420");
    REQUIRE(weld->nodeName == "KR420");
    REQUIRE(scene.findByName("KR420") != nullptr);
    bool linked = false;
    for (const sim::Operation& operation : scene.operations) {
        if (operation.id == "weld") {
            linked = operation.path == "cycle" && operation.logic == "weld_seq" && operation.seconds == Approx(12);
        }
    }
    REQUIRE(linked);
    REQUIRE(scene.robotPaths.front().name == "cycle");
    REQUIRE(scene.logicBlocks.front().name == "weld_seq");
    const sim::ScheduleReport report = sim::scheduleOperations(scene.operations, scene.layout);
    REQUIRE(report.bottleneckName == "焊接");
    REQUIRE(std::find(report.criticalIds.begin(), report.criticalIds.end(), "prep") == report.criticalIds.end());
}

TEST_CASE("line commands and python read the same schedule", "[line]") {
    sim::Scene scene;
    sim::CommandExecutor commands(scene, nullptr, nullptr);
    const auto catalog = sim::CommandExecutor::catalog();
    bool listed = false;
    for (const auto& entry : catalog) {
        if (entry.value("op", "") == "line_schedule") {
            listed = entry.value("executable", false);
        }
    }
    REQUIRE(listed);
    REQUIRE(commands.execute({{"op", "line_add_station"}, {"id", "s1"}, {"name", "Load"}, {"x", 10}, {"y", 20}, {"kind", "manual"}}).ok);
    REQUIRE(commands.execute({{"op", "line_add_op"},
                              {"id", "job"},
                              {"name", "Load"},
                              {"station", "s1"},
                              {"kind", "task"},
                              {"seconds", 4}})
                .ok);
    const sim::CommandResult schedule = commands.execute({{"op", "line_schedule"}});
    REQUIRE(schedule.ok);
    REQUIRE(schedule.value.at("makespan").get<double>() == Approx(4));
    REQUIRE(schedule.value.at("bottleneck").get<std::string>() == "s1");
    const sim::CommandResult reserved = commands.execute({{"op", "ems_import"}, {"path", "cell.xml"}});
    REQUIRE_FALSE(reserved.ok);
    REQUIRE(scene.layout.stations.size() == 1);

    auto host = sim::createScriptHost();
    host->bind(&scene);
    std::string output;
    std::string error;
    REQUIRE(host->execute("stations = sim.line_stations()\nreport = sim.line_schedule()\ncounts = sim.ppr_counts()\n"
                          "ems = sim.ems_import('cell.xml')\n"
                          "print(len(stations), report['makespan'], report['bottleneck'], ems['ok'], counts['example'])\n",
                          output, error));
    REQUIRE(error.empty());
    REQUIRE(output.find("1 4.0 s1 False") != std::string::npos);
}

TEST_CASE("gantt drag writes wait and a pinned finish", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    REQUIRE(sim::adjustOperationStart(scene.operations, scene.layout, "unload", 30));
    sim::Operation* unload = nullptr;
    for (sim::Operation& operation : scene.operations) {
        if (operation.id == "unload") {
            unload = &operation;
        }
    }
    REQUIRE(unload != nullptr);
    REQUIRE(unload->wait > 1.0);
    const double before = sim::scheduleOperations(scene.operations, scene.layout).makespan;
    REQUIRE(sim::adjustOperationFinish(scene.operations, scene.layout, "unload", 40));
    REQUIRE(unload->timeSource == "pinned");
    REQUIRE(unload->timeNote == "gantt finish");
    const sim::ScheduleReport after = sim::scheduleOperations(scene.operations, scene.layout);
    REQUIRE(after.makespan > before);
}

TEST_CASE("dxf arc spline insert and a binary line", "[line]") {
    const char* text = R"(0
SECTION
2
BLOCKS
0
BLOCK
2
MARK
0
LINE
8
Marks
10
0
20
0
11
10
21
0
0
ENDBLK
0
ENDSEC
0
SECTION
2
ENTITIES
0
ARC
8
Floor
10
0
20
0
40
10
50
0
51
90
0
SPLINE
8
Curve
71
3
10
0
20
0
10
10
20
0
10
10
20
10
10
0
20
10
0
INSERT
8
Placed
2
MARK
10
100
20
50
0
ENDSEC
0
EOF
)";
    const sim::DxfResult parsed = sim::parseDxf(text, 1);
    REQUIRE(parsed.ok);
    REQUIRE(parsed.lines.size() > 4);
    bool sawFloor = false;
    bool sawPlaced = false;
    for (const std::string& layer : parsed.layers) {
        sawFloor = sawFloor || layer == "Floor";
        sawPlaced = sawPlaced || layer == "Placed";
    }
    REQUIRE(sawFloor);
    REQUIRE(sawPlaced);

    std::string binary("AutoCAD Binary DXF\r\n\0", 22);
    auto pushCode = [&](int code) {
        binary.push_back(static_cast<char>(code & 0xff));
        binary.push_back(static_cast<char>((code >> 8) & 0xff));
    };
    auto pushString = [&](int code, const char* value) {
        pushCode(code);
        binary.append(value);
        binary.push_back('\0');
    };
    auto pushDouble = [&](int code, double value) {
        pushCode(code);
        binary.append(reinterpret_cast<const char*>(&value), sizeof(value));
    };
    pushString(0, "SECTION");
    pushString(2, "ENTITIES");
    pushString(0, "LINE");
    pushString(8, "Bin");
    pushDouble(10, 0);
    pushDouble(20, 0);
    pushDouble(11, 25);
    pushDouble(21, 0);
    pushString(0, "ENDSEC");
    pushString(0, "EOF");
    const sim::DxfResult binaryResult = sim::parseDxf(binary, 1);
    REQUIRE(binaryResult.ok);
    REQUIRE(binaryResult.lines.size() == 1);
    REQUIRE(binaryResult.layers.front() == "Bin");
    REQUIRE(binaryResult.lines.front().second.x == Approx(25));

    const char* extra = R"(0
SECTION
2
ENTITIES
0
3DFACE
8
Solid
10
0
20
0
30
0
11
10
21
0
31
5
12
10
22
10
32
5
13
0
23
10
33
0
0
ELLIPSE
8
Solid
10
0
20
0
11
20
21
0
40
0.5
41
0
42
6.283185
0
HATCH
8
Fill
93
3
10
0
20
0
10
30
20
0
10
30
20
15
0
DIMENSION
8
Notes
10
0
20
5
11
15
21
5
13
0
23
0
14
40
24
0
0
ENDSEC
0
EOF
)";
    const sim::DxfResult drawn = sim::parseDxf(extra, 1);
    REQUIRE(drawn.ok);
    bool sawHeight = false;
    bool sawSpan = false;
    int solid = 0;
    int fill = 0;
    int notes = 0;
    for (std::size_t i = 0; i < drawn.lines.size(); ++i) {
        const std::string layer = i < drawn.layers.size() ? drawn.layers[i] : "";
        solid += layer == "Solid" ? 1 : 0;
        fill += layer == "Fill" ? 1 : 0;
        notes += layer == "Notes" ? 1 : 0;
        if (std::abs(drawn.lines[i].first.z) > 1 || std::abs(drawn.lines[i].second.z) > 1) {
            sawHeight = true;
        }
        const double span = std::abs(drawn.lines[i].second.x - drawn.lines[i].first.x);
        if (layer == "Notes" && span > 30) {
            sawSpan = true;
        }
    }
    REQUIRE(solid > 8);
    REQUIRE(fill >= 3);
    REQUIRE(notes >= 1);
    REQUIRE(sawHeight);
    REQUIRE(sawSpan);
}
